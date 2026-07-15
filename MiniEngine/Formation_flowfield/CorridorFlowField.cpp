#include "CorridorFlowField.h"
#include "GridNeighbors.h"
#include "ChunkKey.h"
#include <queue>
#include <functional>   // std::greater
#include <cmath>

CorridorFlowField::FlowFieldChunk::ColumnData* CorridorFlowField::FindOrCreateColumn(
    const VoxelGrid& grid, int x, int y, int z,
    int& outSlotIdx, ChunkCache& cache) 
{
    int cx = x / FlowFieldChunk::SIZE;
    int cz = z / FlowFieldChunk::SIZE;
    int64_t key = MakeChunkKey(cx, 0, cz);  // y값은 청크 좌표 관여 x

    if (key != cache.key)
    {
        auto it = m_Chunks.find(key);
        if (it == m_Chunks.end())
            it = m_Chunks.emplace(key, std::make_unique<FlowFieldChunk>()).first;

        cache.key = key;
        cache.chunk = it->second.get();
    }

    outSlotIdx = FindSurfaceSlot(grid, x, y, z);
    if (outSlotIdx < 0) return nullptr;             // 슬롯 없으면 컬럼도 의미 없음

    int lx = x % FlowFieldChunk::SIZE;
    int lz = z % FlowFieldChunk::SIZE;
    return &cache.chunk->At(lx, lz);
}

const CorridorFlowField::FlowFieldChunk::ColumnData* CorridorFlowField::FindColumn(
    const VoxelGrid& grid, int x, int y, int z,
    int& outSlotIdx, ChunkCache& cache) const 
{
    int cx = x / FlowFieldChunk::SIZE;
    int cz = z / FlowFieldChunk::SIZE;
    int64_t key = MakeChunkKey(cx, 0, cz);

    if (key != cache.key)
    {
        auto it = m_Chunks.find(key);
        if (it == m_Chunks.end()) 
        { 
            cache.key = -1; 
            outSlotIdx = -1;
            return nullptr; 
        }
        cache.key = key;
        cache.chunk = const_cast<FlowFieldChunk*>(it->second.get());
    }

    outSlotIdx = FindSurfaceSlot(grid, x, y, z);
    if (outSlotIdx < 0) return nullptr;             // 슬롯 없으면 컬럼도 의미 없음

    int lx = x % FlowFieldChunk::SIZE;
    int lz = z % FlowFieldChunk::SIZE;
    return &cache.chunk->At(lx, lz);
}

void CorridorFlowField::Build(const VoxelGrid& grid, const DirectX::XMINT3& goal, const std::unordered_set<int64_t>& mask)
{
    m_Chunks.clear();

    struct OpenEntry
    {
        DirectX::XMINT3 pos;
        float cost;
        bool operator>(const OpenEntry& o) const { return cost > o.cost; }
    };
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> openList;

    ChunkCache chunkcache;

    // 기준 시작점이 목적지로 -> goal에서 퍼저나가면서 방향을 세기기 위해
    // 따라서 목적지의 비용인 0으로 초기화
    int goalIdx;
    FlowFieldChunk::ColumnData* goalChunk = FindOrCreateColumn(grid, goal.x, goal.y, goal.z, goalIdx, chunkcache);
    if (!goalChunk)  return;    // 목적지 못가는지 체크

    goalChunk->cost[goalIdx] = 0.0f;
    openList.push({ goal, 0.0f });

    std::vector<NeighborInfo> neighbors;
    while (!openList.empty())
    {
        // 가장 낮은 비용부터 뽑기
        DirectX::XMINT3 curr = openList.top().pos;
        openList.pop();

        int currIdx;
        FlowFieldChunk::ColumnData* currChunk = FindOrCreateColumn(grid, curr.x, curr.y, curr.z, currIdx, chunkcache);

        // 같은 좌표 뽑기 방지
        if (true == currChunk->visited[currIdx]) continue;
        currChunk->visited[currIdx] = true;

        float currCost = currChunk->cost[currIdx];
        // 걸어서 갈 수 있는 이웃좌표
        GetWalkableNeighbors(grid, curr, neighbors);

        for (const auto& n : neighbors)
        {
            // 해당 y에 mask없으면 continue
            if (mask.find(MakeCellKey(n.pos.x, n.pos.y, n.pos.z)) == mask.end())  continue;

            int next_idx;
            FlowFieldChunk::ColumnData* nChunk = FindOrCreateColumn(grid, n.pos.x, n.pos.y, n.pos.z, next_idx, chunkcache);
            if (!nChunk) continue;
            if (nChunk->visited[next_idx]) continue;   // 이미 확정된 노드는 더 나아질 수 없음

            float predCost = currCost + n.cost;

            if (predCost < nChunk->cost[next_idx])
            {
                nChunk->cost[next_idx] = predCost;
                openList.push({ n.pos, predCost });
            }

        }
    }

    ComputeDirections(grid);

}

void CorridorFlowField::ComputeDirections(const VoxelGrid& grid)
{
    std::vector<NeighborInfo> neighbors;

    // 다익스트라로 만들어둔 chunk(cost가 계산되어있는) 순회
    for (auto& k : m_Chunks)
    {
        auto& key = k.first;
        auto& chunkPtr = k.second;
        int cx, cy, cz;
        DecodeChunkKey(key, cx, cy, cz);

        ChunkCache chunkCache;

        // 청크 내부 로컬 좌표 순회 (8*8)
        for (int lz = 0; lz < FlowFieldChunk::SIZE; lz++)
        {
            for (int lx = 0; lx < FlowFieldChunk::SIZE; lx++)
            {
                // 월드좌표화
                int x = cx * FlowFieldChunk::SIZE + lx;
                int z = cz * FlowFieldChunk::SIZE + lz;
                // 해당 타일 SoA
                VoxelGrid::SurfaceSpan surfaces = grid.GetSurfaceYList(x, z);
                FlowFieldChunk::ColumnData& col = chunkPtr->At(lx, lz);

                for (int slot = 0; slot < surfaces.count; ++slot)
                {
                    if (false == col.visited[slot])  continue;

                    float myCost = col.cost[slot];
                    if (myCost == 0.0f)  continue;  // 목적지 자신

                    int y = surfaces.data[slot];

                    // 해당 셀의 이웃 후보 neighbors에 저장
                    GetWalkableNeighbors(grid, { x, y, z }, neighbors);

                    // 자신의 비용을 기준으로 가장 비용이 낮은 이웃 찾기
                    float bestCost = myCost;
                    DirectX::XMINT3 bestNeighbor{ x, y, z };
                    bool found = false;

                    for (const auto& n : neighbors)
                    {
                        int nIdx;
                        const FlowFieldChunk::ColumnData* nChunk = FindColumn(grid, n.pos.x, n.pos.y, n.pos.z, nIdx, chunkCache);
                        // 해당 이웃이 방문했고, 찾은것중(나포함) 더 비용이 낮고(목표에 가깝고) 
                        if (nChunk && nChunk->visited[nIdx] && nChunk->cost[nIdx] < bestCost)
                        {
                            bestCost = nChunk->cost[nIdx];
                            bestNeighbor = n.pos;
                            found = true;
                        }
                    }

                    // 더 비용이 낮은 이웃으로 향함
                    if (true == found)
                    {
                        Math::Vector3 selfPos = grid.GetWorldPos(x, y, z);
                        Math::Vector3 neighborPos = grid.GetWorldPos(bestNeighbor.x, bestNeighbor.y, bestNeighbor.z);
                        Math::Vector3 dir = Math::Normalize(neighborPos - selfPos);
                        col.direction[slot] = DirectX::XMFLOAT3(dir.GetX(), dir.GetY(), dir.GetZ());
                    }
                }

 
            }
        }
        
    }
}

bool CorridorFlowField::SampleDirection(const VoxelGrid& grid, int x, int y, int z, DirectX::XMFLOAT3& outDir) const
{
    int slot;
    ChunkCache chunkcache;
    const FlowFieldChunk::ColumnData* col = FindColumn(grid, x, y, z, slot, chunkcache);
    if (!col || slot < 0 || !col->visited[slot]) return false;

    outDir = col->direction[slot];
    return true;
}

bool CorridorFlowField::IsVisited(const VoxelGrid& grid, int x, int y, int z) const
{
    int slotIdx;
    ChunkCache chunkcache;
    const FlowFieldChunk::ColumnData* col = FindColumn(grid, x, y, z, slotIdx, chunkcache);
    if (!col || slotIdx < 0) return false;
    return col->visited[slotIdx];
}
