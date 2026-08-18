#include "CorridorFlowField.h"
#include "GridNeighbors.h"
#include "ChunkKey.h"
#include <queue>
#include <functional>  
#include <cmath>


#include "ScopedCpuTimer.h"

//------------------------------------------
// direction 패킹용 헬퍼
//------------------------------------------
namespace
{
    // dx, dy, dz (각 -1/0/1) -> 2비트씩 패킹
    // dy는 -1 1 로 clamp 해놓음 - 벽타기시, 값이 넘치지만 화살표 시각화때문에
    uint8_t EncodeDirDelta(int dx, int dy, int dz)
    {
        const int dyStore = (dy > 1) ? 1 : ((dy < -1) ? -1 : dy);   // c++14 업글하면 clamp로 바꾸기
        return (uint8_t)((dx      + 1) & 0x3)       // 값 2비트만 원본 남겨주기
            | ((uint8_t)((dyStore + 1) & 0x3) << 2)
            | ((uint8_t)((dz      + 1) & 0x3) << 4);
    }

    void DecodeDirDelta(uint8_t packed, int& dx, int& dy, int& dz)
    {
        dx = (int)(packed & 0x3u) - 1;
        dy = (int)((packed >> 2) & 0x3u) - 1;
        dz = (int)((packed >> 4) & 0x3u) - 1;
    }
    void DecodeDirDelta(uint8_t packed, DirectX::XMINT3& d) // overloaded
    {
        d.x = (int)(packed & 0x3u) - 1;
        d.y = (int)((packed >> 2) & 0x3u) - 1;
        d.z = (int)((packed >> 4) & 0x3u) - 1;
    }
}





CorridorFlowField::FlowFieldChunk::ColumnData* CorridorFlowField::FindOrCreateColumn(
    const VoxelGrid& grid, int x, int y, int z,
    int& outSlotIdx, ChunkCache& cache) 
{
    // y값 관여 x
    int64_t key = ChunkKeyOf(x, z);

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
    // y값 관여 x
    int64_t key = ChunkKeyOf(x, z);

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

void CorridorFlowField::Build(const VoxelGrid& grid, const DirectX::XMINT3& goal,
    const std::unordered_set<int64_t>* mask,
    float maxCost,
    const std::atomic<bool>* cancelFlag)
{
    CORE_SCOPE(CorridorField_Build);
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
    uint32_t sinceCheck = 0;
    {
        CORE_SCOPE(FieldDijkstra);
        while (!openList.empty())
        {
            // 1024번 마다 캔슬인지 확인 - 추후 너무 느리면 숫자 키워라
            if (++sinceCheck >= 1024)
            {
                sinceCheck = 0;
                if (cancelFlag && cancelFlag->load(std::memory_order_relaxed))   return;
            }

            const OpenEntry entry = openList.top();
            openList.pop();

            int currIdx;
            FlowFieldChunk::ColumnData* currChunk = FindOrCreateColumn(grid, entry.pos.x, entry.pos.y, entry.pos.z, currIdx, chunkcache);
            if (!currChunk)  continue;

            // 방문 여부를 cost 비교로 확정
            if (entry.cost > currChunk->cost[currIdx])   continue;

            const float currCost = currChunk->cost[currIdx];

            GetWalkableNeighbors(grid, entry.pos, neighbors);

        for (const auto& n : neighbors)
        {
            // 비용 계산 -> 컷오프 -> 마스크 -> 청크 할당
            const float predCost = currCost + n.cost;
            
            // 컷오프 - near 필드가 마스크 없이도 유한하게 닫히는 근거
            if (predCost > maxCost)  continue;

            // 해당 y에 mask없으면 continue
            if (mask &&  mask->find(MakeCellKey(n.pos)) == mask->end() ) continue;

                int next_idx;
                FlowFieldChunk::ColumnData* nChunk = FindOrCreateColumn(grid, n.pos.x, n.pos.y, n.pos.z, next_idx, chunkcache);
                if (!nChunk) continue;

 
            if (predCost < nChunk->cost[next_idx])
            {
                nChunk->cost[next_idx] = predCost;
                openList.push({ n.pos, predCost });
            }
        }
    }

    {
        CORE_SCOPE(FieldComputeDir);
        CoreCounter::Set(CoreCount::FieldChunks, m_Chunks.size());
        CoreCounter::Set(CoreCount::FieldChunkBytes, sizeof(FlowFieldChunk));
        ComputeDirections(grid);
    }

}

void CorridorFlowField::ComputeDirections(const VoxelGrid& grid)
{
    std::vector<NeighborInfo> neighbors;
    int cnt = 0;

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
                    if (false == col.IsReached(slot))
                        continue;

                    cnt++;

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
                        if (nChunk && nChunk->cost[nIdx] < bestCost)
                        {
                            bestCost = nChunk->cost[nIdx];
                            bestNeighbor = n.pos;
                            found = true;
                        }
                    }

                    // 더 비용이 낮은 이웃으로 향함
                    if (true == found)
                    {
                        col.direction[slot] = EncodeDirDelta(bestNeighbor.x - x,
                                                            bestNeighbor.y - y,
                                                            bestNeighbor.z - z);
                    }
                }

 
            }
        }
        
    }
    CoreCounter::Set(CoreCount::FieldReached, cnt);
}

bool CorridorFlowField::SampleDirection(const VoxelGrid& grid, int x, int y, int z, DirectX::XMINT3& outDir) const
{
    int slot;
    ChunkCache chunkcache;
    const FlowFieldChunk::ColumnData* col = FindColumn(grid, x, y, z, slot, chunkcache);
    if (!col || slot < 0 || !col->IsReached(slot)) return false;

    const uint8_t packed = col->direction[slot];
    if (FlowFieldChunk::kNoDir == packed)    return false;  // 방향없음

    DecodeDirDelta(packed, outDir);
    return true;
}

bool CorridorFlowField::SampleCost(const VoxelGrid& grid, int x, int y, int z, float& outCost) const
{
    ChunkCache cache;
    int slot;
    const FlowFieldChunk::ColumnData* col = FindColumn(grid, x, y, z, slot, cache);
    if (!col || slot < 0 || !col->IsReached(slot)) return false;

    outCost = col->cost[slot];
    return true;
}

bool CorridorFlowField::IsVisited(const VoxelGrid& grid, int x, int y, int z) const
{
    int slotIdx;
    ChunkCache chunkcache;
    const FlowFieldChunk::ColumnData* col = FindColumn(grid, x, y, z, slotIdx, chunkcache);
    if (!col || slotIdx < 0) return false;
    return col->IsReached(slotIdx);
}


void CorridorFlowField::ReportMemory(const char* filePath) const
{
    size_t chunks = m_Chunks.bucket_count() * 16 + m_Chunks.size() * (4352 + 16 + 8);
    char buf[256];
    sprintf_s(buf, "[CorridorFlowField] chunks=%zu  --- %8.1f KB\n", m_Chunks.size(), chunks / 1024.0);
    Utility::Print(buf);
    FILE* fp = nullptr; fopen_s(&fp, filePath, "a");
    if (fp) { fputs(buf, fp); fclose(fp); }
}