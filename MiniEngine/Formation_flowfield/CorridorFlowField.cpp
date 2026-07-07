#include "CorridorFlowField.h"
#include "GridNeighbors.h"
#include "ChunkKey.h"
#include <queue>
#include <cmath>

CorridorFlowField::FlowFieldChunk* CorridorFlowField::FindOrCreateChunk(int x, int y, int z, int& outLocalIdx)
{
    int cx = x / FlowFieldChunk::SIZE;
    int cy = y / FlowFieldChunk::SIZE;
    int cz = z / FlowFieldChunk::SIZE;
    int64_t key = MakeChunkKey(cx, cy, cz);

    auto it = m_Chunks.find(key);
    if (it == m_Chunks.end())
        it = m_Chunks.emplace(key, std::make_unique<FlowFieldChunk>()).first;

    int lx = x % FlowFieldChunk::SIZE;
    int ly = y % FlowFieldChunk::SIZE;
    int lz = z % FlowFieldChunk::SIZE;
    outLocalIdx = lx + FlowFieldChunk::SIZE * (ly + FlowFieldChunk::SIZE * lz);
    return it->second.get();
}

const CorridorFlowField::FlowFieldChunk* CorridorFlowField::FindChunk(int x, int y, int z, int& outLocalIdx) const
{
    int cx = x / FlowFieldChunk::SIZE;
    int cy = y / FlowFieldChunk::SIZE;
    int cz = z / FlowFieldChunk::SIZE;
    int64_t key = MakeChunkKey(cx, cy, cz);

    auto it = m_Chunks.find(key);
    if (it == m_Chunks.end()) return nullptr;

    int lx = x % FlowFieldChunk::SIZE;
    int ly = y % FlowFieldChunk::SIZE;
    int lz = z % FlowFieldChunk::SIZE;
    outLocalIdx = lx + FlowFieldChunk::SIZE * (ly + FlowFieldChunk::SIZE * lz);
    return it->second.get();
}

void CorridorFlowField::Build(const VoxelGrid& grid, const DirectX::XMINT3& goal, const std::unordered_set<int64_t>& mask)
{
    m_Chunks.clear();

    std::queue<DirectX::XMINT3> frontier;

    int goalIdx;
    FlowFieldChunk* goalChunk = FindOrCreateChunk(goal.x, goal.y, goal.z, goalIdx);
    goalChunk->cost[goalIdx] = 0.0f;
    goalChunk->visited[goalIdx] = true;
    frontier.push(goal);

    std::vector<DirectX::XMINT3> neighbors;

    while (!frontier.empty())
    {
        DirectX::XMINT3 current = frontier.front();
        frontier.pop();

        int curIdx;
        FlowFieldChunk* curChunk = FindOrCreateChunk(current.x, current.y, current.z, curIdx);
        float currentCost = curChunk->cost[curIdx];

        GetWalkableNeighbors(grid, current, neighbors);

        for (const auto& n : neighbors)
        {
            int ncx = n.x / VoxelChunk::CHUNK_SIZE;
            int ncy = n.y / VoxelChunk::CHUNK_SIZE;
            int ncz = n.z / VoxelChunk::CHUNK_SIZE;
            if (mask.find(MakeChunkKey(ncx, ncy, ncz)) == mask.end())
                continue;

            int nIdx;
            FlowFieldChunk* nChunk = FindOrCreateChunk(n.x, n.y, n.z, nIdx);
            if (nChunk->visited[nIdx]) continue;

            nChunk->visited[nIdx] = true;
            nChunk->cost[nIdx] = currentCost + 1.0f;
            frontier.push(n);
        }
    }

    ComputeDirections(grid);

}

void CorridorFlowField::ComputeDirections(const VoxelGrid& grid)
{
    std::vector<DirectX::XMINT3> neighbors;

    // 만들어진 청크 기준으로 탐색
    for (auto& [key, chunkPtr] : m_Chunks)
    {
        int cx, cy, cz;
        DecodeChunkKey(key, cx, cy, cz);

        for (int ly = 0; ly < FlowFieldChunk::SIZE; ly++)
        {
            for (int lz = 0; lz < FlowFieldChunk::SIZE; lz++)
            {
                for (int lx = 0; lx < FlowFieldChunk::SIZE; lx++)
                {
                    int localIdx = lx + FlowFieldChunk::SIZE * (ly + FlowFieldChunk::SIZE * lz);
                    if (!chunkPtr->visited[localIdx]) continue;

                    float myCost = chunkPtr->cost[localIdx];
                    if (myCost == 0.0f) continue; // 목적지 자신은 방향 없음

                    int x = cx * FlowFieldChunk::SIZE + lx;
                    int y = cy * FlowFieldChunk::SIZE + ly;
                    int z = cz * FlowFieldChunk::SIZE + lz;

                    // 해당 셀의 이웃 후보 neighbors에 저장
                    GetWalkableNeighbors(grid, { x, y, z }, neighbors);

                    float bestCost = myCost;
                    DirectX::XMINT3 bestNeighbor{ x, y, z };
                    bool found = false;

                    for (const auto& n : neighbors)
                    {
                        int nIdx;
                        const FlowFieldChunk* nChunk = FindChunk(n.x, n.y, n.z, nIdx);
                        if (nChunk && nChunk->visited[nIdx] && nChunk->cost[nIdx] < bestCost)
                        {
                            bestCost = nChunk->cost[nIdx];
                            bestNeighbor = n;
                            found = true;
                        }
                    }

                    if (found)
                    {
                        Math::Vector3 selfPos = grid.GetWorldPos(x, y, z);
                        Math::Vector3 neighborPos = grid.GetWorldPos(bestNeighbor.x, bestNeighbor.y, bestNeighbor.z);
                        Math::Vector3 dir = Math::Normalize(neighborPos - selfPos);
                        chunkPtr->direction[localIdx] = DirectX::XMFLOAT3(dir.GetX(), dir.GetY(), dir.GetZ());
                    }
                }
            }
        }
    }
}

bool CorridorFlowField::SampleDirection(int x, int y, int z, DirectX::XMFLOAT3& outDir) const
{
    int idx;
    const FlowFieldChunk* chunk = FindChunk(x, y, z, idx);
    if (!chunk || !chunk->visited[idx]) return false;

    outDir = chunk->direction[idx];
    return true;
}
