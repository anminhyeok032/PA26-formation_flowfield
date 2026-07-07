#include "AStarPathfinder.h"
#include "GridNeighbors.h"
#include <queue>
#include <algorithm>
#include <cmath>

AStarPathfinder::PathNodeChunk* AStarPathfinder::FindOrCreateChunk(int x, int y, int z, int& outLocalIdx)
{
    int cx = x / PathNodeChunk::SIZE;
    int cy = y / PathNodeChunk::SIZE;
    int cz = z / PathNodeChunk::SIZE;
    int64_t key = MakeChunkKey(cx, cy, cz);

    auto it = m_Chunks.find(key);
    if (it == m_Chunks.end())
        it = m_Chunks.emplace(key, std::make_unique<PathNodeChunk>()).first;

    int lx = x % PathNodeChunk::SIZE;
    int ly = y % PathNodeChunk::SIZE;
    int lz = z % PathNodeChunk::SIZE;
    outLocalIdx = lx + PathNodeChunk::SIZE * (ly + PathNodeChunk::SIZE * lz);
    return it->second.get();
}

const AStarPathfinder::PathNodeChunk* AStarPathfinder::FindChunk(int x, int y, int z, int& outLocalIdx) const
{
    int cx = x / PathNodeChunk::SIZE;
    int cy = y / PathNodeChunk::SIZE;
    int cz = z / PathNodeChunk::SIZE;
    int64_t key = MakeChunkKey(cx, cy, cz);

    auto it = m_Chunks.find(key);
    if (it == m_Chunks.end()) return nullptr;

    int lx = x % PathNodeChunk::SIZE;
    int ly = y % PathNodeChunk::SIZE;
    int lz = z % PathNodeChunk::SIZE;
    outLocalIdx = lx + PathNodeChunk::SIZE * (ly + PathNodeChunk::SIZE * lz);
    return it->second.get();
}

bool AStarPathfinder::FindPath(const VoxelGrid& grid, const XMINT3& start, const XMINT3& goal,
    std::vector<XMINT3>& outPath, int maxIterations)
{
    m_Chunks.clear(); // 이전 탐색 기록 초기화 (탐색마다 새로 시작)
    outPath.clear();

    struct OpenEntry
    {
        XMINT3 pos;
        float   fScore;
        bool operator>(const OpenEntry& o) const { return fScore > o.fScore; }
    };

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> openList;

    int startIdx;
    PathNodeChunk* startChunk = FindOrCreateChunk(start.x, start.y, start.z, startIdx);
    startChunk->gScore[startIdx] = 0.0f;
    openList.push({ start, Heuristic(start, goal) });

    std::vector<DirectX::XMINT3> neighbors;
    int iterations = 0;

    while (!openList.empty())
    {
        if (++iterations > maxIterations)
        {
            return false; // 비정상적으로 오래 걸리면 실패 처리 (도달 불가 등)
        }

        XMINT3 current = openList.top().pos;
        openList.pop();

        // 이미 더 나은 경로로 확정된 노드라면 스킵
        int curIdx;
        PathNodeChunk* curChunk = FindOrCreateChunk(current.x, current.y, current.z, curIdx);
        if (true == curChunk->closed[curIdx]) continue;
        curChunk->closed[curIdx] = true;

        if (current == goal)
        {
            return ReconstructPath(grid, start, goal, outPath); // gScore만으로 경로 복원
        }

        float currentG = curChunk->gScore[curIdx];

        GetWalkableNeighbors(grid, current, neighbors);

        for (const auto& n : neighbors)
        {
            int idx;
            PathNodeChunk* nChunk = FindOrCreateChunk(n.x, n.y, n.z, idx);
            if (nChunk->closed[idx]) continue;

            float predG = currentG + 1.0f;

            if (predG < nChunk->gScore[idx])
            {
                nChunk->gScore[idx] = predG;
                openList.push({ n, predG + Heuristic(n, goal) });
            }
        }
    }

    return false; // openList가 비었는데 목적지 도달 못함 -> 경로 없음
}


bool AStarPathfinder::ReconstructPath(const VoxelGrid& grid, const DirectX::XMINT3& start,
    const DirectX::XMINT3& goal, std::vector<DirectX::XMINT3>& outPath) const
{
    outPath.clear();
    outPath.push_back(goal);

    DirectX::XMINT3 trace = goal;
    std::vector<DirectX::XMINT3> neighbors;

    while (!(trace == start))
    {
        int traceIdx;
        const PathNodeChunk* traceChunk = FindChunk(trace.x, trace.y, trace.z, traceIdx);
        float traceG = traceChunk->gScore[traceIdx];

        GetWalkableNeighbors(grid, trace, neighbors);

        DirectX::XMINT3 prev{ -1,-1,-1 };
        bool found = false;

        for (const auto& n : neighbors)
        {
            int idx;
            const PathNodeChunk* nChunk = FindChunk(n.x, n.y, n.z, idx);
            // 나보다 gScore가 1 작은 이웃 == 내가 여기 온 경로의 부모
            if (nChunk && 
                true == nChunk->closed[idx] &&
                std::abs(nChunk->gScore[idx] - (traceG - 1.0f)) < 0.001f)   // float 비교 epsilon
            {
                prev = n;
                found = true;
                break;
            }
        }

        if (!found) return false; // 이론상 발생하면 안 되지만, 안전장치로 실패 처리

        outPath.push_back(prev);
        trace = prev;
    }

    std::reverse(outPath.begin(), outPath.end());
    return true;
}

