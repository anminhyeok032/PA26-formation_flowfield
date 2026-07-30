#include "AStarPathfinder.h"
#include "GridNeighbors.h"
#include <queue>
#include <algorithm>
#include <cmath>

// PathNodeChunk 내에 parentDir을 위한 헬퍼
namespace
{
    constexpr uint8_t kNoParent = 0xFFu;

    // dx,dy,dz(각 -1/0/1) -> 2비트씩 packing (0/1/2로 offset해서 저장)
    uint8_t EncodeParentDelta(int dx, int dy, int dz)
    {
        uint8_t ex = (uint8_t)(dx + 1);
        uint8_t ey = (uint8_t)(dy + 1);
        uint8_t ez = (uint8_t)(dz + 1);
        return ex | (ey << 2) | (ez << 4);
    }

    void DecodeParentDelta(uint8_t packed, int& dx, int& dy, int& dz)
    {
        dx = (int)(packed & 0x3u) - 1;
        dy = (int)((packed >> 2) & 0x3u) - 1;
        dz = (int)((packed >> 4) & 0x3u) - 1;
    }
}

AStarPathfinder::PathNodeChunk* AStarPathfinder::FindOrCreateChunk(int x, int y, int z, int& outLocalIdx, ChunkCache& cache)
{
    int cx = x / PathNodeChunk::SIZE;
    int cy = y / PathNodeChunk::SIZE;
    int cz = z / PathNodeChunk::SIZE;
    int64_t key = MakeChunkKey(cx, cy, cz);

    if (key != cache.key)
    {
        auto it = m_Chunks.find(key);
        if (it == m_Chunks.end())
            it = m_Chunks.emplace(key, std::make_unique<PathNodeChunk>()).first;

        cache.key = key;
        cache.chunk = it->second.get();
    }

    int lx = x % PathNodeChunk::SIZE;
    int ly = y % PathNodeChunk::SIZE;
    int lz = z % PathNodeChunk::SIZE;
    outLocalIdx = lx + PathNodeChunk::SIZE * (ly + PathNodeChunk::SIZE * lz);
    return cache.chunk;
}

const AStarPathfinder::PathNodeChunk* AStarPathfinder::FindChunk(int x, int y, int z, int& outLocalIdx, ChunkCache& cache) const
{
    int cx = x / PathNodeChunk::SIZE;
    int cy = y / PathNodeChunk::SIZE;
    int cz = z / PathNodeChunk::SIZE;
    int64_t key = MakeChunkKey(cx, cy, cz);

    if (key != cache.key)
    {
        auto it = m_Chunks.find(key);
        if (it == m_Chunks.end())
        {
            cache.key = -1;   // 실패 시 캐시를 무효화
            return nullptr;
        }
        cache.key = key;
        cache.chunk = const_cast<PathNodeChunk*>(it->second.get());
    }

    int lx = x % PathNodeChunk::SIZE;
    int ly = y % PathNodeChunk::SIZE;
    int lz = z % PathNodeChunk::SIZE;
    outLocalIdx = lx + PathNodeChunk::SIZE * (ly + PathNodeChunk::SIZE * lz);
    return cache.chunk;
}

// 제외 집합 없는 기본 호출 — 정적 빈 집합 재사용(매 호출 할당 방지)
bool AStarPathfinder::FindPath(const VoxelGrid& grid, const XMINT3& start, const XMINT3& goal,
    std::vector<XMINT3>& outPath, int maxIterations)
{
    static const std::unordered_set<int64_t> s_Empty;
    return FindPath(grid, start, goal, outPath, s_Empty, maxIterations);
}

bool AStarPathfinder::FindPath(const VoxelGrid& grid, const DirectX::XMINT3& start, const DirectX::XMINT3& goal, 
    std::vector<DirectX::XMINT3>& outPath, const std::unordered_set<int64_t>& excluded, int maxIterations)
{
    // 시작이나 목표 막혀있으면 탐색 불가
    if (!excluded.empty())
    {
        if (excluded.count(MakeCellKey(start)) > 0)  return false;
        if (excluded.count(MakeCellKey(goal)) > 0)  return false;
    }

    // 각 인스턴스마다 따로 가지고 있음
    m_Chunks.clear(); // 이전 탐색 기록 초기화 (탐색마다 새로 시작)
    outPath.clear();

    struct OpenEntry
    {
        XMINT3 pos;
        float   fScore;
        bool operator>(const OpenEntry& o) const { return fScore > o.fScore; }
    };

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> openList;

    ChunkCache chunkCache;

    int startIdx;
    PathNodeChunk* startChunk = FindOrCreateChunk(start.x, start.y, start.z, startIdx, chunkCache);
    startChunk->gScore[startIdx] = 0.0f;
    startChunk->parentDir[startIdx] = kNoParent;
    openList.push({ start, Heuristic(start, goal) });

    std::vector<NeighborInfo> neighbors;
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
        PathNodeChunk* currChunk = FindOrCreateChunk(current.x, current.y, current.z, curIdx, chunkCache);
        if (true == currChunk->closed[curIdx]) continue;
        currChunk->closed[curIdx] = true;

        if (current == goal)
        {
            return ReconstructPath(start, goal, outPath); // gScore만으로 경로 복원
        }

        float currentG = currChunk->gScore[curIdx];

        GetWalkableNeighbors(grid, current, neighbors);

        for (const auto& n : neighbors)
        {
            int idx;
            PathNodeChunk* nChunk = FindOrCreateChunk(n.pos.x, n.pos.y, n.pos.z, idx, chunkCache);
            if (nChunk->closed[idx]) continue;
            // 제외 셀은 통행 불가로 간주 (지형은 그대로 두고 탐색에서만 배제)
            if (!excluded.empty() && excluded.count(MakeCellKey(n.pos)) > 0) continue;

            float predG = currentG + n.cost;

            if (predG < nChunk->gScore[idx])
            {
                nChunk->gScore[idx] = predG;

                // n의 부모는 current이므로, n -> current로 가는 델타(= current - n)를 저장.
                // 나중에 재구성 시 n + delta = current(부모)로 즉시 복원 가능.
                int dx = current.x - n.pos.x;
                int dy = current.y - n.pos.y;
                int dz = current.z - n.pos.z;

                nChunk->parentDir[idx] = EncodeParentDelta(dx, dy, dz);


                openList.push({ n.pos, predG + Heuristic(n.pos, goal) });
            }
        }
    }

    return false; // openList가 비었는데 목적지 도달 못함 -> 경로 없음
}


bool AStarPathfinder::ReconstructPath(const DirectX::XMINT3& start,
    const DirectX::XMINT3& goal, std::vector<DirectX::XMINT3>& outPath) const
{
    outPath.clear();
    outPath.push_back(goal);

    DirectX::XMINT3 trace = goal;
    ChunkCache cachechunk;

    while (!(trace == start))
    {
        int idx;
        const PathNodeChunk* traceChunk = FindChunk(trace.x, trace.y, trace.z, idx, cachechunk);

        if (!traceChunk || traceChunk->parentDir[idx] == kNoParent)
            return false;   // 이론상 발생하면 안 되지만, 안전장치로 실패 처리

        int dx, dy, dz;
        DecodeParentDelta(traceChunk->parentDir[idx], dx, dy, dz);

        DirectX::XMINT3 prev{ trace.x + dx, trace.y + dy, trace.z + dz };
        outPath.emplace_back(prev);
        trace = prev;
    }

    std::reverse(outPath.begin(), outPath.end());
    return true;
}

