#include "AStarPathfinder.h"
#include "GridNeighbors.h"
#include <queue>
#include <algorithm>
#include <cmath>

AStarPathfinder::PathNodeRecord& AStarPathfinder::GetRecord(int x, int y, int z)
{
    // 좌표를 청크 좌표로
    int cx = x / PathNodeChunk::SIZE;
    int cy = y / PathNodeChunk::SIZE;
    int cz = z / PathNodeChunk::SIZE;
    // 청크 좌표를 하나로 합침(key로 쓸려고)
    int64_t key = MakeChunkKey(cx, cy, cz);

    auto it = m_Chunks.find(key);
    if (it == m_Chunks.end())   it = m_Chunks.emplace(key, std::make_unique<PathNodeChunk>()).first;

    int lx = x % PathNodeChunk::SIZE;
    int ly = y % PathNodeChunk::SIZE;
    int lz = z % PathNodeChunk::SIZE;
    return it->second->records[lx + PathNodeChunk::SIZE * (ly + PathNodeChunk::SIZE * lz)];
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

    GetRecord(start.x, start.y, start.z).gScore = 0.0f;
    openList.push({ start, Heuristic(start, goal) });

    std::vector<XMINT3> neighbors;
    int iterations = 0;

    while (!openList.empty())
    {
        if (++iterations > maxIterations)
        {
            return false; // 비정상적으로 오래 걸리면 실패 처리 (도달 불가 등)
        }

        XMINT3 current = openList.top().pos;
        openList.pop();

        PathNodeRecord& curRec = GetRecord(current.x, current.y, current.z);

        // 이미 더 나은 경로로 확정된 노드라면 스킵
        // (priority_queue는 삭제가 없으므로, 중복 push된 낡은 항목을 여기서 걸러냄)
        if (curRec.closed) continue;
        curRec.closed = true;

        if (current == goal)
        {
            // 목적지 도달 -> cameFrom을 거슬러 올라가며 경로 재구성
            XMINT3 trace = goal;
            while (!(trace == start))
            {
                outPath.push_back(trace);
                trace = GetRecord(trace.x, trace.y, trace.z).cameFrom;
            }
            outPath.push_back(start);
            std::reverse(outPath.begin(), outPath.end());
            return true;
        }

        GetWalkableNeighbors(grid, current, neighbors);

        for (const auto& n : neighbors)
        {
            PathNodeRecord& nRec = GetRecord(n.x, n.y, n.z);
            if (true == nRec.closed) continue;

            float tentativeG = curRec.gScore + 1.0f; // 이동 비용은 항상 셀 1칸

            if (tentativeG < nRec.gScore)
            {
                nRec.gScore = tentativeG;
                nRec.cameFrom = current;
                openList.push({ n, tentativeG + Heuristic(n, goal) });
            }
        }
    }

    return false; // openList가 비었는데 목적지 도달 못함 -> 경로 없음
}
