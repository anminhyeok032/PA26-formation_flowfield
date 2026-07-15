#include "PathCorridor.h"
#include "VoxelGrid.h"
#include "GridNeighbors.h"
#include "ChunkKey.h"
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <cmath>

int ComputeMarginCells(int memberCount, int bufferCells)
{
    const int MIN_MARGIN = 12;

    if (memberCount <= 1)   return MIN_MARGIN;

    float side = std::sqrt((float)memberCount);

    // 셀 단위 반지름 청크 단위로 올림
    int margin = (int)std::ceil(side * 0.5f) + bufferCells;
    return std::max(margin, MIN_MARGIN);
}

std::unordered_set<int64_t> BuildLayerMask(const VoxelGrid& grid,
    const std::vector<DirectX::XMINT3>& path, 
    const std::vector<DirectX::XMINT3>& extraSeeds, 
    int marginCells)
{
    // 청크 key 집합
    std::unordered_set<int64_t> mask;
    std::queue<DirectX::XMINT3> q;
    std::unordered_map<int64_t, int> dist;

    auto seed = [&](const DirectX::XMINT3& node) {
        int64_t key = MakeCellKey(node.x, node.y, node.z);
        if (dist.emplace(key, 0).second) { mask.insert(key); q.push(node); }
    };

    for (const auto& node : path)       seed(node);
    for (const auto& node : extraSeeds) seed(node);

    //for (const auto& node : path)
    //{
    //    int64_t key = MakeCellKey(node.x, node.y, node.z);
    //    if (dist.emplace(key, 0).second)
    //    {
    //        mask.insert(key);
    //        q.push(node);
    //    }
    //}

    std::vector<NeighborInfo> neighbors;
    while (!q.empty())
    {
        DirectX::XMINT3 curr = q.front();
        q.pop();

        int currDist = dist[MakeCellKey(curr.x, curr.y, curr.z)];
        if (currDist >= marginCells) continue;

        GetWalkableNeighbors(grid, curr, neighbors);

        for (const auto& n : neighbors)
        {
            int64_t nKey = MakeCellKey(n.pos.x, n.pos.y, n.pos.z);
            if (dist.emplace(nKey, currDist + 1).second)
            {
                mask.insert(nKey);
                q.push(n.pos);
            }
        }
    }


    return mask;
}
