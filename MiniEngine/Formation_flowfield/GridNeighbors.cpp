#include "GridNeighbors.h"
#include <cmath>
#include <climits>

void GetWalkableNeighbors(const VoxelGrid& grid, const DirectX::XMINT3& curr, std::vector<DirectX::XMINT3>& outNeighbors)
{
    outNeighbors.clear();

    const int dx[] = { 1, -1, 0, 0 };
    const int dz[] = { 0, 0, 1, -1 };

    for (int d = 0; d < 4; d++)
    {
        int nx = curr.x + dx[d];
        int nz = curr.z + dz[d];

        std::vector<int> surfaces = grid.GetSurfaceYList(nx, nz);

        int bestY = -1;
        int bestDiff = INT_MAX;
        for (int sy : surfaces)
        {
            int diff = std::abs(sy - curr.y);
            if (diff <= 1 && diff < bestDiff)
            {
                bestDiff = diff;
                bestY = sy;
            }
        }

        if (bestY >= 0 && grid.IsWalkable(nx, bestY, nz))
            outNeighbors.push_back({ nx, bestY, nz });
    }
}
