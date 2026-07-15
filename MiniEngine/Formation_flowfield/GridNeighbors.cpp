#include "GridNeighbors.h"
#include <cmath>
#include <climits>

// 해당 cpp에서만 사용할 helper
namespace
{
    // 표면 y 탐색 공통 로직
    // 현재 y와 높이차 1 이내 표면 중 가까운거 찾음. 없으면 -1
    int FindConnectableSurfaceY(const VoxelGrid& grid, int nx, int nz, int currY)
    {
        int bestY = -1;
        int bestDiff = INT_MAX;

        VoxelGrid::SurfaceSpan surfaces = grid.GetSurfaceYList(nx, nz);
        for (auto y : surfaces)
        {
            int diff = std::abs(y - currY);
            if (diff <= 1 && diff < bestDiff)
            {
                bestDiff = diff;
                bestY = y;
            }
        }
        return bestY;
    }

    // 실제 이동량(dx, dyActual, dz)으로 유클리드 거리를 정직하게 계산
    float ComputeMoveCost(int dx, int dyActual, int dz)
    {
        return std::sqrt((float)(dx * dx + dyActual * dyActual + dz * dz));
    }
}

void GetWalkableNeighbors(const VoxelGrid& grid, const DirectX::XMINT3& curr, std::vector<NeighborInfo>& outNeighbors)
{
    outNeighbors.clear();

    // 0~3: 카디널(직교) 4방향, 4~7: 대각선 4방향
    static const int dx[8] = { 1, -1, 0, 0,  1, 1, -1, -1};
    static const int dz[8] = { 0, 0, 1, -1,  1, -1, 1, -1};

    bool cardiWalkable[4] = { false, false, false, false };

    for (int d = 0; d < 4; d++)
    {
        int nx = curr.x + dx[d];
        int nz = curr.z + dz[d];

        int bestY = FindConnectableSurfaceY(grid, nx, nz, curr.y);
     

        if (bestY >= 0 && grid.IsWalkable(nx, bestY, nz))
        {
            cardiWalkable[d] = true;
            int dyActual = bestY - curr.y;   // 실제 높이차 (-1, 0, +1)
            float cost = ComputeMoveCost(dx[d], dyActual, dz[d]);
            outNeighbors.push_back({ DirectX::XMINT3{ nx, bestY, nz }, cost });
        }
    }

    // 대각선 4방향 - 코너 파고드는거 방지 후 계산
    static const int cardA[4] = { 0, 0, 1, 1 };
    static const int cardB[4] = { 2, 3, 2, 3 };

    for (int d = 4; d < 8; d++)
    {
        int pairidx = d - 4;
        if (false == cardiWalkable[cardA[pairidx]] ||
            false == cardiWalkable[cardB[pairidx]])
        {
            // 갈려는곳 양옆 막혀있으면 못가는곳 처리
            continue;
        }

        int nx = curr.x + dx[d];
        int nz = curr.z + dz[d];

        int bestY = FindConnectableSurfaceY(grid, nx, nz, curr.y);

        if (bestY >= 0 && grid.IsWalkable(nx, bestY, nz))
        {
            int dyActual = bestY - curr.y;   // 실제 높이차 (-1, 0, +1)
            float cost = ComputeMoveCost(dx[d], dyActual, dz[d]);
            outNeighbors.push_back({ DirectX::XMINT3{ nx, bestY, nz }, cost });
        }
    }
}

int FindSurfaceSlot(const VoxelGrid& grid, int x, int y, int z)
{
    VoxelGrid::SurfaceSpan surfaces = grid.GetSurfaceYList(x, z);
    for (int i = 0; i < surfaces.count; ++i)
    {
        if (surfaces.data[i] == y)   return i;
    }
    return -1;
}
