#include "GridNeighbors.h"
#include <cmath>
#include <climits>

// 해당 cpp에서만 사용할 helper
namespace
{
    /// 짚고 오를 벽면이 있는지
    // 첫 empty에서 실패시, 그위는 볼필요x
    bool IsWallFaceSolid(const VoxelGrid& grid, int x, int z,int loY, int hiY)
    {
        for (int y = loY; y <= hiY; ++y)
        {
            if (grid.GetCell(x, y, z) == VoxelGrid::CellType::Empty)    return false;
        }
        return true;
    }

    // 실제 이동량(dx, dyActual, dz)으로 유클리드 거리를 정직하게 계산
    //  Climb 구간은 패널티로 환산 - 다익스트라 대칭 위해
    float ComputeMoveCost(int dx, int dyActual, int dz)
    {
        const int ady = std::abs(dyActual);
        if (ady <= 1)
        {
            return std::sqrt((float)(dx * dx + dyActual * dyActual + dz * dz));
        }
        
        const float base = std::sqrt((float)(dx * dx + 1 + dz * dz));
        return base + CLIMB_COST_PER_CELL * (float)(ady - 1);
    }
}

// 표면 y 탐색 공통 로직
// 현재 y와 높이차 1 이내 표면 중 가까운거 찾음. 없으면 -1
int FindReachableSurfaceY(const VoxelGrid& grid, DirectX::XMINT3 curr,
    int nx, int nz, int maxClimbCells)
{
    int walkY = -1;
    int walkDiff = INT_MAX;
    int climbY = -1;
    int climbDiff = INT_MAX;

    VoxelGrid::SurfaceSpan surfaces = grid.GetSurfaceYList(nx, nz);
    for (auto y : surfaces)
    {
        const int diff = std::abs((int)y - curr.y);
        if (diff > maxClimbCells)           continue;   // 높이 차 제한
        if (!grid.IsWalkable(nx, y, nz))    continue;   // 올라갈 셀이 walkable인지
        // 높이차 1인거 기록
        if (diff <= 1)
        {
            if (diff < walkDiff)
            {
                walkDiff = diff;
                walkY = y;
            }
        }
        if (diff >= climbDiff)   continue;

        // 벽면은 항상 두 컬럼 중 높은 쪽에 있다
        // 중간에 다른 표면이 끼면 그 위가 empty라 자동으로 걸러짐
        const bool up   = ((int)y > curr.y);   // 위
        const int faceX = up ? nx : curr.x;
        const int faceZ = up ? nz : curr.z;
        const int loY   = up ? curr.y : (int)y;
        const int hiY   = up ? (int)y : curr.y;

        if (IsWallFaceSolid(grid, faceX, faceZ, loY, hiY))
        {
            climbDiff = diff;
            climbY = y;
        }

    }
    return (walkY >= 0) ? walkY : climbY;   // 걸을 수 있으면 걸음 우선
}

int FindReachableSurfaceY(const VoxelGrid& grid, int x, int y, int z, int nx, int nz, int maxClimbCells)
{
    DirectX::XMINT3 d{ x, y, z };
    return FindReachableSurfaceY(grid, d, nx, nz, maxClimbCells);
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

        int bestY = FindReachableSurfaceY(grid, curr, nx, nz);
        if (bestY < 0)   continue;

        const int dyActual = bestY - curr.y;

        // 대각 게이트는 걸어서만 가능
        cardiWalkable[d] = (std::abs(dyActual) <= 1);

        outNeighbors.push_back({ DirectX::XMINT3{nx, bestY, nz}, 
                                ComputeMoveCost(dx[d], dyActual, dz[d]) });

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


        // 대각선은 climb 금지 - maxclimbcell = 1로 고정
        int bestY = FindReachableSurfaceY(grid, curr, nx, nz, 1);

        if (bestY < 0) continue;

        const int dyActual = bestY - curr.y;
        outNeighbors.push_back({ DirectX::XMINT3{ nx, bestY, nz },
                                 ComputeMoveCost(dx[d], dyActual, dz[d]) });
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
