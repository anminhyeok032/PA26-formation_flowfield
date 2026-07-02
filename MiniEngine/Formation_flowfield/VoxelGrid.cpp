#include "VoxelGrid.h"
#include "HeightMap.h"
#include <algorithm>
#include <cmath>

void VoxelGrid::Initialize(int sizeX, int sizeY, int sizeZ, float cellSize)
{
    m_SizeX = sizeX;
    m_SizeY = sizeY;
    m_SizeZ = sizeZ;
    m_CellSize = cellSize;
    m_Grid.assign(sizeX * sizeY * sizeZ, CellType::Walkable);
    m_Cells.clear();
}

void VoxelGrid::Shutdown()
{
    m_Cells.clear();
    m_Grid.clear();
}

void VoxelGrid::BuildFromHeightMap(const HeightMap& hm)
{
    BuildCells(hm);
    ValidateWalkable();
}

void VoxelGrid::BuildCells(const HeightMap& hm)
{
    m_Cells.clear();

    m_SizeX = hm.GetWidth();
    m_SizeZ = hm.GetDepth();
    m_CellSize = hm.GetCellSize();
    m_SizeY = (int)(hm.GetMaxHeight() / m_CellSize) + 1;

    // 3D 그리드 전체 Empty로 초기화
    m_Grid.assign(m_SizeX * m_SizeY * m_SizeZ, CellType::Empty);

    // 1단계: 높이맵 읽어서 3D 그리드에 채워진 공간 마킹
    //        m_Grid만 채움 (m_Cells는 아직 안 넣음)
    for (int z = 0; z < m_SizeZ; z++)
    {
        for (int x = 0; x < m_SizeX; x++)
        {
            float worldH = hm.GetHeight(x, z);
            int   surfY = (int)(worldH / m_CellSize);
            surfY = std::max(0, std::min(surfY, m_SizeY - 1));

            // y=0 ~ surfY 공간을 Blocked로 마킹 (패스 2에서 Walkable로 바뀔 수 있음)
            for (int y = 0; y <= surfY; y++)
            {
                m_Grid[Index(x, y, z)] = CellType::Blocked;
            }
        }
    }

    // 2단계: 렌더링할 복셀만 m_Cells에 추가
    //        조건: 표면 OR 바닥(y=0) OR 4면 벽(맵 가장자리)
    for (int z = 0; z < m_SizeZ; z++)
    {
        for (int x = 0; x < m_SizeX; x++)
        {
            int surfY = GetSurfaceY(x, z);

            for (int y = 0; y <= surfY; y++)
            {
                // 이 복셀을 렌더링해야 하는지 판단
                bool isTopSurface = (y == surfY);
                bool isBottom = (y == 0);
                bool isWallX = (x == 0 || x == m_SizeX - 1);
                bool isWallZ = (z == 0 || z == m_SizeZ - 1);

                // 4면 벽: 가장자리 x,z 열의 모든 y
                // 바닥: 모든 x,z에서 y=0
                // 표면: 각 x,z의 최상단 y
                // 벽면: 이웃 셀보다 높은 y는 옆면이 공기에 노출됨
                bool isExposedSide = false;
                if (!isTopSurface && !isBottom && !isWallX && !isWallZ)
                {
                    const int dx[] = { 1, -1,  0, 0 };
                    const int dz[] = { 0,  0,  1,-1 };

                    for (int d = 0; d < 4; d++)
                    {
                        int nx = x + dx[d];
                        int nz = z + dz[d];

                        // 맵 밖이면 공기에 노출된 것
                        if (nx < 0 || nx >= m_SizeX ||
                            nz < 0 || nz >= m_SizeZ)
                        {
                            isExposedSide = true;
                            break;
                        }

                        // 이웃의 surfY보다 현재 y가 높으면
                        // 이 y는 이웃 방향에서 보일 수 있는 옆면
                        if (y > GetSurfaceY(nx, nz))
                        {
                            isExposedSide = true;
                            break;
                        }
                    }
                }

                // 내부면 렌더 스킵
                if (!isTopSurface && !isBottom && !isWallX && !isWallZ && !isExposedSide)   continue;

                VoxelCell cell;
                cell.x = x;
                cell.y = y;
                cell.z = z;
                cell.type = CellType::Blocked; // 패스 2에서 덮어씀
                m_Cells.push_back(cell);
            }
        }
    }
}

void VoxelGrid::ValidateWalkable()
{
    for (auto& cell : m_Cells)
    {
        // 표면 복셀만 판정 (위쪽 y+1이 비어있어야 표면)
        if (false == IsSurface(cell.x, cell.y, cell.z))
        {
            cell.type = CellType::Blocked; // 내부 복셀은 항상 Blocked
            continue;
        }

        // 표면 복셀 walkable 판정 조건들
        bool walkable = true;

        // 조건 1 — 이웃 4방향 표면 복셀과 높이차 검사
        // 패스 1이 끝난 뒤라 GetSurfaceY로 정확한 이웃 높이를 알 수 있음
        const int dx[] = { 1, -1, 0,  0 };
        const int dz[] = { 0,  0, 1, -1 };

        for (int d = 0; d < 4; d++)
        {
            int nx = cell.x + dx[d];
            int nz = cell.z + dz[d];

            // 맵 경계 밖은 Blocked
            if (nx < 0 || nx >= m_SizeX || nz < 0 || nz >= m_SizeZ)
            {
                walkable = false;
                break;
            }

            int neighborSurfaceY = GetSurfaceY(nx, nz);
            int heightDiff = std::abs(cell.y - neighborSurfaceY);

            // 이웃과 높이차가 복셀 2개 이상이면 경사가 너무 가파름
            if (heightDiff > 2)
            {
                walkable = false;
                break;
            }
        }

        // 조건 2 — 머리 위 공간 확인 (NPC 키 높이만큼 비어있어야 함)
        // 지금은 2칸만 확인, 나중에 NPC 키에 따라 조절 가능
        if (true == walkable)
        {
            int headY = cell.y + 2;
            if (headY < m_SizeY)
            {
                // 머리 위에 복셀이 있으면 이동 불가 (천장)
                if (m_Grid[Index(cell.x, headY, cell.z)] == CellType::Blocked)
                {
                    // 단, 이 경우는 내부 채우기로 생긴 Blocked인지
                    // 진짜 지형 복셀인지 구분이 필요함
                    // GetSurfaceY보다 높은 y의 Blocked는 진짜 지형
                    int surfY = GetSurfaceY(cell.x, cell.z);
                    if (headY <= surfY)
                    {
                        walkable = false;
                    }
                }
            }
        }

        // 조건 3 — TODO : 장애물 마킹 (동적 변경 시)

        cell.type = walkable ? CellType::Walkable : CellType::Blocked;

        // 3D 그리드에도 반영 (FlowField 계산 시 빠른 접근용)
        m_Grid[Index(cell.x, cell.y, cell.z)] = cell.type;
    }

}

bool VoxelGrid::IsSurface(int x, int y, int z) const
{
    // 위쪽이 Empty이면 표면
    int above = y + 1;
    if (above >= m_SizeY) return true;
    return m_Grid[Index(x, above, z)] == CellType::Empty;
}

int VoxelGrid::GetSurfaceY(int x, int z) const
{
    // 위에서 아래로 내려오면서 처음으로 채워진 복셀 찾기
    for (int y = m_SizeY - 1; y >= 0; y--)
    {
        if (m_Grid[Index(x, y, z)] != CellType::Blocked)
        {
            continue;
        }
        // 이 x,z에 복셀이 하나라도 있으면
        // BuildCells에서 y=0~voxelY까지 채웠으니
        // 가장 위가 표면
        return y;
    }
    return 0;
}

void VoxelGrid::BuildInstanceList(std::vector<VoxelRenderer::InstanceData>& outInstances) const
{
    outInstances.clear();
    outInstances.reserve(m_Cells.size());

    for (const auto& cell : m_Cells)
    {
        // Empty는 렌더링하지 않음
        if (cell.type == CellType::Empty) continue;

        VoxelRenderer::InstanceData inst = {};
        inst.position[0] = cell.x * m_CellSize;
        inst.position[1] = cell.y * m_CellSize;
        inst.position[2] = cell.z * m_CellSize;
        inst.scale = m_CellSize;

        // Empty=0, Walkable=1(흰색), Blocked=2(빨강)
        inst.colorType = (cell.type == CellType::Walkable) ? 0 : 1;
        outInstances.push_back(inst);
    }
}


void VoxelGrid::SetCell(int x, int y, int z, CellType type)
{
    if (x < 0 || x >= m_SizeX) return;
    if (y < 0 || y >= m_SizeY) return;
    if (z < 0 || z >= m_SizeZ) return;
    m_Grid[Index(x, y, z)] = type;
}

VoxelGrid::CellType VoxelGrid::GetCell(int x, int y, int z) const
{
    if (x < 0 || x >= m_SizeX) return CellType::Blocked;
    if (y < 0 || y >= m_SizeY) return CellType::Blocked;
    if (z < 0 || z >= m_SizeZ) return CellType::Blocked;
    return m_Grid[Index(x, y, z)];
}

bool VoxelGrid::IsWalkable(int x, int y, int z) const
{
    return GetCell(x, y, z) == CellType::Walkable;
}

Math::Vector3 VoxelGrid::GetWorldPos(int x, int y, int z) const
{
    return Math::Vector3(x * m_CellSize, y * m_CellSize,z * m_CellSize);
}
