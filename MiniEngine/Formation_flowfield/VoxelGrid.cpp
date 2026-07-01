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
    m_Cells.clear();

    m_SizeX = hm.GetWidth();
    m_SizeZ = hm.GetDepth();
    m_CellSize = hm.GetCellSize();

    // 최대 높이를 셀 단위로 변환해서 Y 그리드 크기 결정
    m_SizeY = (int)(hm.GetMaxHeight() / m_CellSize) + 1;

    for (int z = 0; z < m_SizeZ; z++)
    {
        for (int x = 0; x < m_SizeX; x++)
        {
            float worldH = hm.GetHeight(x, z);

            // 높이값 → 복셀 Y 인덱스
            int voxelY = (int)(worldH / m_CellSize);
            voxelY = std::max(0, voxelY);

            // 이웃 셀과 경사각으로 walkable 판정
            // 경사가 45도(기울기 1.0) 이상이면 Blocked
            float hRight = hm.GetHeight(std::min(x + 1, m_SizeX - 1), z);
            float hFront = hm.GetHeight(x, std::min(z + 1, m_SizeZ - 1));
            float slopeX = std::abs(worldH - hRight) / m_CellSize;
            float slopeZ = std::abs(worldH - hFront) / m_CellSize;
            float maxSlope = std::max(slopeX, slopeZ);

            CellType surfaceType = (maxSlope < 1.0f)
                ? CellType::Walkable
                : CellType::Blocked;

            // y = 0 부터 voxelY 까지 전부 채우기
            for (int y = 0; y <= voxelY; y++)
            {
                VoxelCell cell;
                cell.x = x;
                cell.y = y;
                cell.z = z;

                // 표면(voxelY)은 경사각 기반 타입
                // 내부(y < voxelY)는 Blocked (지형 내부라 이동 불가)
                cell.type = (y == voxelY) ? surfaceType : CellType::Blocked;

                m_Cells.push_back(cell);
            }
        }
    }
}

void VoxelGrid::BuildInstanceList(std::vector<VoxelRenderer::InstanceData>& outInstances) const
{
    outInstances.clear();
    outInstances.reserve(m_Cells.size());

    for (const auto& cell : m_Cells)
    {
        VoxelRenderer::InstanceData inst = {};
        inst.position[0] = cell.x * m_CellSize;
        inst.position[1] = cell.y * m_CellSize;
        inst.position[2] = cell.z * m_CellSize;
        inst.scale = m_CellSize;
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
    return Math::Vector3(
        x * m_CellSize,
        y * m_CellSize,
        z * m_CellSize);
}
