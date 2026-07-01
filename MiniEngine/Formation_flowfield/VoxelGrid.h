#pragma once
#include "VectorMath.h"
#include "VoxelRenderer.h"
#include <vector>

class HeightMap;

// TODO : 복셀을 3d A*에 활용하기 위해 복셀의 메모리를 어떻게 구성할지 고민이 필요함.
class VoxelGrid
{
public:
    enum class CellType : uint8_t
    {
        Walkable = 0,
        Blocked = 1,
    };

    // 기존
    void Initialize(int sizeX, int sizeY, int sizeZ, float cellSize);
    void Shutdown();
    // TODO - 지형 동적 변경
    void SetCell(int x, int y, int z, CellType type);
    // 렌더용 인스턴스 목록 생성
    void BuildInstanceList(std::vector<VoxelRenderer::InstanceData>& outInstances) const;
    CellType      GetCell(int x, int y, int z) const;
    bool          IsWalkable(int x, int y, int z) const;
    Math::Vector3 GetWorldPos(int x, int y, int z) const;
    int   GetSizeX()    const { return m_SizeX; }
    int   GetSizeZ()    const { return m_SizeZ; }
    float GetCellSize() const { return m_CellSize; }

    // HeightMap 기반 복셀 생성
    void BuildFromHeightMap(const HeightMap& hm);

private:
    // 표면 복셀 하나만 저장 (위치 + 타입)
    struct VoxelCell
    {
        int      x, y, z;
        CellType type;
    };

    std::vector<VoxelCell>  m_Cells;    // 기존 구조와 병행
    std::vector<CellType>   m_Grid;     // 3D 그리드 (A* 등 용도)

    int   m_SizeX = 0;
    int   m_SizeY = 0;
    int   m_SizeZ = 0;
    float m_CellSize = 1.0f;

    int Index(int x, int y, int z) const
    {
        return x + m_SizeX * (z + m_SizeZ * y);
    }
};
