#pragma once
#include "VectorMath.h"
#include "VoxelRenderer.h"
#include <vector>

// TODO : 복셀을 3d A*에 활용하기 위해 복셀의 메모리를 어떻게 구성할지 고민이 필요함.
class VoxelGrid
{
public:
    enum class CellType : uint8_t
    {
        Walkable = 0,
        Blocked = 1,
    };

    void Initialize(int sizeX, int sizeY, int sizeZ, float cellSize);
    void Shutdown();

    // 메시 기반 복셀화
    // void BuildFromMesh(const ModelMesh& mesh);

    // 테스트용 평면 그리드 생성
    // void BuildFlatGrid(int sizeX, int sizeZ);

    // TODO - 지형 동적 변경
    void SetCell(int x, int y, int z, CellType type);

    // 렌더용 인스턴스 목록 생성
    void BuildInstanceList(std::vector<VoxelRenderer::InstanceData>& outInstances) const;

    // 플로우필드 계산용 접근자
    CellType GetCell(int x, int y, int z) const;
    bool IsWalkable(int x, int y, int z) const;
    Math::Vector3 GetWorldPos(int x, int y, int z) const;

    int GetSizeX() const { return m_SizeX; }
    int GetSizeZ() const { return m_SizeZ; }
    float GetCellSize() const { return m_CellSize; }

private:
    std::vector<CellType> m_Cells;
    int   m_SizeX = 0;
    int   m_SizeY = 0;
    int   m_SizeZ = 0;
    float m_CellSize = 1.0f;

    int Index(int x, int y, int z) const
    {
        return x + m_SizeX * (z + m_SizeZ * y);
    }
};
