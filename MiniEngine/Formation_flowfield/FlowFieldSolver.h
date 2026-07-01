#pragma once
#include "VectorMath.h"
#include <vector>

class VoxelGrid;

// TODO : 각 복셀 맵에 방향값을 어떻게 넣어야 효율적일지 고민 필요.
class FlowFieldSolver
{
public:
    void Initialize(const VoxelGrid* grid);
    void Shutdown();

    // TODO : 목적지 설정 + 플로우필드 계산
    void Solve(int goalX, int goalZ);

    // TODO : NPC 위치에서 방향 벡터 샘플링
    Math::Vector3 SampleDirection(const Math::Vector3& worldPos) const;

    bool IsReady() const { return m_IsReady; }

private:
    const VoxelGrid* m_Grid = nullptr;
    std::vector<float>          m_CostField;
    std::vector<Math::Vector3>  m_DirectionField;
    bool m_IsReady = false;
};
