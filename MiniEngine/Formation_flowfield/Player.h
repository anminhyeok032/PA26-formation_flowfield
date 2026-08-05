#pragma once
#include "VectorMath.h"
#include "NPCRenderer.h"
#include "GridNeighbors.h"
#include <DirectXMath.h>
#include <vector>

class VoxelGrid;

// 플레이어 - NPC와 같은 셀 스냅 + 보간 이동.
// flowfield가 아닌 입력을 통해 이동
class Player
{
public:
    bool Init(const VoxelGrid& grid);
    void Update(const VoxelGrid& grid, float dt);

    const DirectX::XMINT3& GetCell() const { return m_CurrCell; }
    const Math::Vector3& GetPosition() const { return m_Position; }
    bool  IsValid() const { return m_CurrCell.x >= 0; }

    // 이번 프레임에 셀이 바뀌었는지 - 읽으면 소비
    // SetChaseTarget을 매 프레임 부르지 않기 위한 신호
    bool ConsumeCellChanged() { bool v = m_CellChanged; m_CellChanged = false; return v; }

    NpcRenderer::InstanceData MakeInstance() const;

    // 이동의 앞 방향. 3인칭이면 카메라 yaw, 아니면 0(월드축)
    // Player가 카메라를 참조하면 입력 계층이 렌더 계층에 묶인다
    void SetMoveBasis(float yawRadians) { m_MoveYaw = yawRadians; }

private:
    // 입력 방향으로 갈 수 있는 이웃 셀을 고른다. 없으면 false
    bool PickInputCell(const VoxelGrid& grid, DirectX::XMINT3& outCell);

    // NpcManager::GetNpcStandPos와 같은 공식 - 다르면 플레이어만 지형에 파묻힌다
    Math::Vector3 GetStandPos(const VoxelGrid& grid, const DirectX::XMINT3& cell) const;

    DirectX::XMINT3 m_CurrCell{ -1, -1, -1 };
    DirectX::XMINT3 m_TargetCell{ -1, -1, -1 };

    Math::Vector3 m_Position{ 0.0f, 0.0f, 0.0f };
    Math::Vector3 m_TargetWorldPos{ 0.0f, 0.0f, 0.0f };

    float m_HalfHeight = 1.5f / 2.0f;   // Init에서 NPC와 같은 값으로 맞출 것
    float m_Width = 0.5f;
    float m_SpeedScale = 1.0f;   // NPC_SPEED 배수. 1.0 = 좀비와 동속
    bool  m_CellChanged = false;

    std::vector<NeighborInfo> m_NeighborScratch;   // 매 프레임 재할당 방지


    float m_MoveYaw = 0.0f;
};
