#pragma once
#include "NpcRenderer.h"
#include "VoxelGrid.h"
#include "AStarPathfinder.h"
#include "PathCorridor.h"
#include "CorridorFlowField.h"
#include <DirectXMath.h>
#include <vector>

struct NpcMoveData
{
    std::vector<Math::Vector3>      position;           // 현재 좌표
    std::vector<Math::Vector3>      targetWorldPos;     // 목표 셀 월드 좌표
    std::vector<DirectX::XMINT3>    currCell;           // 현재 셀
    std::vector<DirectX::XMINT3>    targetCell;         // 목표 셀
    std::vector<uint8_t>            active;             // 이동중 = 1;
    std::vector<float>              halfHeight;         // scaleY 복사본
    size_t count = 0;

    void Resize(size_t n)
    {
        position.resize(n);
        targetWorldPos.resize(n);
        currCell.resize(n);
        targetCell.resize(n);
        active.resize(n);
        halfHeight.resize(n);
        count = n;
    }

};

class NpcManager
{
public:
	// 지형 참조 보관 + npc 인스턴스 저장
	void Init(const VoxelGrid& grid);

	// 좌클릭 선택
	bool TrySelectNpc(const Math::Vector3& rayOrigin, const Math::Vector3& rayDir);
    bool HasSelection() const { return m_SelectedNpcIndex >= 0; }

	// 우클릭 목적지 확정 -> A* -> 마스크 -> FlowField -> 이동초기화 파이프라인 전체 실행.
	bool SetGroupDestination(const DirectX::XMINT3& goalCell, std::vector<DirectX::XMINT3>* outPath = nullptr);

	// 매 프레임 이동
	void Update(float dt);

	// -- 시각화 / 렌더용 접근자 -- 
    const CorridorFlowField& GetFlowField() const { return m_CorridorField; }
    const std::vector<NpcRenderer::InstanceData>& GetInstances() const { return m_NpcInstances; }
    bool HasGoal() const { return m_HasGoal; }

private:
    const VoxelGrid* m_Grid = nullptr;   // 참조만 (소유 X)

    std::vector<NpcRenderer::InstanceData> m_NpcInstances;
    int m_SelectedNpcIndex = -1;

    AStarPathfinder   m_Pathfinder;
    CorridorFlowField m_CorridorField;
    bool m_HasGoal = false;

    NpcMoveData m_Move;   // SoA 이동 데이터

    // 내부 헬퍼
    void InitGroupMovement();
    void AdvanceCell(size_t i);
    void SyncInstances();   // SoA position → m_NpcInstances → UpdateInstances
    Math::Vector3 GetNpcStandPos(const DirectX::XMINT3& cell, float halfHeight) const;

};
