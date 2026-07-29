#pragma once
#include "NpcRenderer.h"
#include "VoxelGrid.h"
#include "CellReservation.h"
#include "NpcTypes.h"
#include "NpcMovementSolver.h"
#include "LeafSplitController.h"
#include <DirectXMath.h>
#include <unordered_set>
#include <vector>



class NpcManager
{
public:
    NpcManager();

	// 지형 참조 보관 + npc 인스턴스 저장
	void Init(const VoxelGrid& grid);

	// 좌클릭 선택
	bool TrySelectNpc(const Math::Vector3& rayOrigin, const Math::Vector3& rayDir);
    bool HasSelection() const { return m_GroupSelected; }

	// 우클릭 목적지 확정 -> A* -> 마스크 -> FlowField -> 이동초기화 파이프라인 전체 실행.
	bool SetGroupDestination(const DirectX::XMINT3& goalCell, std::vector<DirectX::XMINT3>* outPath = nullptr);

	// 매 프레임 이동
	void Update(float dt);

	// -- 시각화 / 렌더용 접근자 --
    // leaf는 내부 구현이지만, 시각화는 그룹 전체 상태를 그려야 하므로 개수와 인덱스 접근만 열어두기
    int GetLeafCount() const { return (int)m_Leaves.size(); }
    const CorridorFlowField& GetLeafField(int idx) const { return m_Leaves[idx]->field; }

    // 그 셀이 어느 leaf에 포함되면 true (시각화 || 판정)
    bool IsVisitedAny(const VoxelGrid& grid, int x, int y, int z) const;
    // 그 셀을 포함하는 첫 leaf의 방향(시각화용)
    bool SampleDirectionAny(const VoxelGrid& grid, int x, int y, int z, DirectX::XMINT3& outDir) const;

    const std::vector<NpcRenderer::InstanceData>& GetInstances() const { return m_NpcInstances; }
    bool HasGoal() const { return m_Group.hasGoal; }


private:
    const VoxelGrid* m_Grid = nullptr;   // 참조만 (소유 X)

    std::vector<NpcRenderer::InstanceData> m_NpcInstances;
    int m_SelectedNpcIndex = -1;
    bool m_GroupSelected = false;

    std::vector<std::unique_ptr<LeafGroup>> m_Leaves;
    NpcGroup               m_Group;

    NpcMoveData m_Move;   // SoA 이동 데이터

    // -------- NPC 이동을 위한 복셀 예약 ----------
    CellReservation m_Reserve;      // 각 복셀마다 npc 예약 리스트
    std::vector<int> m_MoveOrder;   // 어디가 앞인지 정의할 npc 리스트

    std::vector<DirectX::XMINT3> m_StartCells;

    // NpcManager 비대화에 따라 분리된 협력 객체 - 데이터는 NpcManager가 계속 소유, 로직만 위임
    NpcMovementSolver    m_MovementSolver;    // 프레임 단위 셀 이동 판정
    LeafSplitController  m_SplitController;   // leaf 구축 + 병목 분리

    // 내부 헬퍼
    void InitGroupMovement();
    void SyncInstances();   // SoA position -> m_NpcInstances -> UpdateInstances
    Math::Vector3 GetNpcStandPos(const DirectX::XMINT3& cell, float halfHeight) const;

};
