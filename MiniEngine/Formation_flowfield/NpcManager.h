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

#include "FieldWorker.h"


class NpcManager
{
public:
    NpcManager();

	// 지형 참조 보관 + npc 인스턴스 저장
	void Init(const VoxelGrid& grid, const ChunkGraph& chunkgraph, const DirectX::XMINT3& playerStartCell);

	// 좌클릭 선택
	bool TrySelectNpc(const Math::Vector3& rayOrigin, const Math::Vector3& rayDir);
    bool HasSelection() const { return m_GroupSelected; }

    // 현재는 전원 강제 어그로 버튼으로만 사용
	bool SetGroupDestination(const DirectX::XMINT3& goalCell, std::vector<DirectX::XMINT3>* outPath = nullptr);

	// 매 프레임 이동 - Agro 판정 포함
	void Update(float dt, const DirectX::XMINT3& playerCell, bool playerValid);

	// -- 시각화 / 렌더용 접근자 --
    // leaf는 내부 구현이지만, 시각화는 그룹 전체 상태를 그려야 하므로 개수와 인덱스 접근만 열어두기
    int GetLeafCount() const { return (int)m_Leaves.size(); }
    const CorridorFlowField& GetLeafField(int idx) const { return *m_Leaves[idx]->field; }

    // 그 셀이 어느 leaf에 포함되면 true (시각화 || 판정)
    bool IsVisitedAny(const VoxelGrid& grid, int x, int y, int z) const;
    // 그 셀을 포함하는 첫 leaf의 방향(시각화용)
    bool SampleDirectionAny(const VoxelGrid& grid, int x, int y, int z, DirectX::XMINT3& outDir) const;

    const std::vector<NpcRenderer::InstanceData>& GetInstances() const { return m_NpcInstances; }
    bool HasGoal() const { return m_Group.hasGoal; }
    const DirectX::XMINT3& GetGoal() const { return m_Group.goal; }

    // 동적 지형 생성에 대한 갱신 ( TODO : 다중 그룹으로 변환시, 해당 갱신은 상위에서 한번에)
    void OnTerrainChanged(const std::vector<DirectX::XMINT3>& editedCells);

    // Trigger 등록 - Update에서 소비
    // 메인 스레드용
    void PushStimulus(const Stimulus& s) { m_Stimulus.push_back(s); }


    // 지형 / 그래프 수정 직전 호출 - worker가 그리드 읽을때까지 block용
    void CancelFieldWork() { m_FieldWorker.CancelAndWait(); }

    // 시각화 갱신 트리거용
    bool ConsumeFieldSwapped() 
    {
        bool v = m_FieldSwapped;
        m_FieldSwapped = false;
        return v;
    }

    bool ConsumeGroupArrived()
    {
        bool v = m_GroupArrived;
        m_GroupArrived = false;
        return v;
    }


private:
    const VoxelGrid* m_Grid = nullptr;   // 참조만 (소유 X)
    const ChunkGraph* m_ChunkGraph = nullptr;

    std::vector<NpcRenderer::InstanceData> m_NpcInstances;
    int m_SelectedNpcIndex = -1;
    bool m_GroupSelected = false;
    bool m_VisualDirty = false;

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

    // flowfield 계산 전담 워커
    FieldWorker m_FieldWorker;
    bool m_FieldSwapped = false;    // 시각화 갱신용
    bool m_GroupArrived = false;    // 시각화 해제용

    // 마지막으로 워커에 submit한 목표셀
    DirectX::XMINT3 m_LastRequestedGoal{ -1, -1, -1 };

    // 지형 / 그래프 변경시 증가 - 결과 판정 확인용
    uint64_t m_TerrainGeneration = 0;

    // 내부 헬퍼
    void SyncInstances();   // SoA position -> m_NpcInstances -> UpdateInstances
    Math::Vector3 GetNpcStandPos(const DirectX::XMINT3& cell, float halfHeight) const;



    // ------ State 처리 ------
    std::vector<BehaviorGroup> m_BehaviorGroups;
    float m_FieldRequestTimer = 0.0f;   // 요청 배칭 쿨다운
    bool m_ChaseSetDirty = false;       // Chase 집합 변경 대기 플래그

    std::vector<Stimulus> m_Stimulus;   // update에서 소비하는 trigger

    void UpdateAgro();
    void PropagateAgro(float dt);
    void UpdateDeagro(const DirectX::XMINT3& playerCell, float dt);
    void RebuildChaseLeaf();

    // 스폰 헬퍼 - Init 비대 방지용
    bool TrySpawnGroup(const VoxelGrid& grid, int seedX, int seedY, int seedZ,
        int wantCount, float npcWidth, float npcHeight,
        std::unordered_set<int64_t>& usedColumns);


    // 공간 분할
    struct RegionCell { std::vector<int> groupIndices; };
    std::vector<RegionCell> m_Regions;
    int m_RegionCountX = 0, m_RegionCountZ = 0;

    std::vector<int> m_GroupOrder;  // 그룹 인덱스 - 살아있는 순서대로 vector 앞으로 swap해서 사용
    std::vector<int> m_GroupSlot;   // grounpIdx -> m_GroupOrder - order 몇번째에 있는지 역방향 조회
    int m_ActiveGroupCount = 0;     // m_GroupOrder앞 몇개가 활성화 되어있는지 cnt

    int m_LastRegionX = INT32_MIN, m_LastRegionZ = INT32_MIN;
    //std::vector<int> m_PersistentGroups; // 추격 및 경계 상태

    // GroupOrder 앞으로 swap
    void ActivateGroup(int g);
    // GroupOrder 뒤로 swap
    void DeactivateGroup(int g);
    void RefreshActiveSet(const DirectX::XMINT3& playerCell);
};
