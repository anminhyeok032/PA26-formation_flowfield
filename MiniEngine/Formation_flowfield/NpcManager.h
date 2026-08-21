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
#include "ScopedCpuTimer.h"
#include "MemoryProbe.h"

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

    bool IsVisitedAny(const VoxelGrid& grid, int x, int y, int z) const;

    bool SampleDirectionAny(const VoxelGrid& grid, int x, int y, int z, DirectX::XMINT3& outDir) const;

	// -- 시각화 / 렌더용 접근자 --
    // leaf는 내부 구현이지만, 시각화는 그룹 전체 상태를 그려야 하므로 개수와 인덱스 접근만 열어두기
    int GetLeafCount() const { return (int)m_Leaves.size(); }
    const CorridorFlowField& GetLeafField(int idx) const { return *m_Leaves[idx]->field; }



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

    void ReportAndResetStats(const char* filePath = "Report/advancecell_stats.txt")
    {
        auto& s = m_Stats;
        char buf[512];
        sprintf_s(buf,
            "AdvanceCell calls=%llu\n"
            "  SampleCost=%llu SampleDir=%llu ReserveFind=%llu  (avg hash/call=%.2f)\n"
            "  primaryHit=%llu waited=%llu fallbackHit=%llu arrived=%llu stuck=%llu\n",
            s.calls,
            s.sampleCostCalls, s.sampleDirCalls, s.reserveFindCalls,
            s.calls ? (double)(s.sampleCostCalls + s.sampleDirCalls + s.reserveFindCalls) / s.calls : 0.0,
            s.primaryHit, s.waited, s.fallbackHit, s.arrived, s.stuck);

        Utility::Print(buf);
        FILE* fp = nullptr;
        fopen_s(&fp, filePath, "w");
        if (fp) { fputs(buf, fp); fclose(fp); }

        s = AdvanceCellStats{};   // 리셋
    }
    void ReportMemory(const char* filePath = "Report/mem_npc.txt") const;

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

    // near 필드 - 목표 주변 갱신용
    std::shared_ptr<const CorridorFlowField> m_NearField;
    // far 전용 목표점
    DirectX::XMINT3 m_FarFieldGoal{ -1, -1, -1 };
    // far가 낡았는지(목표가 원점에서 너무 멀어졌는지) 판정
    bool IsFarFieldGoalTooOld() const;

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

    // 패닉 자극원 - TODO : 여러개 만들고 싶으면 확장 해야한다
    DirectX::XMINT3 m_PanicCell{ -1, -1, -1 };

    void UpdateAgro();
    void PropagateAgro(float dt);
    void UpdateDeagro(const DirectX::XMINT3& playerCell, float dt);
    void RebuildChaseLeaf();

    // state를 직접 대입하는 코드는 이 함수 밖에 두지 않는다 (propagationTimer/blockedTime/active/stopReason)
    void SetNpcState(size_t i, uint8_t next);


    // 스폰 헬퍼 - Init 비대 방지용
    bool TrySpawnGroup(const VoxelGrid& grid, int seedX, int seedY, int seedZ,
        int wantCount, float npcWidth, float npcHeight,
        std::unordered_set<int64_t>& usedColumns);


    // ------ 공간 분할 -------
    struct RegionCell { std::vector<int> groupIndices; };
    std::vector<RegionCell> m_Regions;
    int m_RegionCountX = 0, m_RegionCountZ = 0;

    std::vector<int> m_GroupOrder;  // 그룹 인덱스 - 살아있는 순서대로 vector 앞으로 swap해서 사용
    std::vector<int> m_GroupSlot;   // grounpIdx -> m_GroupOrder - order 몇번째에 있는지 역방향 조회
    int m_ActiveGroupCount = 0;     // m_GroupOrder앞 몇개가 활성화 되어있는지 cnt

    int m_LastRegionX = INT32_MIN, m_LastRegionZ = INT32_MIN;


    // 리전은 청크(8셀)와 별개 격자 - (청크 = 필드/그래프 메모리 블록, 리전 = 어그로 반경).
    int RegionIndexOf(int cellX, int cellZ) const
    {
        const int rx = std::min(std::max(cellX / REGION_SIZE_CELLS, 0), m_RegionCountX - 1);
        const int rz = std::min(std::max(cellZ / REGION_SIZE_CELLS, 0), m_RegionCountZ - 1);
        return rz * m_RegionCountX + rx;
    }

    // GroupOrder 앞으로 swap
    void ActivateGroup(int g);
    // GroupOrder 뒤로 swap
    void DeactivateGroup(int g);
    void RefreshActiveSet(const DirectX::XMINT3& playerCell);


    // ---- AdvanceCell 계측용 카운터 (디버그) ----
    // 타이머 오버헤드(~100~160ns)가 AdvanceCell 자체 비용(~280ns)의 35~55%를 차지해
    // CPU_SCOPE로는 못 재므로, 정수 증가(1~2ns, 무시 가능)로 호출 빈도/분기 분포를 잰다.
    struct AdvanceCellStats
    {
        uint64_t calls = 0;          // AdvanceCell 진입 횟수
        uint64_t sampleCostCalls = 0;    // CorridorFlowField::SampleCost 호출 횟수
        uint64_t sampleDirCalls = 0;     // CorridorFlowField::SampleDirection 호출 횟수
        uint64_t reserveFindCalls = 0;   // CellReservation::Find 호출 횟수 (IsMoveCross 내부 포함)

        uint64_t primaryHit = 0;     // 1순위(FlowField 방향) 즉시 성공
        uint64_t waited = 0;         // 활성 상대에게 막혀 대기
        uint64_t slideHit = 0;       // 성분 분해(zKeep/xKeep)로 성공
        uint64_t fallbackHit = 0;    // 3순위 cost 폴백으로 성공
        uint64_t arrived = 0;        // 도착 확정 (active=0)
        uint64_t stuck = 0;          // 폴백도 실패, 활성 방해자 있어 제자리 대기
    };
    AdvanceCellStats m_Stats;

    bool SampleCostCounted(int x, int y, int z, float& outCost);
    bool SampleDirectionCounted(int x, int y, int z, DirectX::XMINT3& outDir);
    int ReserveFindCounted(int64_t key);
};
