#pragma once
#include "NpcRenderer.h"
#include "VoxelGrid.h"
#include "AStarPathfinder.h"
#include "PathCorridor.h"
#include "CorridorFlowField.h"
#include "CellReservation.h"
#include "GridNeighbors.h"
#include <DirectXMath.h>
#include <unordered_set>
#include <vector>



struct NpcMoveData
{
    std::vector<Math::Vector3>      position;           // 현재 좌표
    std::vector<Math::Vector3>      targetWorldPos;     // 목표 셀 월드 좌표
    std::vector<DirectX::XMINT3>    currCell;           // 현재 셀
    std::vector<DirectX::XMINT3>    targetCell;         // 목표 셀
    std::vector<uint8_t>            active;             // 이동중 = 1;
    std::vector<float>              halfHeight;         // scaleY 복사본

    std::vector<int> claimedSlot;                       // -1 = 미청구
    std::vector<DirectX::XMINT3> lastDir;               // 직전 이동 방향 (dx,dy,dz)

    std::vector<float> blockedTime;
    std::vector<float> congestionTime;                  // 분리 트리거용 (길음)
    std::vector<int> leafId;                            // 해당 NPC가 속한 leaf 인덱스(-1 = 무소속)

    std::vector<uint8_t> stopReason;                    // 0=정상도착, 1=필드밖(무효화)

    size_t size() const { return position.size(); }     // 길막당한 프레임 체크용

    void Resize(size_t n)
    {
        position.resize(n);
        targetWorldPos.resize(n);
        currCell.resize(n);
        targetCell.resize(n);
        active.resize(n);
        halfHeight.resize(n);

        claimedSlot.assign(n, -1);
        lastDir.resize(n, { 0,0,0 });

        blockedTime.assign(n, 0.0f);
        congestionTime.assign(n, 0.0f);

        leafId.resize(n, -1);
        stopReason.resize(n, 0);
    }

};


// path / flowfield 소유 단위 - 여러 leaf여도 외부에는 하나로 보이게 캡슐화
struct LeafGroup
{
    int                             leafId = -1;
    int                             parentId = -1;
    CorridorFlowField               field;      // leaf 소유 flowfield 
    std::vector<int>                members;    // Npc index
    std::unordered_set<int64_t>     excluded;   // 분리시, 막은(벽) 병목 복셀들(head는 항상 empty)
    int                             depth = 0;  // leaf depth (head==0)
    std::vector<DirectX::XMINT3>    path;       // 이 leaf의 A* 경로(분리시, 겹칩 판별)
    bool                            active = false;

    void Reset()
    {
        members.clear();
        excluded.clear();
        path.clear();
        depth = 0;
        active = false;
    }
};

// 사용자 인식 기준 단위 - 선택/dest/도착 단위
struct NpcGroup
{
    int                         groupId = 0;
    DirectX::XMINT3             goal{ 0,0,0 };
    std::vector<int>            leafIds;
    bool                        hasGoal = false;

    void Reset()
    {
        leafIds.clear();
        hasGoal = false;
    }
};


class NpcManager
{
public:
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

    AStarPathfinder   m_Pathfinder;
    std::vector<std::unique_ptr<LeafGroup>> m_Leaves;
    NpcGroup               m_Group;

    NpcMoveData m_Move;   // SoA 이동 데이터

    // -------- NPC 이동을 위한 복셀 예약 ----------
    CellReservation m_Reserve;      // 각 복셀마다 npc 예약 리스트
    std::vector<int> m_MoveOrder;   // 어디가 앞인지 정의할 npc 리스트

    // 조건 조립 함수
    void AdvanceCell(size_t i, float dt);
    void SnapToTargetCell(size_t i);                                // 이동
    void HoldPosition(size_t i, const DirectX::XMINT3& curr);       // 제자리 대기
    // 특정 이웃 셀이 지금 이동 가능한 유효 후보인지 검사
    bool TryCandidate(size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
        const DirectX::XMINT3& cand, DirectX::XMINT3& out) const;

    // 1순위(FlowField 방향) + 대기 판정 + 성분 분해 슬라이딩 담당
    // return true = best에 후보 확정 / outWaited = true면 이번 프레임 대기 확정
    bool TryPrimaryDirection(size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
        DirectX::XMINT3& best, bool& outWaited, float dt);

    // 2순위(cost 최소 + lastDir 정렬) 폴백
    bool PickFallbackCell(size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
        DirectX::XMINT3& best, bool& outHasActiveBlocker);
    // 이동할 복셀 예약
    void CommitMove(size_t i, const DirectX::XMINT3& curr, const DirectX::XMINT3& best);
    // 두 예약이 대각선 교차해서 지나가는지 확인
    bool IsMoveCross(size_t i, const DirectX::XMINT3& curr,
        const DirectX::XMINT3& next) const;


    bool BuildLeafCorridor(LeafGroup& leaf, const DirectX::XMINT3& goalCell, 
        std::vector<DirectX::XMINT3>* outPath = nullptr);


    std::vector<NeighborInfo> m_NeighborScratch;   // AdvanceCell 전용 임시 버퍼


    // 내부 헬퍼
    void InitGroupMovement();
    void SyncInstances();   // SoA position -> m_NpcInstances -> UpdateInstances
    Math::Vector3 GetNpcStandPos(const DirectX::XMINT3& cell, float halfHeight) const;

    std::vector<DirectX::XMINT3> m_StartCells;


private:
    // TODO : NpcManager 비대화에 따른 파일 분리 및 리팩토링 할것
    // --- 분리 제한 요소 ---
    static constexpr int    MIN_SPLIT_SIZE = 4;         // 분리 최소 인원
    static constexpr int    MAX_SPLIT_DEPTH = 2;        // 최대 분리 횟수
    static constexpr int    MAX_LEAVES      = 8;        // 메모리 상한(leaf당 flowfield 하나)
    static constexpr int    BOTTLENECK_SPREAD = 10;      // 병목시 단면 확보용 bfs 반경
    static constexpr float  PATH_OVERLAP_LIMIT = 0.5f; // (0.0~1.0) path 겹침 ratio -> 이 이상 겹치면 분할x
    static constexpr float  NEW_PATH_LIMIT = 10.0f;      // 우회경로가 n배 보다 길면 분할 포기
    static constexpr float  SPLIT_BLOCK_TIME = 2.0f;    // 이 시간 이상 막히면 분리 검토

    // a* 실행
    bool FindLeafPath(LeafGroup& leaf, const DirectX::XMINT3& goalCell, 
        std::vector<DirectX::XMINT3>& outPath, DirectX::XMINT3& outCentroidCell);

    // margin / mask / field 구축
    void BuildLeafField(LeafGroup& leaf, const DirectX::XMINT3& goalCell,
        const DirectX::XMINT3& centroidCell);

    // 막힌 Npc들이 가려던 방향을 BFS로 넓혀서 병목 단면 수집용
    void CollectBottleneckCells(const std::vector<int>& stuckMem,
        std::unordered_set<int64_t>& outCells) const;

    // 막힌 멤버들을 새 leaf로 분리 시도 - 실패하면 false
    bool TrySplitLeaf(int leafIdx, const std::vector<int>& stuckMem);

    // leafId와 members를 항상 함께 갱신 — 둘 중 하나만 바뀌는 사고 방지
    void ReassignMember(int npcIdx, int fromLeaf, int toLeaf);

    void CheckSplitTriggers();   // 이동 루프 종료 후 호출
    bool AreSpatiallyClustered(const std::vector<int>& members) const;
};
