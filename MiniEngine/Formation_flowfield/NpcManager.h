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

    std::vector<uint16_t> blockedFrames;

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

        blockedFrames.assign(n, 0);
    }

};

struct ArrivalRegion
{
    std::vector<DirectX::XMINT3> slots;     // 채울 순서
    std::unordered_set<int64_t> slotKeys;   // 영역 소속 o(1) 판정을 위해
    int nextSlot = 0;                       // 청구 포인터 - 순서 담음
    bool valid = false;                    

    void Clear()
    { 
        slots.clear();
        slotKeys.clear();
        nextSlot = 0;
        valid = false;
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
    const CorridorFlowField& GetFlowField() const { return m_CorridorField; }
    const std::vector<NpcRenderer::InstanceData>& GetInstances() const { return m_NpcInstances; }
    bool HasGoal() const { return m_HasGoal; }


private:
    const VoxelGrid* m_Grid = nullptr;   // 참조만 (소유 X)

    std::vector<NpcRenderer::InstanceData> m_NpcInstances;
    int m_SelectedNpcIndex = -1;
    bool m_GroupSelected = false;

    AStarPathfinder   m_Pathfinder;
    CorridorFlowField m_CorridorField;
    bool m_HasGoal = false;

    NpcMoveData m_Move;   // SoA 이동 데이터

    // -------- NPC 이동을 위한 복셀 예약 ----------
    CellReservation m_Reserve;      // 각 복셀마다 npc 예약 리스트
    std::vector<int> m_MoveOrder;   // 어디가 앞인지 정의할 npc 리스트

    // 조건 조립 함수
    void AdvanceCell(size_t i);
    void SnapToTargetCell(size_t i);                                // 이동
    void HoldPosition(size_t i, const DirectX::XMINT3& curr);       // 제자리 대기
    // 특정 이웃 셀이 지금 이동 가능한 유효 후보인지 검사
    bool TryCandidate(size_t i, const DirectX::XMINT3& curr, float currCost,
        const DirectX::XMINT3& cand, DirectX::XMINT3& out) const;

    // 1순위(FlowField 방향) + 대기 판정 + 성분 분해 슬라이딩 담당
    // return true = best에 후보 확정 / outWaited = true면 이번 프레임 대기 확정
    bool TryPrimaryDirection(size_t i, const DirectX::XMINT3& curr, float currCost,
        DirectX::XMINT3& best, bool& outWaited);

    // 2순위(cost 최소 + lastDir 정렬) 폴백
    bool PickFallbackCell(size_t i, const DirectX::XMINT3& curr, float currCost,
        DirectX::XMINT3& best, bool& outHasActiveBlocker);
    // 이동할 복셀 예약
    void CommitMove(size_t i, const DirectX::XMINT3& curr, const DirectX::XMINT3& best);
    // 두 예약이 대각선 교차해서 지나가는지 확인
    bool IsMoveCross(size_t i, const DirectX::XMINT3& curr,
        const DirectX::XMINT3& next) const;


    std::vector<NeighborInfo> m_NeighborScratch;   // AdvanceCell 전용 임시 버퍼


    // 내부 헬퍼
    void InitGroupMovement();
    void SyncInstances();   // SoA position -> m_NpcInstances -> UpdateInstances
    Math::Vector3 GetNpcStandPos(const DirectX::XMINT3& cell, float halfHeight) const;

    std::vector<DirectX::XMINT3> m_StartCells;

};
