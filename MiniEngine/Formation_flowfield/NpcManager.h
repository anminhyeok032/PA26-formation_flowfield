#pragma once
#include "NpcRenderer.h"
#include "VoxelGrid.h"
#include "AStarPathfinder.h"
#include "PathCorridor.h"
#include "CorridorFlowField.h"
#include "CellReservation.h"
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
    size_t count = 0;

    std::vector<int> claimedSlot;                       // -1 = 미청구

    void Resize(size_t n)
    {
        position.resize(n);
        targetWorldPos.resize(n);
        currCell.resize(n);
        targetCell.resize(n);
        active.resize(n);
        halfHeight.resize(n);
        count = n;
        claimedSlot.assign(n, -1);
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

    //--- 슬롯 상태 조회용 (시각화 전용, 읽기 전용)----
    enum class SlotState : uint8_t { Unclaimed, Claimed, Arrived };
    const std::vector<DirectX::XMINT3>& GetArrivalSlots() const { return m_Arrival.slots; }
    SlotState GetSlotState(int slotIdx) const;

private:
    const VoxelGrid* m_Grid = nullptr;   // 참조만 (소유 X)

    std::vector<NpcRenderer::InstanceData> m_NpcInstances;
    int m_SelectedNpcIndex = -1;
    bool m_GroupSelected = false;

    AStarPathfinder   m_Pathfinder;
    CorridorFlowField m_CorridorField;
    bool m_HasGoal = false;

    NpcMoveData m_Move;   // SoA 이동 데이터

    // --- NPC 이동을 위한 복셀 예약 ---
    CellReservation m_Reserve;      // 각 복셀마다 npc 예약 리스트
    std::vector<int> m_MoveOrder;   // 어디가 앞인지 정의할 npc 리스트
    bool TryAdvanceTo(size_t i, const DirectX::XMINT3& next);

    // 두 예약이 대각선 교차해서 지나가는지 확인
    bool IsMoveCross(size_t i, const DirectX::XMINT3& curr,
        const DirectX::XMINT3& next) const;


    // 내부 헬퍼
    void InitGroupMovement();
    void AdvanceCell(size_t i);
    void SyncInstances();   // SoA position → m_NpcInstances → UpdateInstances
    Math::Vector3 GetNpcStandPos(const DirectX::XMINT3& cell, float halfHeight) const;

    std::vector<DirectX::XMINT3> m_StartCells;


    // ---- 도착 데이터 정의 ----
    ArrivalRegion m_Arrival;    // 도착 데이터
    void BuildArrivalRegion(const DirectX::XMINT3& goal, int npcCount, const Math::Vector3& groupCenter);
    bool StepTowardSlot(size_t i, DirectX::XMINT3& outNext) const;

};
