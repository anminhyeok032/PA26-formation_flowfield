#pragma once
#include "NpcTypes.h"
#include "CellReservation.h"
#include "GridNeighbors.h"
#include <DirectXMath.h>
#include <memory>
#include <vector>

// NpcManager 비대화에 따라 분리됨 - 프레임 단위 셀 이동 판정 담당
// (1순위 FlowField 방향 + 대기 판정 + 성분 분해 슬라이딩 + 2순위 폴백 + 예약)
class NpcMovementSolver
{
public:
    NpcMovementSolver(NpcMoveData& move, CellReservation& reserve);

    // 조건 조립 함수
    void AdvanceCell(const VoxelGrid& grid, const std::vector<std::unique_ptr<LeafGroup>>& leaves, size_t i, float dt);

private:
    void SnapToTargetCell(size_t i);                                // 이동
    void HoldPosition(size_t i, const DirectX::XMINT3& curr);       // 제자리 대기

    // 특정 이웃 셀이 지금 이동 가능한 유효 후보인지 검사
    bool TryCandidate(const VoxelGrid& grid, size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
        const DirectX::XMINT3& cand, DirectX::XMINT3& out) const;

    // 1순위(FlowField 방향) + 대기 판정 + 성분 분해 슬라이딩 담당
    // return true = best에 후보 확정 / outWaited = true면 이번 프레임 대기 확정
    bool TryPrimaryDirection(const VoxelGrid& grid, size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
        DirectX::XMINT3& best, bool& outWaited, float dt);

    // 2순위(cost 최소 + lastDir 정렬) 폴백
    bool PickFallbackCell(const VoxelGrid& grid, size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
        DirectX::XMINT3& best, bool& outHasActiveBlocker);

    // 이동할 복셀 예약
    void CommitMove(const VoxelGrid& grid, size_t i, const DirectX::XMINT3& curr, const DirectX::XMINT3& best);

    // 두 예약이 대각선 교차해서 지나가는지 확인
    bool IsMoveCross(size_t i, const DirectX::XMINT3& curr,
        const DirectX::XMINT3& next) const;

    // NpcManager::GetNpcStandPos와 동일 공식(중복이지만, CommitMove 전용 최소 의존성 유지 목적)
    Math::Vector3 GetNpcStandPos(const VoxelGrid& grid, const DirectX::XMINT3& cell, float halfHeight) const;

    NpcMoveData& m_Move;
    CellReservation& m_Reserve;      // 각 복셀마다 npc 예약 리스트

    std::vector<NeighborInfo> m_NeighborScratch;   // AdvanceCell 전용 임시 버퍼
};
