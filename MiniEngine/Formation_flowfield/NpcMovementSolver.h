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
    void AdvanceCell(const VoxelGrid& grid, const std::vector<std::unique_ptr<LeafGroup>>& leaves, 
        const CorridorFlowField* nearField,
        size_t i, float dt, bool chasing);


    // 필드 없이 anchorCell 반경 안을 배회 - Idle 전용
    void AdvanceWanderCell(const VoxelGrid& grid, size_t i, float dt, 
        const DirectX::XMINT3& anchor, int radius);

    // 필드 없이 anchorCell 방향으로 한 칸 - Lost 전용
    // 지형에 막히면 false - 호출측이 타임아웃으로 처리
    bool AdvanceReturnCell(const VoxelGrid& grid, size_t i,
        const DirectX::XMINT3& anchor);


private:
    void SnapToTargetCell(size_t i);                                // 이동
    void HoldPosition(size_t i, const DirectX::XMINT3& curr);       // 제자리 대기


    // 이웃 목록에서 (x,z)의 실제 도착 y 추출
    bool ResolveNeighborCell(int nx, int nz, DirectX::XMINT3& out)  const;

    // 해석이 끝난 이웃 셀의 이동 가능 판정 (cost 감소 + 미점유 + 비교차)
    bool AcceptCell(const VoxelGrid& grid, size_t i, const CorridorFlowField& field,
        const DirectX::XMINT3& curr, float currCost, const DirectX::XMINT3& cell) const;

    // 1순위(FlowField 방향) + 대기 판정 + 성분 분해 슬라이딩 담당
    // return true = best에 후보 확정 / outWaited = true면 이번 프레임 대기 확정
    bool TryPrimaryDirection(const VoxelGrid& grid, size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
        DirectX::XMINT3& best, bool& outWaited, float dt);

    // 2순위(cost 최소 + lastDir 정렬) 폴백
    bool PickFallbackCell(const VoxelGrid& grid, size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
        DirectX::XMINT3& best, bool& outHasActiveBlocker);

    // 이동할 복셀 예약
    void CommitMove(const VoxelGrid& grid, size_t i, const DirectX::XMINT3& curr, 
        const DirectX::XMINT3& best);

    // 두 예약이 대각선 교차해서 지나가는지 확인
    bool IsMoveCross(size_t i, const DirectX::XMINT3& curr,
        const DirectX::XMINT3& next) const;

    // NpcManager::GetNpcStandPos와 동일 공식(중복이지만, CommitMove 전용 최소 의존성 유지 목적)
    Math::Vector3 GetNpcStandPos(const VoxelGrid& grid, const DirectX::XMINT3& cell, float halfHeight) const;

    NpcMoveData& m_Move;
    CellReservation& m_Reserve;      // 각 복셀마다 npc 예약 리스트

    std::vector<NeighborInfo> m_NeighborScratch;   // AdvanceCell 전용 임시 버퍼

};
