#pragma once

#include "VoxelInstanceStore.h"
#include "VoxelGrid.h"
#include "NpcManager.h"
#include <DirectXMath.h>
#include <vector>
#include <unordered_map>
#include <cstdint>

// 색상 레이어 상수
static constexpr uint32_t kColorDefault     = 0;       // default
static constexpr uint32_t kColorSelected    = 1;       // npc select
static constexpr uint32_t kColorHover       = 2;       // hovering
static constexpr uint32_t kColorPath        = 3;       // A* path

static constexpr uint32_t kColorPredEdit    = 20;      // terrain editor prediction
static constexpr uint32_t kColorAgroRing    = 21;      // Agro Range 
static constexpr uint32_t kColorDeagroRing  = 22;      // Deagro Range

// 디버그 시각화 전담: 청크 점유색 / A* 경로 / 호버·확정 목적지 / FlowField 화살표
// 게임 로직에는 관여x  비활성화 가능
//
// 경로 확정 목적지는 인덱스가 아니라 좌표로 보관
class DebugVisualizer
{
public:
    void Initialize(VoxelInstanceStore* store, const VoxelGrid* grid,
        const NpcManager* npc);

    // ---- 토글 ----
    void ToggleChunks() { m_ShowChunks = !m_ShowChunks; }
    void ToggleArrows() { m_ShowArrows = !m_ShowArrows; }
    bool IsShowingChunks() const { return m_ShowChunks; }
    bool IsShowingArrows() const { return m_ShowArrows; }

    // ---- 청크 점유 ----
    void OccupyChunks(int groupId);     // 점유 청크 등록 (그룹색)
    void ReleaseChunks(int groupId);    // 점유 청크 해제 (그룹색)
    const std::vector<int64_t>& OccupiedKeys() const { return m_OccupiedChunkKeys; }

    // ---- 선택 상태 (좌표 기반) ----
    void SetConfirmed(const DirectX::XMINT3& coord) { m_HasConfirmed = true; m_ConfirmedCoord = coord; }
    void ClearConfirmed() { m_HasConfirmed = false; }
    bool HasConfirmed() const { return m_HasConfirmed; }

    void SetPath(std::vector<DirectX::XMINT3> pathCoords) { m_PathCoords = std::move(pathCoords); }
    void ClearPath() { m_PathCoords.clear(); }

    // 호버는 매 프레임 갱신되므로 인덱스 그대로 사용
    void SetHover(int index) { m_HoverIndex = index; }
    void ClearHover() { m_HoverIndex = -1; }
    int  GetHover() const { return m_HoverIndex; }

    // 이 인덱스가 지금 확정된 목적지인지 (호버 복원 시 판정용)
    bool IsConfirmedIndex(int index) const;
    // 확정 좌표 조회 (이전 확정 셀 복원 시 필요)
    const DirectX::XMINT3& ConfirmedCoord() const { return m_ConfirmedCoord; }

    // ---- 색상 ----
    uint32_t GetLayeredColorType(int index) const;      // 호버가 지나간 자리를 원래 레이어 색으로 되돌릴 때
    void RestoreCellColor(int index);                   // 헬퍼: 실제 셀 타입 기준으로 색 복원
    // 레이어 순서대로 덮어써서 디버그 색을 갱신하고 GPU에 반영
    void CollectDebugColorChanges(std::vector<int>& changed, const std::vector<int64_t>& extraKeys);
    // 모으고 즉시 Flush. 단독 호출(F2 등)
    void RefreshDebugColors(const std::vector<int64_t>& extraKeys = {});

    // ---- 화살표 ----
    void BuildArrowInstances();
    // 필드 스왑시 색 + 화살표 갱신
    void RefreshFieldVisuals();

    // ---- 도착 시 전체 해제 ----
    void OnGroupArrived();



private:
    // 확정/경로 좌표가 현재 어느 인덱스인지 (없으면 -1)
    int ConfirmedIndex() const;
    bool IsPathCoord(const VoxelGrid::CellCoord& c) const;

    VoxelInstanceStore* m_Store = nullptr;
    const VoxelGrid* m_Grid = nullptr;
    const NpcManager* m_Npc = nullptr;

    bool m_ShowChunks = true;
    bool m_ShowArrows = true;

    // 청크 key / 청크 점유중인 그룹 ID (진입 순서순)
    // back()으로 색 표시를 기준 - 해당 그룹 도착시, erase해라.
    std::unordered_map<int64_t, std::vector<int>> m_ChunkOccupants;
    // 그룹들이 점유한 청크 키
    std::vector<int64_t>                          m_OccupiedChunkKeys;

    bool                          m_HasConfirmed = false;
    DirectX::XMINT3               m_ConfirmedCoord{};   // 우클릭으로 확정된 목적지 (다음 확정 전까지 유지)
    std::vector<DirectX::XMINT3>  m_PathCoords;         // A* 경로 (진빨강 추가 표시용)
    int                           m_HoverIndex = -1;    // 지금 마우스가 가리키는 셀 (매 프레임 갱신)
};
