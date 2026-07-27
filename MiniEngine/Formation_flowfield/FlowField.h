#pragma once
#include "GameCore.h"
#include "Camera.h"
#include "CameraController.h"
#include "HeightMap.h"
#include "VoxelGrid.h"
#include "VoxelRenderer.h"
#include "PreviewRenderer.h"
#include "NpcRenderer.h"
#include <d3d12.h>
#include <vector>
#include <unordered_map>
#include <memory>

// npc pathfinding
#include "AStarPathfinder.h"
#include "PathCorridor.h"
#include "CorridorFlowField.h"
#include "ChunkKey.h"
#include "FlowFieldArrowRenderer.h"
#include "NpcManager.h"



// 색상 상수 정리
static constexpr uint32_t kColorDefault     = 0;    // default
static constexpr uint32_t kColorSelected    = 1;    // npc select
static constexpr uint32_t kColorHover       = 2;    // hovering
static constexpr uint32_t kColorPath        = 3;    // A* path
static constexpr uint32_t kColorSlotEmpty   = 4;    // 빈 슬롯
static constexpr uint32_t kColorSlotClaimed = 5;    // 청구됨(이동 중)
static constexpr uint32_t kColorSlotArrived = 6;    // 도착 완료

class FlowField : public GameCore::IGameApp
{
public:
    virtual void Startup(void) override;
    virtual void Cleanup(void) override;
    virtual void Update(float deltaT) override;
    virtual void RenderScene(void) override;

private:
    Camera                              m_Camera;
    std::unique_ptr<CameraController>   m_CameraController;
    D3D12_VIEWPORT                      m_MainViewport;
    D3D12_RECT                          m_MainScissor;

    HeightMap                           m_HeightMap;
    VoxelGrid                           m_VoxelGrid;

    NpcManager                          m_Npc;

    // ------------ flowfield 시각화 관련 ----------------
    // 
    // 청크 디버그 시각화(f2)
    bool                m_DebugShowChunks = true;
    // 청크 key / 청크 내부 복셀 인스턴스 인덱스 목록
    std::unordered_map<int64_t, std::vector<int>>   m_ChunkToVoxelIndices;
    // 청크 key / 청크 점유중인 그룹 ID (진입 순서순)
    // back()으로 색 표시를 기준 - 해당 그룹 도착시, erase해라.
    std::unordered_map<int64_t, std::vector<int>>   m_ChunkOccupants;
    // 그룹들이 점유한 청크 키
    std::vector<int64_t>                            m_OccupiedChunkKeys;
    // A* 경로 복셀 인스턴스 인덱스 (진빨강 추가 표시용)
    std::vector<int>                                m_PathVoxelIndices;

    // 특정 청크 최종 colorType 우선순위 저장
    uint32_t GetBaseColorType(int instanceIndex) const;
    // 반영한 색. 호버가 지나간 자리를 원래 레이어 색으로 되돌릴 때 사용.
    uint32_t GetLayeredColorType(int instanceIndex) const;

    // 점유 청크 등록 / 해제
    void OccupyChunks(int groupId);
    void ReleaseChunks(int groupId);

    // 레이어 순서대로 덮어써서 디버그 색을 갱신하고 GPU에 반영.
    // 디버그 색을 changed에 "모으기만" 함 (Flush 안 함). 밖에서 다른 변경과 합쳐 1회 Flush할 때 사용.
    void CollectDebugColorChanges(std::vector<int>& changed, const std::vector<int64_t>& extraKeys);
    // 모으고 즉시 Flush. 단독 호출(F2 등)용.
    void RefreshDebugColors(const std::vector<int64_t>& extraKeys = {});


    //----------------- 동적지형 관련 ----------------------
    DirectX::XMINT3 m_PreviewAnchor{ INT32_MIN, INT32_MIN, INT32_MIN }; // 마지막으로 미리보기 그린셀
    bool            m_PreviewVisible = false;                           // 마지막으로 그린곳 있는지 확인용

    // 동적지형 미리보기
    std::vector<DirectX::XMINT3> ComputeBoxCells(int hx, int hy, int hz) const;
    
    // 지형 생성용 피킹(미리보기 + 우클릭시 커밋)
    void HandleTerrainBuildPicking();

    // N x N x N 박스 지형에 커밋 (SetSell -> 표면/렌더 재빌드 -> Flowfield 재구축)
    void ApplyTerrainEdit(const std::vector<DirectX::XMINT3>& cells);

    // 마우스 모드 전환시, 상태 정리용(미리보기 삭제 / 호버 삭제)
    void OnEditModeChanged();


    //----------------- flowfield dir 시각화----------------
    bool m_DebugShowArrows = true;
    void BuildArrowInstances();

    bool m_PrevHasGoal = false;   // HasGoal의 true->false 전환(전원 도착) 감지용
    void OnGroupArrived();        // 도착 시 디버그 시각화 전체 해제


    // ----------------- 목적지 복셀 선택 관련 --------------
    struct PickResult
    {
        bool hasRay = false;
        Math::Vector3 rayOrigin, rayDir;
        bool hit = false;
        DirectX::XMINT3 cell{ 0, 0, 0 };
    };
    // 현재 마우스 위치 기준 레이캐스트
    PickResult PickVoxel() const;
    std::vector<VoxelRenderer::InstanceData> m_VoxelInstances;  // 지속 보관
    std::vector<VoxelGrid::CellCoord>        m_VoxelCellCoords; // m_VoxelInstances와 1:1 대응

    // (x,y,z) 좌표 -> m_VoxelInstances 배열의 인덱스로 즉시 찾아가기 위한 색인.
    // BuildInstanceList 직후 딱 한 번만 만들고, 그 이후로는 조회만 함.
    std::unordered_map<int64_t, int> m_VoxelCoordToIndex;

    int  m_HoverCellIndex = -1;      // 지금 마우스가 가리키는 셀 (매 프레임 갱신)
    int  m_ConfirmedCellIndex = -1;  // 우클릭으로 확정된 목적지 (다음 확정 전까지 유지)
    bool m_HoverActive = false;      // NPC 선택 시 true, 우클릭으로 확정되는 순간 false (호버 정지)

    void RestoreCellColor(int instanceIndex); // 헬퍼: 실제 셀 타입 기준으로 색 복원

    // true = 인게임(카메라 조작 모드, 커서 숨김) / false = 해제(커서 보임, 피킹 가능)
    // 기본값 true로 시작 -> Startup()에서 커서 숨김과 짝을 맞춰야 함
    bool m_MouseCaptured = true;

    // ---- 편집 모드 (1: 그룹 이동 / 2: 지형 생성) ----
    enum class EditMode { GroupMove, TerrainBuild };
    EditMode m_EditMode = EditMode::GroupMove;

    // ---- 피킹(레이캐스팅) 관련 헬퍼 ----
    // 화면 좌표(마우스 클라이언트 좌표)를 월드 공간 레이(origin, dir)로 변환
    bool ScreenPointToRay(int mouseX, int mouseY, Math::Vector3& outOrigin, Math::Vector3& outDir) const;

    // 매 프레임 좌/우클릭을 검사해서 NPC 선택 / 목적지 지정 처리
    void HandleGroupMovePicking();


    // 바뀐 복셀 gpu 업로드
    void FlushVoxelInstanceChanges(std::vector<int>& changedIndices);
};