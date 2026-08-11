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
//#include "AStarPathfinder.h"
#include "PathCorridor.h"
#include "CorridorFlowField.h"
#include "ChunkKey.h"
#include "FlowFieldArrowRenderer.h"
#include "NpcManager.h"

#include "Player.h"
#include "PlayerOrbitCamera.h"

// helper class
#include "VoxelInstanceStore.h"
#include "DebugVisualizer.h"
#include "TerrainEditor.h"
#include "ChunkGraph.h"
#include "ScopedCpuTimer.h"
#include "MemoryProbe.h"

class FlowField : public GameCore::IGameApp
{
public:
    virtual void Startup(void) override;
    virtual void Cleanup(void) override;
    virtual void Update(float deltaT) override;
    virtual void RenderScene(void) override;

    void ReportMemory(const char* filePath = "Report/mem_flowfield.txt") const;

private:
    Camera                              m_Camera;
    std::unique_ptr<CameraController>   m_CameraController;
    D3D12_VIEWPORT                      m_MainViewport;
    D3D12_RECT                          m_MainScissor;

    
    // height map bmp file loader
    HeightMap                           m_HeightMap;

    // voxel data
    VoxelGrid                           m_VoxelGrid;
    // NPC data
    NpcManager                          m_Npc;

    // 청크 포탈
    ChunkGraph                          m_ChunkGraph;

    // 복셀 인스턴스 + 색인 + GPU 업로드의 단일 소유자
    VoxelInstanceStore  m_Store;

    // 디버그 시각화용 라이브러리
    DebugVisualizer     m_Debug;
    bool                m_HoverActive = false;      // 피킹 흐름 제어 (선택 시 시작, 확정 시 정지)
    bool                m_PrevHasGoal = false;      // HasGoal의 true->false 전환(전원 도착) 감지용

    // 동적 지형 라이브러리
    TerrainEditor       m_TerrainEditor;
    enum class EditMode { GroupMove, TerrainBuild, PlayerFollow };    // 편집 모드 (1: 그룹 이동 / 2: 지형 생성 / 3: 플레이어 모드)
    EditMode m_EditMode = EditMode::GroupMove;

    // Agro Range 시각화
    std::vector<PreviewRenderer::InstanceData> m_RingInstances;
    DirectX::XMINT3 m_RingBuiltAtCell{ INT32_MIN, INT32_MIN, INT32_MIN };
    bool m_ShowAgroRing = false;
    void BuildAgroRing(const DirectX::XMINT3& pc);

    // 추격용 멤버
    DirectX::XMINT3 m_ChaseCell{ -1, -1, -1 };
    float           m_ChaseAccum = 0.0f;

    // 플레이어
    Player          m_Player;
    std::unique_ptr<PlayerOrbitCamera> m_FollowCam;


    // ----------------- 목적지 복셀 선택(피킹) 관련 함수 -----------------
    struct PickResult
    {
        bool hasRay = false;
        Math::Vector3 rayOrigin, rayDir;
        bool hit = false;
        DirectX::XMINT3 cell{ 0, 0, 0 };
    };

    // 현재 마우스 위치 기준 레이캐스트
    PickResult PickVoxel() const;

    // 마우스 모드 전환시, 상태 정리용(미리보기 삭제 / 호버 삭제)
    void OnEditModeChanged();

    // true = 인게임(카메라 조작 모드, 커서 숨김) / false = 해제(커서 보임, 피킹 가능)
    bool m_MouseCaptured = true;

    // ---- 피킹(레이캐스팅) 관련 헬퍼 ----
    // 화면 좌표(마우스 클라이언트 좌표)를 월드 공간 레이(origin, dir)로 변환
    bool ScreenPointToRay(int mouseX, int mouseY, Math::Vector3& outOrigin, Math::Vector3& outDir) const;

    // 매 프레임 좌/우클릭을 검사해서 NPC 선택 / 목적지 지정 처리
    void HandleGroupMovePicking();

    int m_StatFrameCounter = 0;
    int m_StatDelayCounter = 0;
};