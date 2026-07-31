#include "FlowField.h"

#include "BufferManager.h"      // rendertarget/depthbuffer
#include "CommandContext.h"     // GraphicsContext/CommandContext
#include "SystemTime.h"
#include "TextRenderer.h"       // Text
#include "GameInput.h"

// model load
#include "glTF.h"
#include "Renderer.h"
#include "Model.h"
#include "ModelLoader.h"

#include "Display.h"
#include "GraphicsCore.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include "GraphicsCommon.h"



using namespace GameCore;
using namespace Math;
using namespace Graphics;
using namespace std;

namespace GameCore { extern HWND g_hWnd; }

constexpr float CAMERA_SPEED = 100.0f;
constexpr float MAX_HEIGHT = 10.0f;
constexpr float WORLD_SCALE = 1.0f;
constexpr float VOXEL_SIZE = 0.5f;


CREATE_APPLICATION(FlowField);

void FlowField::Startup(void)
{
    MemoryProbe totalProbe("Startup 전체");
    m_Camera.SetZRange(1.0f, 20000.0f); // 1-20000까지만 그려짐

    m_CameraController.reset(
        new FlyingFPSCamera(m_Camera, Vector3(kYUnitVector)));

    auto* fpsCam = static_cast<FlyingFPSCamera*>(m_CameraController.get());
    
    // 카메라 속도 조정
    fpsCam->SetMoveSpeed(CAMERA_SPEED);
    fpsCam->SetStrafeSpeed(CAMERA_SPEED);

    // 뷰포트 / 시저
    m_MainViewport.TopLeftX = 0;
    m_MainViewport.TopLeftY = 0;
    m_MainViewport.Width = (float)g_SceneColorBuffer.GetWidth();
    m_MainViewport.Height = (float)g_SceneColorBuffer.GetHeight();
    m_MainViewport.MinDepth = 0.0f;
    m_MainViewport.MaxDepth = 1.0f;

    m_MainScissor.left = 0;
    m_MainScissor.top = 0;
    m_MainScissor.right = (LONG)g_SceneColorBuffer.GetWidth();
    m_MainScissor.bottom = (LONG)g_SceneColorBuffer.GetHeight();

    // ----- 렌더러 초기화 -----
    VoxelRenderer::Initialize();
    NpcRenderer::Initialize();
    FlowFieldArrowRenderer::Initialize();
    PreviewRenderer::Initialize();

    // BMP 로드 -> 복셀 생성 -> GPU 업로드
    // Heightmap02.bmp를 실행 파일과 같은 폴더에 두거나 경로 조정
    bool loaded = m_HeightMap.LoadFromBMP("Heightmap02.bmp",
        MAX_HEIGHT,     // 최대 높이 (월드 유닛)
        WORLD_SCALE,    // MapScale
        VOXEL_SIZE);    // 복셀 1개 크기 -> 복셀 수 = pow( (맵 크기 * MAP_SCALE) / VOXEL_SIZE), 2 )
    

    if (true == loaded)
    {
        DirectX::XMFLOAT3 StartPos { 70.0f, 0.0f, 90.0f };
        DirectX::XMFLOAT3 EndPos { 100.0f, 0.0f, 91.0f };
        // 관통 터널 생성
        m_VoxelGrid.BuildFromHeightMapWithTunnel(
            m_HeightMap,
            StartPos,
            EndPos,
            15.0f,               // 터널 반지름
            1.0,              
            true, true);        // 양쪽 다 개방


        // 병목 협곡 지형 추가 (5주차 병목 테스트용)
        // 월드 좌표계: m_Size=514, cellSize=0.5 -> 유효 범위 0~257. 터널(z월드 90 부근)과 안 겹침.
        // 축 구간(z 150~190) 안에서는 통로 밖이 맵 경계까지 벽이므로 우회 불가.
        DirectX::XMFLOAT3 BottleneckStart{ 70.0f, 0.0f, 150.0f };
        DirectX::XMFLOAT3 BottleneckEnd{ 70.0f, 0.0f, 190.0f };
        m_VoxelGrid.AddNarrowingCliffs(BottleneckStart, BottleneckEnd, 12.0f, 1.0f);
    }
    else
    {
        // 임시용 1000x1000 그리드 인스턴스 생성
        std::vector<VoxelRenderer::InstanceData> instances;
        instances.reserve(1000 * 1000);
        // BMP 로드 실패 시 평면 그리드로 폴백
        for (int x = -500; x < 500; x++)
        {
            for (int z = -500; z < 500; z++)
            {
                VoxelRenderer::InstanceData inst;
                inst.position[0] = (float)x * 1.0f;
                inst.position[1] = 0.0f;
                inst.position[2] = (float)z * 1.0f;
                inst.scale = 1.0f;
                inst.colorType = ((x + z) & 1) == 0 ? 0 : 1;
                inst.pad[0] = inst.pad[1] = inst.pad[2] = 0;
                instances.push_back(inst);
            }
        }
        VoxelRenderer::UpdateInstances(instances);
    }

    float xz_position = m_HeightMap.GetWidth() * VOXEL_SIZE / 2;
    fpsCam->SetHeadingPitchAndPosition(
        XM_PI,                          // 180도 - +Z 방향(큐브 있는 곳)을 바라봄
        -90.0f,                         // 아래를 봄
        Vector3(xz_position, MAX_HEIGHT * 2.0f, xz_position)    // 카메라 위치
    );

    // m_MouseCaptured 기본값(true)과 맞춰 커서를 처음부터 숨김 상태로 시작
    ShowCursor(FALSE);
    GameInput::SetMouseExclusiveMode(true);




    // ----- 각 클래스 초기화 -----
    // 복셀 인스턴스 렌더값 초기화
    m_Store.Initialize(&m_VoxelGrid);
    m_Store.Build();

    // 동적 지형 생성기 초기화
    m_TerrainEditor.Initialize(&m_VoxelGrid, &m_Store, &m_Debug, &m_ChunkGraph);

    // 청크 링크 그래프 빌드
    {
        MemoryProbe probe("ChunkGraph::Build");
        m_ChunkGraph.Build(m_VoxelGrid);
        probe.Report();
    }
    
    const auto mem = m_ChunkGraph.GetMemoryFootprint();
    Utility::Printf("[ChunkGraph] edges=%zu splitChunks=%d\n",
        m_ChunkGraph.GetEdgeCount(),
        m_ChunkGraph.GetSplitChunkCount());
    Utility::Printf("[ChunkGraph] mem total=%.1fKB (adj=%.1fKB slack=%.1fKB)\n",
        mem.Total() / 1024.0, mem.adjacencyData / 1024.0, mem.adjacencySlack / 1024.0);

    // npc 배치 초기화
    m_Npc.Init(m_VoxelGrid, m_ChunkGraph);

    // 디버그 시각화 값 초기화
    m_Debug.Initialize(&m_Store, &m_VoxelGrid, &m_Npc);

}

void FlowField::Cleanup(void)
{
    CoreTimer::Report("Report/core_timing.txt");  // 종료 시 전체 통계 저장

    VoxelRenderer::Shutdown();
    NpcRenderer::Shutdown();
    FlowFieldArrowRenderer::Shutdown();
}





//-------------------------------------
//
//  피킹 및 피킹 후 처리 함수
//
//-------------------------------------
bool FlowField::ScreenPointToRay(int mouseX, int mouseY, Math::Vector3& outOrigin, Math::Vector3& outDir) const
{
    // 뷰포트 크기 (m_MainViewport는 기존 RenderScene에서 쓰던 값 재사용)
    float screenW = m_MainViewport.Width;
    float screenH = m_MainViewport.Height;
    if (screenW <= 0 || screenH <= 0) return false;

    // 화면 좌표 -> NDC(-1~1) 변환. Y는 위아래가 뒤집힘에 주의.
    float ndcX = (2.0f * mouseX / screenW) - 1.0f;
    float ndcY = 1.0f - (2.0f * mouseY / screenH);

    Math::Matrix4 invViewProj = Math::Invert(m_Camera.GetViewProjMatrix());

    // 
    // near/far 두 점을 NDC(정규화된 장치 좌표)에서 월드로 역변환해서 방향 벡터를 구함
    Math::Vector4 nearPoint = invViewProj * Math::Vector4(ndcX, ndcY, 1.0f, 1.0f); // ReverseZ라 near=1
    Math::Vector4 farPoint = invViewProj * Math::Vector4(ndcX, ndcY, 0.0f, 1.0f); // far=0

    Math::Vector3 nearWorld = Math::Vector3(nearPoint) / nearPoint.GetW();
    Math::Vector3 farWorld = Math::Vector3(farPoint) / farPoint.GetW();

    outOrigin = nearWorld;
    outDir = Math::Normalize(farWorld - nearWorld);
    return true;
}

FlowField::PickResult FlowField::PickVoxel() const
{
    POINT pt;
    GetCursorPos(&pt);
    ScreenToClient(GameCore::g_hWnd, &pt);

    PickResult result;
    result.hasRay = ScreenPointToRay(pt.x, pt.y, result.rayOrigin, result.rayDir);
    if (result.hasRay)
    {
        result.hit = m_VoxelGrid.RaycastVoxel(result.rayOrigin, result.rayDir, 2000.0f,
            result.cell.x, result.cell.y, result.cell.z);
    }
    return result;
}

void FlowField::HandleGroupMovePicking()
{
    // Win32 절대 마우스 좌표 획득 (GameInput은 상대 델타만 제공하므로 별도 호출 필요)
    PickResult pick = PickVoxel();

    bool leftClicked = GameInput::IsFirstPressed(GameInput::kMouse0);
    bool rightClicked = GameInput::IsFirstPressed(GameInput::kMouse1);

    std::vector<int> changedVoxelIndices;

    // 1. 좌클릭: NPC 선택
    if (leftClicked && pick.hasRay)
    {
        // 선택 / 색상  업로드
        m_Npc.TrySelectNpc(pick.rayOrigin, pick.rayDir);

        int hover = m_Debug.GetHover();
        if (hover >= 0 && !m_Debug.IsConfirmedIndex(hover))
        {
            m_Debug.RestoreCellColor(hover);
            changedVoxelIndices.push_back(hover);
        }
        m_Debug.ClearHover();
        m_HoverActive = m_Npc.HasSelection();
    }

    // ---------- 2. 호버 프리뷰: NPC가 선택된 동안 매 프레임 갱신 ----------
    if (true == m_Npc.HasSelection() && m_HoverActive && pick.hasRay)
    {
        int newHoverIndex = -1;
        if (pick.hit)
        {
            int idx = m_Store.FindIndex(pick.cell);
            if (idx > -1)
            {
                newHoverIndex = idx;
            }
        }
        int hover = m_Debug.GetHover();
        if (newHoverIndex != hover)
        {
            // 이전 호버 셀 복원 (단, 그게 이미 확정된 목적지라면 초록 유지)
            if (hover >= 0 && !m_Debug.IsConfirmedIndex(hover))
            {
                m_Debug.RestoreCellColor(hover);
                changedVoxelIndices.push_back(hover);
            }

            m_Debug.SetHover(newHoverIndex);

            // 새 호버 셀 초록 표시 (그게 이미 확정 목적지라면 어차피 초록이라 중복이어도 무해)
            if (newHoverIndex >= 0)
            {
                m_Store.SetColor(newHoverIndex, kColorHover);
                changedVoxelIndices.push_back(newHoverIndex);
            }
        }
    }
    else if (m_Debug.GetHover() >= 0)
    {
        // NPC 선택이 해제된 상태라면 호버 프리뷰 종료 (확정 셀이 아니면 복원)
        int hover = m_Debug.GetHover();
        if (false == m_Debug.IsConfirmedIndex(hover))
        {
            m_Debug.RestoreCellColor(hover);
            changedVoxelIndices.push_back(hover);
        }
        m_Debug.ClearHover();
    }

    // ---------- 3. 우클릭: 지금 호버 중인 셀을 목적지로 확정 ----------
    int hover = m_Debug.GetHover();
    if (rightClicked && m_Npc.HasSelection() && hover >= 0)
    {
        // 이전 확정 셀이 있었고, 지금 호버 셀과 다르면 원래 색으로 복원
        if (m_Debug.HasConfirmed() && !m_Debug.IsConfirmedIndex(hover))
        {
            int prevConfirmed = m_Store.FindIndex(m_Debug.ConfirmedCoord());
            if (prevConfirmed >= 0)
            {
                m_Debug.RestoreCellColor(prevConfirmed);
                changedVoxelIndices.push_back(prevConfirmed);
            }
        }

        VoxelGrid::CellCoord g = m_Store.CoordAt(hover);
        DirectX::XMINT3 goalCoord{ g.x, g.y, g.z };
        m_Debug.SetConfirmed(goalCoord);
        m_Store.SetColor(hover, kColorHover);       // 이미 초록이지만 명시적으로 재확인
        changedVoxelIndices.push_back(hover);

        m_HoverActive = false; // 목적지 확정 -> 실시간 호버 종료


        // ---- 파이프라인: 선택된 NPC 위치 -> A* -> 마스크 -> CorridorFlowField ----
        std::vector<DirectX::XMINT3> path;
        if (m_Npc.SetGroupDestination(goalCoord, &path))
        {
            // A* 경로 셀 -> 복셀 인스턴스 인덱스로 변환
            m_Debug.SetPath(path);

            std::vector<int64_t> prevKeys = m_Debug.OccupiedKeys();
            m_Debug.ReleaseChunks(0);
            m_Debug.OccupyChunks(0);
            m_Debug.CollectDebugColorChanges(changedVoxelIndices, prevKeys);
            m_Debug.BuildArrowInstances();
        }
    }
    m_Store.Flush(changedVoxelIndices);
}

void FlowField::OnEditModeChanged()
{
    if (m_EditMode == EditMode::GroupMove)
    {
        m_TerrainEditor.OnDeactivate();   // 미리보기 잔상 제거
    }
    else
    {
        // 진행 중이던 호버만 복원 (확정 목적지와 NPC 이동은 유지)
        int hover = m_Debug.GetHover();
        if (hover >= 0 && !m_Debug.IsConfirmedIndex(hover))
        {
            std::vector<int> changed;
            m_Debug.RestoreCellColor(hover);
            changed.push_back(hover);
            m_Store.Flush(changed);
        }
        m_Debug.ClearHover();
        m_HoverActive = false;
    }

}






//-------------------------------------
//
//  업데이트 및 렌더
//
//-------------------------------------
void FlowField::Update(float dt)
{
    // Tab: 인게임(카메라 조작) <-> 해제(마우스 보임, 피킹 가능) 모드 토글
    if (GameInput::IsFirstPressed(GameInput::kKey_tab))
    {
        m_MouseCaptured = !m_MouseCaptured;
        ShowCursor(m_MouseCaptured ? FALSE : TRUE);
        GameInput::SetMouseExclusiveMode(m_MouseCaptured);
    }


    // 1/2: 편집 모드 전환 (그룹 이동 / 지형 생성)
    if (GameInput::IsFirstPressed(GameInput::kKey_1) && m_EditMode != EditMode::GroupMove)
    {
        m_EditMode = EditMode::GroupMove;
        OnEditModeChanged();
    }
    if (GameInput::IsFirstPressed(GameInput::kKey_2) && m_EditMode != EditMode::TerrainBuild)
    {
        m_EditMode = EditMode::TerrainBuild;
        OnEditModeChanged();
    }


    // 인게임 모드: 기존처럼 카메라만 조작됨 (마우스 회전 + WASD)
    if (m_MouseCaptured)
    {
        m_CameraController->Update(dt);
    }
    else  // 해제 모드: 카메라는 멈추고, 클릭으로 NPC 선택/목적지 지정만 가능
    {
        if (m_EditMode == EditMode::GroupMove)
        {
            HandleGroupMovePicking();
        }
        else
        {
            PickResult pick = PickVoxel();
            bool rightClicked = GameInput::IsFirstPressed(GameInput::kMouse1);
            if (true == m_TerrainEditor.HandlePicking(pick.hit, pick.cell, rightClicked))
            {
                m_Npc.OnTerrainChanged(m_TerrainEditor.GetLastEditedCells());
                m_Debug.ReleaseChunks(0);
                m_Debug.OccupyChunks(0);
                m_Debug.BuildArrowInstances();
            }
        }
    }

    m_Npc.Update(dt); // 모드(카메라/피킹)와 무관하게 매 프레임 이동은 계속 갱신


    // 전원 도착(HasGoal true->false) 시 시각화 초기화
    const bool hasGoal = m_Npc.HasGoal();
    if (m_PrevHasGoal && !hasGoal)   m_Debug.OnGroupArrived();
    m_PrevHasGoal = hasGoal;

    // F1: 솔리드 <-> 와이어프레임 토글
    // IsFirstPressed = 키를 막 누른 순간 한 번만 true
    if (GameInput::IsFirstPressed(GameInput::kKey_f1))
    {
        VoxelRenderer::ToggleWireframe();
        NpcRenderer::ToggleWireframe();
    }

    // F2 - flowfield 색 바꾸기
    if (GameInput::IsFirstPressed(GameInput::kKey_f2))
    {
        m_Debug.ToggleChunks();
        m_Debug.RefreshDebugColors();  // 켜고 끌 때 즉시 반영
    }
    // F3 - flowfield dir 시각화
    if (GameInput::IsFirstPressed(GameInput::kKey_f3))
    {
        m_Debug.ToggleArrows();
        m_Debug.BuildArrowInstances();
    }
}

void FlowField::RenderScene(void)
{
    GraphicsContext& ctx = GraphicsContext::Begin(L"Scene Render");

    // 렌더타겟 + 깊이버퍼 전이
    ctx.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET);
    ctx.TransitionResource(g_SceneDepthBuffer, D3D12_RESOURCE_STATE_DEPTH_WRITE);

    ctx.ClearColor(g_SceneColorBuffer);
    ctx.ClearDepth(g_SceneDepthBuffer);

    ctx.SetRenderTarget(
        g_SceneColorBuffer.GetRTV(),
        g_SceneDepthBuffer.GetDSV());
    ctx.SetViewportAndScissor(m_MainViewport, m_MainScissor);

    // 큐브 드로우
    VoxelRenderer::Render(ctx, m_Camera.GetViewProjMatrix());

    // NPC 드로우
    NpcRenderer::Render(ctx, m_Camera.GetViewProjMatrix());

    // flowfield 화살표
    FlowFieldArrowRenderer::Render(ctx, m_Camera.GetViewProjMatrix());

    // 지형 생성 미리보기 (반투명, 깊이 읽기전용) — 불투명 렌더(마지막에 그릴것)
    PreviewRenderer::Render(ctx, m_Camera.GetViewProjMatrix());

    // Present 전이
    ctx.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_PRESENT);
    ctx.Finish();
}

void FlowField::ReportMemory(const char* filePath) const
{
    auto vecB = [](const auto& v) {
        return v.capacity() * sizeof(typename std::decay_t<decltype(v)>::value_type);
    };

    size_t instances = m_Store.Count();
    size_t coords = m_Store.CoordCount();

    // m_ChunkToVoxelIndices: 바깥 맵 + 안쪽 vector들 각각
    size_t chunkIdx = m_Store.ChunkIndicesCount() * 16;


    char buf[1024];
    int n = sprintf_s(buf,
        "[FlowField]\n"
        "  m_VoxelInstances       : %8.1f KB\n"
        "  m_VoxelCellCoords      : %8.1f KB\n"
        "  m_ChunkToVoxelIndices  : %8.1f KB\n"
        "  --- 합계               : %8.1f KB\n",
        instances / 1024.0, coords / 1024.0, chunkIdx / 1024.0,
        (instances + coords  + chunkIdx) / 1024.0);

    Utility::Print(buf);
    FILE* fp = nullptr; fopen_s(&fp, filePath, "a");
    if (fp) { fputs(buf, fp); fclose(fp); }

    m_VoxelGrid.ReportMemory(filePath);
    m_Npc.ReportMemory();
}