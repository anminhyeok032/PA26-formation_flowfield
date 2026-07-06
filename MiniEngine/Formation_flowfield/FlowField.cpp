#include "GameCore.h"
#include "CameraController.h"
#include "Camera.h"				// view/proj
#include "BufferManager.h"		// rendertarget/depthbuffer
#include "CommandContext.h"		// GraphicsContext/CommandContext
#include "SystemTime.h"
#include "TextRenderer.h"		// Text
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

#include "VoxelRenderer.h"
#include "VoxelGrid.h"
#include "HeightMap.h"
#include "NPCRenderer.h"


using namespace GameCore;
using namespace Math;
using namespace Graphics;
using namespace std;

namespace GameCore { extern HWND g_hWnd; }

constexpr float CAMERA_SPEED = 100.0f;
constexpr float MAX_HEIGHT = 10.0f;
constexpr float WORLD_SCALE = 1.0f;
constexpr float VOXEL_SIZE = 0.5f;

constexpr float NPC_HEIGHT = (VOXEL_SIZE * 3.0f) / 2.0f;
constexpr float NPC_WIDTH = VOXEL_SIZE / 2.0f;

class FlowField : public GameCore::IGameApp
{
public:
    virtual void Startup(void) override;
    virtual void Cleanup(void) override;
    void FlushVoxelInstanceChanges(std::vector<int>& changedIndices);
    virtual void Update(float deltaT) override;
    virtual void RenderScene(void) override;

private:
    Camera m_Camera;
    std::unique_ptr<CameraController> m_CameraController;
    D3D12_VIEWPORT m_MainViewport;
    D3D12_RECT m_MainScissor;

    HeightMap m_HeightMap;
    VoxelGrid m_VoxelGrid;

    // ---- NPC 선택 관련 ----
    std::vector<NPCRenderer::InstanceData> m_NpcInstances;      // 지속 보관 (색상만 바꿔 재업로드하기 위해)
    float m_NpcPickRadius = 1.0f;                               // 좌클릭 피킹 시 NPC를 구(sphere)로 취급할 반지름
    int   m_SelectedNpcIndex = -1;                              // 현재 선택된 NPC 인덱스, 없으면 -1

    // ---- 목적지 복셀 선택 관련 ----
    std::vector<VoxelRenderer::InstanceData> m_VoxelInstances;  // 지속 보관
    std::vector<VoxelGrid::CellCoord>        m_VoxelCellCoords; // m_VoxelInstances와 1:1 대응

    int m_HoverCellIndex = -1;      // 지금 마우스가 가리키는 셀 (매 프레임 갱신)
    int m_ConfirmedCellIndex = -1;  // 우클릭으로 확정된 목적지 (다음 확정 전까지 유지)
    bool m_HoverActive = false;     // NPC 선택 시 true, 우클릭으로 확정되는 순간 false (호버 정지)

    void RestoreCellColor(int instanceIndex); // 헬퍼: 실제 셀 타입 기준으로 색 복원


    // true = 인게임(카메라 조작 모드, 커서 숨김) / false = 해제(커서 보임, 피킹 가능)
    // 기본값 true로 시작 -> Startup()에서 커서 숨김과 짝을 맞춰야 함
    bool m_MouseCaptured = true;

    // ---- 피킹(레이캐스팅) 관련 헬퍼 ----
    // 화면 좌표(마우스 클라이언트 좌표)를 월드 공간 레이(origin, dir)로 변환
    bool ScreenPointToRay(int mouseX, int mouseY, Math::Vector3& outOrigin, Math::Vector3& outDir) const;
    // 매 프레임 좌/우클릭을 검사해서 NPC 선택 / 목적지 지정 처리
    void HandlePicking();
};

CREATE_APPLICATION(FlowField);

void FlowField::Startup(void)
{
    m_Camera.SetZRange(1.0f, 20000.0f); // 1-10000까지만 그려짐

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

    VoxelRenderer::Initialize();
    NPCRenderer::Initialize();

    // BMP 로드 → 복셀 생성 → GPU 업로드
    // heightmap.bmp를 실행 파일과 같은 폴더에 두거나 경로 조정
    bool loaded = m_HeightMap.LoadFromBMP("Heightmap02.bmp",
        MAX_HEIGHT,     // 최대 높이 (월드 유닛)
        WORLD_SCALE,    // MapScale
        VOXEL_SIZE);    // 복셀 1개 크기 -> 복셀 수 = pow( (맵 크기 * MAP_SCALE) / VOXEL_SIZE), 2 )


    if (true == loaded)
    {
        // 관통 터널 생성
        m_VoxelGrid.BuildFromHeightMapWithTunnel(
            m_HeightMap,
            70.0f, 90.0f,       // 입구 좌표
            170.0f, 104.0f,     // 출구 좌표
            15.0f,              // 터널 내부 높이(헤드룸)
            30.0f,              // 터널 반경
            2.0f,               // 아치 두께
            true, true);        // 양쪽 다 개방

        
        m_VoxelGrid.BuildInstanceList(m_VoxelInstances, &m_VoxelCellCoords);
        VoxelRenderer::UpdateInstances(m_VoxelInstances);
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
        XM_PI,                          // 180도 — +Z 방향(큐브 있는 곳)을 바라봄
        -90.0f,                         // 아래를 봄
        Vector3(xz_position, MAX_HEIGHT * 2.0f, xz_position)    // 카메라 위치
    );


    // 테스트용 NPC 배치
    m_NpcInstances.clear();
    m_NpcInstances.reserve(1000);

    // 지형 위에 격자 형태로 100개 배치
    for (int i = 0; i < 100; i++)
    {
        for (int j = 0; j < 100; j++)
        {
            int gx = 100 + i * 3;
            int gz = 100 + j * 3;

            // 지형 표면 Y 위에 올리기
            float surfY = (float)m_VoxelGrid.GetSurfaceY(gx, gz) * m_HeightMap.GetVoxelSize();

            NPCRenderer::InstanceData inst = {};
            inst.scaleXZ = NPC_WIDTH;
            inst.scaleY  = NPC_HEIGHT;
            inst.position[0] = gx * m_HeightMap.GetVoxelSize();
            inst.position[1] = surfY + (m_HeightMap.GetVoxelSize() / 2.0f) + inst.scaleY + 0.1f;
            inst.position[2] = gz * m_HeightMap.GetVoxelSize();
            inst.colorType = 0;
            m_NpcInstances.push_back(inst);
        }
    }
    NPCRenderer::UpdateInstances(m_NpcInstances);

    // m_MouseCaptured 기본값(true)과 맞춰 커서를 처음부터 숨김 상태로 시작
    ShowCursor(FALSE);
    GameInput::SetMouseExclusiveMode(true);
}

void FlowField::Cleanup(void)
{
    VoxelRenderer::Shutdown();
    NPCRenderer::Shutdown();
}


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


// TODO : 현재는 Group이 아닌 npc 하나만 적용되는 상태.
//        Group 구현시, 실시간으로 해당 넓이 값 어떻게 적용시킬지 설계 필요함
void FlowField::HandlePicking()
{
    // Win32 절대 마우스 좌표 획득 (GameInput은 상대 델타만 제공하므로 별도 호출 필요)
    POINT pt;
    GetCursorPos(&pt);
    ScreenToClient(GameCore::g_hWnd, &pt);

    Math::Vector3 rayOrigin, rayDir;
    bool hasRay = ScreenPointToRay(pt.x, pt.y, rayOrigin, rayDir);

    bool leftClicked = GameInput::IsFirstPressed(GameInput::kMouse0);
    bool rightClicked = GameInput::IsFirstPressed(GameInput::kMouse1);

    // 1. 좌클릭: NPC 선택
    if (leftClicked && hasRay)
    {
        int hitIndex = -1;
        float closestDist = FLT_MAX;
        const float PICK_MAX_DISTANCE = 500.0f;

        for (size_t i = 0; i < m_NpcInstances.size(); i++)
        {
            NPCRenderer::Capsule cap = NPCRenderer::MakeCapsule(m_NpcInstances[i]);
            float t;
            if (NPCRenderer::RayIntersectsCapsule(rayOrigin, rayDir, PICK_MAX_DISTANCE, cap, t))
            {
                if (t >= 0.0f && t < closestDist) 
                { 
                    closestDist = t; 
                    hitIndex = static_cast<int>(i); 
                }
            }
        }

        if (m_SelectedNpcIndex >= 0 && m_SelectedNpcIndex < (int)m_NpcInstances.size())
            m_NpcInstances[m_SelectedNpcIndex].colorType = 0;

        m_SelectedNpcIndex = (hitIndex == m_SelectedNpcIndex) ? -1 : hitIndex;
        if (m_SelectedNpcIndex >= 0)
            m_NpcInstances[m_SelectedNpcIndex].colorType = 2;

        NPCRenderer::UpdateInstances(m_NpcInstances);

        // NPC 선택이 바뀌었으니, 진행 중이던 호버 프리뷰는 정리
        // (확정 목적지(m_ConfirmedCellIndex)는 정책상 그대로 유지 — 건드리지 않음)
        if (m_HoverCellIndex >= 0 && m_HoverCellIndex != m_ConfirmedCellIndex)
            RestoreCellColor(m_HoverCellIndex);
        m_HoverCellIndex = -1;

        // 새로 NPC를 선택했으면 호버 재개, 선택 해제면 호버도 꺼둠
        m_HoverActive = (m_SelectedNpcIndex >= 0);
    }

    // ---------- 2. 호버 프리뷰: NPC가 선택된 동안 매 프레임 갱신 ----------
    std::vector<int> changedVoxelIndices;

    if (m_SelectedNpcIndex >= 0 && m_HoverActive && hasRay)
    {
        int hx, hy, hz;
        bool hit = m_VoxelGrid.RaycastVoxel(rayOrigin, rayDir, 500.0f, hx, hy, hz);

        int newHoverIndex = -1;
        if (hit)
        {
            for (size_t i = 0; i < m_VoxelCellCoords.size(); i++)
            {
                if (m_VoxelCellCoords[i].x == hx &&
                    m_VoxelCellCoords[i].y == hy &&
                    m_VoxelCellCoords[i].z == hz)
                {
                    newHoverIndex = (int)i;
                    break;
                }
            }
        }

        if (newHoverIndex != m_HoverCellIndex)
        {
            // 이전 호버 셀 복원 (단, 그게 이미 확정된 목적지라면 초록 유지)
            if (m_HoverCellIndex >= 0 && m_HoverCellIndex != m_ConfirmedCellIndex)
            {
                RestoreCellColor(m_HoverCellIndex);
                changedVoxelIndices.push_back(m_HoverCellIndex);
            }

            m_HoverCellIndex = newHoverIndex;

            // 새 호버 셀 초록 표시 (그게 이미 확정 목적지라면 어차피 초록이라 중복이어도 무해)
            if (m_HoverCellIndex >= 0)
            {
                m_VoxelInstances[m_HoverCellIndex].colorType = 2;
                changedVoxelIndices.push_back(m_HoverCellIndex);
            }
        }
    }
    else if (m_HoverCellIndex >= 0)
    {
        // NPC 선택이 해제된 상태라면 호버 프리뷰 종료 (확정 셀이 아니면 복원)
        if (m_HoverCellIndex != m_ConfirmedCellIndex)
        {
            RestoreCellColor(m_HoverCellIndex);
            changedVoxelIndices.push_back(m_HoverCellIndex);
        }
        m_HoverCellIndex = -1;
    }

    // ---------- 3. 우클릭: 지금 호버 중인 셀을 목적지로 확정 ----------
    if (rightClicked && m_SelectedNpcIndex >= 0 && m_HoverCellIndex >= 0)
    {
        // 이전 확정 셀이 있었고, 지금 호버 셀과 다르면 원래 색으로 복원
        if (m_ConfirmedCellIndex >= 0 &&
            m_ConfirmedCellIndex != m_HoverCellIndex)
        {
            RestoreCellColor(m_ConfirmedCellIndex);
            changedVoxelIndices.push_back(m_ConfirmedCellIndex);
        }

        m_ConfirmedCellIndex = m_HoverCellIndex;
        m_VoxelInstances[m_ConfirmedCellIndex].colorType = 2; // 이미 초록이지만 명시적으로 재확인
        changedVoxelIndices.push_back(m_HoverCellIndex);

        m_HoverActive = false; // 목적지 확정 -> 실시간 호버 종료


        // TODO: 여기서 FlowFieldSolver.Solve(...) 호출 예정 (길찾기는 이후 별도 구현)
    }

    FlushVoxelInstanceChanges(changedVoxelIndices);
}

void FlowField::RestoreCellColor(int instanceIndex)
{
    if (instanceIndex < 0 || instanceIndex >= (int)m_VoxelCellCoords.size()) return;

    auto& c = m_VoxelCellCoords[instanceIndex];
    m_VoxelInstances[instanceIndex].colorType = (m_VoxelGrid.GetCell(c.x, c.y, c.z) == VoxelGrid::CellType::Walkable) ? 0 : 1;
}

// 변경된 인스턴스 인덱스들을 정렬 후 "연속 구간"으로 묶어서,
// 구간 하나당 GPU 호출 1번(UpdateInstanceRange)만 발생하도록 최소화.
// GPU 갱신 호출은 매번 CPU-GPU 동기화(Finish(true))를 동반하므로,
// 변경된 셀 개수만큼 호출을 반복하면 오히려 손해 -> 반드시 묶어서 호출해야 함.
void FlowField::FlushVoxelInstanceChanges(std::vector<int>& changedIndices)
{
    if (changedIndices.empty()) return;

    std::sort(changedIndices.begin(), changedIndices.end());
    // 중복 인덱스 제거 (한 프레임에 같은 셀이 두 번 바뀐 경우 대비)
    changedIndices.erase(std::unique(changedIndices.begin(), changedIndices.end()), changedIndices.end());

    size_t i = 0;
    while (i < changedIndices.size())
    {
        size_t runStart = i;
        // 인덱스가 연속(1씩 증가)인 동안 구간을 넓힘
        while (i + 1 < changedIndices.size() && changedIndices[i + 1] == changedIndices[i] + 1)
        {
            i++;
        }

        int startIndex = changedIndices[runStart];
        int count = changedIndices[i] - startIndex + 1;

        // m_VoxelInstances에서 해당 구간이 실제로도 메모리상 연속이므로
        // 포인터 하나로 count개를 통째로 넘길 수 있음
        VoxelRenderer::UpdateInstanceRange((uint32_t)startIndex, (uint32_t)count, &m_VoxelInstances[startIndex]);
        i++;
    }
}


void FlowField::Update(float dt)
{
    // Tab: 인게임(카메라 조작) <-> 해제(마우스 보임, 피킹 가능) 모드 토글
    if (GameInput::IsFirstPressed(GameInput::kKey_tab))
    {
        m_MouseCaptured = !m_MouseCaptured;
        ShowCursor(m_MouseCaptured ? FALSE : TRUE);
        GameInput::SetMouseExclusiveMode(m_MouseCaptured);
    }

    if (m_MouseCaptured)
    {
        // 인게임 모드: 기존처럼 카메라만 조작됨 (마우스 회전 + WASD)
        m_CameraController->Update(dt);
    }
    else
    {
        // 해제 모드: 카메라는 멈추고, 클릭으로 NPC 선택/목적지 지정만 가능
        HandlePicking();
    }

    // F1: 솔리드 ↔ 와이어프레임 토글
    // IsFirstPressed = 키를 막 누른 순간 한 번만 true
    if (GameInput::IsFirstPressed(GameInput::kKey_f1))
    {
        VoxelRenderer::ToggleWireframe();
        NPCRenderer::ToggleWireframe();
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
    NPCRenderer::Render(ctx, m_Camera.GetViewProjMatrix());

    // Present 전이
    ctx.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_PRESENT);
    ctx.Finish();
}