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

constexpr float CAMERA_SPEED = 100.0f;
constexpr float MAX_HEIGHT = 10.0f;
constexpr float WORLD_SCALE = 1.0f;
constexpr float VOXEL_SIZE = 0.5f;

class FlowField : public GameCore::IGameApp
{
public:
    virtual void Startup(void) override;
    virtual void Cleanup(void) override;
    virtual void Update(float deltaT) override;
    virtual void RenderScene(void) override;

private:
    Camera m_Camera;
    std::unique_ptr<CameraController> m_CameraController;
    D3D12_VIEWPORT m_MainViewport;
    D3D12_RECT m_MainScissor;

    HeightMap m_HeightMap;
    VoxelGrid m_VoxelGrid;

    // 모델 로딩이 필요해지면 그때 ModelInstance 또는 직접 만든 메시 구조체 추가
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

    // BMP 로드 → 복셀 생성 → GPU 업로드
    // heightmap.bmp를 실행 파일과 같은 폴더에 두거나 경로 조정
    bool loaded = m_HeightMap.LoadFromBMP("Heightmap02.bmp",
        MAX_HEIGHT,     // 최대 높이 (월드 유닛)
        WORLD_SCALE,    // MapScale
        VOXEL_SIZE);    // 복셀 1개 크기 -> 복셀 수 = pow( (맵 크기 * MAP_SCALE) / VOXEL_SIZE), 2 )

    if (true == loaded)
    {
        m_VoxelGrid.BuildFromHeightMap(m_HeightMap);

        std::vector<VoxelRenderer::InstanceData> instances;
        m_VoxelGrid.BuildInstanceList(instances);
        VoxelRenderer::UpdateInstances(instances);
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
    std::vector<NPCRenderer::InstanceData> npcInstances;
    npcInstances.reserve(100);

    // 지형 위에 격자 형태로 100개 배치
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            int gx = 10 + i * 3;
            int gz = 10 + j * 3;

            // 지형 표면 Y 위에 올리기
            float surfY = (float)m_VoxelGrid.GetSurfaceY(gx, gz)
                * m_HeightMap.GetVoxelSize();

            NPCRenderer::InstanceData inst = {};
            inst.position[0] = gx * m_HeightMap.GetVoxelSize();
            inst.position[1] = surfY; // 발 기준이 아닌 중심 기준이면 + scaleY/2 필요
            inst.position[2] = gz * m_HeightMap.GetVoxelSize();
            inst.scaleXZ = 0.3f; // 가로 반지름
            inst.scaleY = 0.8f; // 세로 반지름 (길쭉하게)
            inst.colorType = 0;
            npcInstances.push_back(inst);
        }
    }
    NPCRenderer::UpdateInstances(npcInstances);
}

void FlowField::Cleanup(void) 
{
    VoxelRenderer::Shutdown();
}

void FlowField::Update(float dt)
{
    m_CameraController->Update(dt);

    // F1: 솔리드 ↔ 와이어프레임 토글
    // IsFirstPressed = 키를 막 누른 순간 한 번만 true
    if (GameInput::IsFirstPressed(GameInput::kKey_f1))
    {
        VoxelRenderer::ToggleWireframe();
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