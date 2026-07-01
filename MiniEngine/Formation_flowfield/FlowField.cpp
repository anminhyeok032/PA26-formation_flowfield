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


using namespace GameCore;
using namespace Math;
using namespace Graphics;
using namespace std;



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

    // 모델 로딩이 필요해지면 그때 ModelInstance 또는 직접 만든 메시 구조체 추가
};

CREATE_APPLICATION(FlowField);

void FlowField::Startup(void)
{
    m_Camera.SetZRange(1.0f, 10000.0f); // 1-10000까지만 그려짐

    m_CameraController.reset(
        new FlyingFPSCamera(m_Camera, Vector3(kYUnitVector)));

    auto* fpsCam = static_cast<FlyingFPSCamera*>(m_CameraController.get());
    fpsCam->SetHeadingPitchAndPosition(
        XM_PI,                         // 180도 — +Z 방향(큐브 있는 곳)을 바라봄
        -1.0f,                         // 약간 아래를 봄
        Vector3(0.0f, 100.0f, 0.0f)    // 카메라 위치
    );
    // 카메라 속도 조정
    fpsCam->SlowMovement(true);

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

    // 테스트용 1000x1000 그리드 인스턴스 생성
    std::vector<VoxelRenderer::InstanceData> instances;
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

void FlowField::Cleanup(void) 
{
    VoxelRenderer::Shutdown();
}

void FlowField::Update(float dt)
{
    m_CameraController->Update(dt);
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

    // Present 전이
    ctx.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_PRESENT);
    ctx.Finish();
}