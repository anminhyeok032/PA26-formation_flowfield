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


using namespace GameCore;
using namespace Math;
using namespace Graphics;
using namespace std;

using Renderer::MeshSorter;

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

namespace CubeRenderer
{
    RootSignature s_RootSig;
    GraphicsPSO s_PSO(L"Cube PSO");
    StructuredBuffer s_VertexBuffer;
    ByteAddressBuffer s_IndexBuffer;

    void Initialize()
    {
        // TODO: PSO, RootSignature, 버텍스/인덱스 버퍼 초기화
    }
}

void FlowField::Startup(void)
{
    m_Camera.SetEyeAtUp(Vector3(0, 5, -10), Vector3(0, 0, 0), Vector3(0, 1, 0));
    m_Camera.SetPerspectiveMatrix(
        XM_PIDIV4, 9.0f / 16.0f,
        1.0f, 1000.0f
    );
    m_CameraController.reset(new FlyingFPSCamera(m_Camera, Vector3(kYUnitVector)));

    m_MainViewport.Width = (float)g_SceneColorBuffer.GetWidth();
    m_MainViewport.Height = (float)g_SceneColorBuffer.GetHeight();
    m_MainViewport.MinDepth = 0.0f;
    m_MainViewport.MaxDepth = 1.0f;
    m_MainViewport.TopLeftX = 0.0f;
    m_MainViewport.TopLeftY = 0.0f;

    m_MainScissor.left = 0;
    m_MainScissor.top = 0;
    m_MainScissor.right = (LONG)g_SceneColorBuffer.GetWidth();
    m_MainScissor.bottom = (LONG)g_SceneColorBuffer.GetHeight();

    CubeRenderer::Initialize();
}

void FlowField::Update(float deltaT)
{
    m_CameraController->Update(deltaT);
}

void FlowField::Cleanup(void)
{
}

void FlowField::RenderScene(void)
{
    GraphicsContext& gfxContext = GraphicsContext::Begin(L"Scene Render");

    gfxContext.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET);
    gfxContext.ClearColor(g_SceneColorBuffer);

    gfxContext.SetViewportAndScissor(m_MainViewport, m_MainScissor);
    gfxContext.SetRenderTarget(g_SceneColorBuffer.GetRTV());

    gfxContext.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_PRESENT);
    gfxContext.Finish();
}


