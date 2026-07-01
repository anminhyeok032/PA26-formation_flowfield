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

#include "CompiledShaders/CubeVS.h"
#include "CompiledShaders/CubePS.h"


using namespace GameCore;
using namespace Math;
using namespace Graphics;
using namespace std;

// =====================================================
// CubeRenderer
// =====================================================
namespace CubeRenderer
{
    // 일단 단색이라 위치값만
    struct Vertex { float x, y, z; };

    static const Vertex s_CubeVerts[8] =
    {
        {-0.5f,-0.5f,-0.5f}, { 0.5f,-0.5f,-0.5f},
        { 0.5f, 0.5f,-0.5f}, {-0.5f, 0.5f,-0.5f},
        {-0.5f,-0.5f, 0.5f}, { 0.5f,-0.5f, 0.5f},
        { 0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f},
    };

    static const uint16_t s_CubeIndices[36] =
    {
        0,2,1, 2,0,3,   // back
        4,5,6, 6,7,4,   // front
        0,4,7, 7,3,0,   // left
        1,6,5, 6,1,2,   // right
        3,7,6, 6,2,3,   // top
        0,1,5, 5,4,0,   // bottom
    };

    RootSignature s_RootSig;            // 슬롯
    GraphicsPSO   s_PSO(L"Cube PSO");   // 셰이더, 래스터라이저, 블랜드 등 덩어리
    ByteAddressBuffer s_VertexBuffer;   // gpu에 올릴 메쉬 데이터
    ByteAddressBuffer s_IndexBuffer;

    void Initialize()
    {
        // 루트 시그니처
        s_RootSig.Reset(2, 0);      // 루트 파라미터 2개 (mvp cbv, 색상 cbv), 정적 샘플러 0개
        s_RootSig[0].InitAsConstantBuffer(0, D3D12_SHADER_VISIBILITY_VERTEX);   // 슬롯 0 - cbv 바인딩 mvp 행렬
        s_RootSig[1].InitAsConstantBuffer(1, D3D12_SHADER_VISIBILITY_PIXEL);    // 슬롯 1 - cbv 색상 바인딩
        s_RootSig.Finalize(L"CubeRootSig",
            D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

        // 인풋 레이아웃
        D3D12_INPUT_ELEMENT_DESC layout[] =
        {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,
              D3D12_APPEND_ALIGNED_ELEMENT,
              D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        };

        // 래스터라이저 — 와이어프레임
        D3D12_RASTERIZER_DESC wireframe = {};
        wireframe.FillMode = D3D12_FILL_MODE_WIREFRAME;
        wireframe.CullMode = D3D12_CULL_MODE_NONE;
        wireframe.FrontCounterClockwise = FALSE;
        wireframe.DepthBias = D3D12_DEFAULT_DEPTH_BIAS;
        wireframe.DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP;
        wireframe.SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;
        wireframe.DepthClipEnable = TRUE;
        wireframe.MultisampleEnable = FALSE;
        wireframe.AntialiasedLineEnable = FALSE;
        wireframe.ForcedSampleCount = 0;
        wireframe.ConservativeRaster =
            D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF;

        DXGI_FORMAT colorFmt = g_SceneColorBuffer.GetFormat();
        DXGI_FORMAT depthFmt = g_SceneDepthBuffer.GetFormat();

        // PSO
        s_PSO.SetRootSignature(s_RootSig);
        s_PSO.SetRasterizerState(wireframe);
        s_PSO.SetBlendState(BlendDisable);
        s_PSO.SetDepthStencilState(DepthStateReadWrite);
        s_PSO.SetInputLayout(_countof(layout), layout);
        s_PSO.SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);
        s_PSO.SetRenderTargetFormats(1, &colorFmt, depthFmt);
        s_PSO.SetVertexShader(g_pCubeVS, sizeof(g_pCubeVS));
        s_PSO.SetPixelShader(g_pCubePS, sizeof(g_pCubePS));
        s_PSO.Finalize();

        // GPU 버퍼 복사
        s_VertexBuffer.Create(L"Cube VB",
            sizeof(s_CubeVerts), 1, s_CubeVerts);
        s_IndexBuffer.Create(L"Cube IB",
            sizeof(s_CubeIndices), 1, s_CubeIndices);
    }

    void Render(GraphicsContext& ctx, const Matrix4& viewProj)
    {
        ctx.SetRootSignature(s_RootSig);
        ctx.SetPipelineState(s_PSO);
        ctx.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        // 버텍스 버퍼
        D3D12_VERTEX_BUFFER_VIEW vbv;
        vbv.BufferLocation = s_VertexBuffer.GetGpuVirtualAddress();
        vbv.SizeInBytes = sizeof(s_CubeVerts);
        vbv.StrideInBytes = sizeof(Vertex);
        ctx.SetVertexBuffer(0, vbv);

        // 인덱스 버퍼
        D3D12_INDEX_BUFFER_VIEW ibv;
        ibv.BufferLocation = s_IndexBuffer.GetGpuVirtualAddress();
        ibv.SizeInBytes = sizeof(s_CubeIndices);
        ibv.Format = DXGI_FORMAT_R16_UINT;
        ctx.SetIndexBuffer(ibv);



        // Matrix4는 __m128 기반이라 16바이트 정렬 보장됨
        //ctx.SetDynamicConstantBufferView(0, sizeof(Matrix4), &viewProj);
   

        // alignas(16) 추가
        alignas(16) float color[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
        ctx.SetDynamicConstantBufferView(1, sizeof(color), color);

        //ctx.DrawIndexed(36);

        // 큐브 위치 목록
        Vector3 positions[] = {
            Vector3(0, 0, 0),
            Vector3(3, 0, 0),
            Vector3(-3, 0, 0),
            Vector3(0, 0, 3),
        };
        for (const auto& pos : positions)
        {
            // Model 행렬 = 이 큐브의 위치로 이동
            Matrix4 model = Matrix4(kIdentity);
            model.SetW(Vector4(pos, 1.0f));  // translation 설정

            // MVP = Model * View * Proj
            Matrix4 mvp = viewProj * model;

            ctx.SetDynamicConstantBufferView(0, sizeof(Matrix4), &mvp);
            ctx.DrawIndexed(36);  // 같은 버퍼, 다른 MVP로 반복
        }
    }
}


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
    float aspectHeightOverWidth = (float)g_SceneColorBuffer.GetHeight()
        / (float)g_SceneColorBuffer.GetWidth();

    m_Camera.SetPerspectiveMatrix(
        XM_PIDIV4,           // 수직 FOV 45도
        aspectHeightOverWidth,
        1.0f, 10000.0f);

    m_Camera.SetZRange(1.0f, 10000.0f); // 1-10000까지만 그려짐

    m_CameraController.reset(
        new FlyingFPSCamera(m_Camera, Vector3(kYUnitVector)));

    auto* fpsCam = static_cast<FlyingFPSCamera*>(m_CameraController.get());
    fpsCam->SetHeadingPitchAndPosition(
        XM_PI,               // 180도 — +Z 방향(큐브 있는 곳)을 바라봄
        -0.3f,               // 약간 아래를 봄
        Vector3(0.0f, 3.0f, -5.0f)  // 카메라 위치
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

    CubeRenderer::Initialize();
}

void FlowField::Cleanup(void) {}

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
    CubeRenderer::Render(ctx, m_Camera.GetViewProjMatrix());

    // Present 전이
    ctx.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_PRESENT);
    ctx.Finish();
}