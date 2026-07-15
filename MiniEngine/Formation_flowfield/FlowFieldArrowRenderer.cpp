#include "FlowFieldArrowRenderer.h"
#include "CommandContext.h"
#include "BufferManager.h"
#include "GraphicsCommon.h"
#include "GraphicsCore.h"

#include "CompiledShaders/FlowFieldArrowVS.h"
#include "CompiledShaders/FlowFieldArrowPS.h"

using namespace Math;
using namespace Graphics;

namespace FlowFieldArrowRenderer
{
    struct Vertex { float x, y, z; };

    // 화살표 메시 - 로컬 공간에서 +Z를 향함, 길이 1 기준.
    // 선분 3개: 몸통(0->1), 화살촉 좌(1->2), 화살촉 우(1->3).
    // LINELIST는 정점 2개씩 짝지어 선분 하나를 만들므로, 정점을 중복해서 나열함.
    static const Vertex s_ArrowVerts[6] =
    {
        { 0.0f, 0.0f, -0.5f },   // 몸통 시작 (셀 중심에서 뒤로)
        { 0.0f, 0.0f,  0.5f },   // 몸통 끝 (셀 중심에서 앞으로)

        { 0.0f, 0.0f,  0.5f },
        {-0.2f, 0.0f,  0.25f },

        { 0.0f, 0.0f,  0.5f },
        { 0.2f, 0.0f,  0.25f },
    };

    static RootSignature     s_RootSig;
    static GraphicsPSO       s_PSO(L"FlowField Arrow PSO");
    static ByteAddressBuffer s_VertexBuffer;
    static StructuredBuffer  s_InstanceBuffer;
    static uint32_t          s_InstanceCount = 0;
    static bool              s_Enabled = true;

    void Initialize()
    {
        s_RootSig.Reset(2, 0);
        s_RootSig[0].InitAsConstantBuffer(0);   // b0: ViewProj
        s_RootSig[1].InitAsBufferSRV(0);        // t0: 인스턴스
        // 입력 레이아웃(POSITION)을 쓰므로 이 플래그 필수 - 없으면 PSO 생성 시 D3D12 에러
        s_RootSig.Finalize(L"FlowField Arrow RootSig",
            D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

        D3D12_INPUT_ELEMENT_DESC layout[] =
        {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        };

        // 라인 렌더링 - 컬링 없음(선은 면이 없으므로), 깊이 테스트는 유지해서
        // 지형에 가려지는 화살표는 안 보이게 함
        D3D12_RASTERIZER_DESC raster = RasterizerDefault;
        raster.CullMode = D3D12_CULL_MODE_NONE;
        raster.DepthBias = -2;                  // 표면 위에 살짝 띄워 z-fighting 방지
        raster.SlopeScaledDepthBias = -1.0f;
        raster.AntialiasedLineEnable = TRUE;
        raster.MultisampleEnable = TRUE;

        DXGI_FORMAT colorFmt = g_SceneColorBuffer.GetFormat();
        DXGI_FORMAT depthFmt = g_SceneDepthBuffer.GetFormat();

        s_PSO.SetRootSignature(s_RootSig);
        s_PSO.SetRasterizerState(raster);
        s_PSO.SetBlendState(BlendDisable);
        s_PSO.SetDepthStencilState(DepthStateReadWrite);
        s_PSO.SetInputLayout(_countof(layout), layout);
        // TRIANGLE이 아니라 LINE - 화살표를 선분으로 그리기 위함
        s_PSO.SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE);
        s_PSO.SetRenderTargetFormats(1, &colorFmt, depthFmt);
        s_PSO.SetVertexShader(g_pFlowFieldArrowVS, sizeof(g_pFlowFieldArrowVS));
        s_PSO.SetPixelShader(g_pFlowFieldArrowPS, sizeof(g_pFlowFieldArrowPS));
        s_PSO.Finalize();

        s_VertexBuffer.Create(L"FlowField Arrow VB", 6, sizeof(Vertex), s_ArrowVerts);
        s_InstanceBuffer.Create(L"FlowField Arrow Instance Buffer", MAX_ARROWS, sizeof(InstanceData));
    }

    void Shutdown()
    {
        s_VertexBuffer.Destroy();
        s_InstanceBuffer.Destroy();
        s_InstanceCount = 0;
    }

    void UpdateInstances(const std::vector<InstanceData>& instances)
    {
        if (instances.empty()) { s_InstanceCount = 0; return; }

        s_InstanceCount = (uint32_t)instances.size();
        if (s_InstanceCount > s_InstanceBuffer.GetElementCount())
        {
            s_InstanceBuffer.Create(L"FlowField Arrow Instance Buffer",
                s_InstanceCount, sizeof(InstanceData), instances.data());
        }
        else
        {
            CommandContext::InitializeBuffer(s_InstanceBuffer, instances.data(),
                s_InstanceCount * sizeof(InstanceData));
        }
    }

    void Render(GraphicsContext& ctx, const Matrix4& viewProj)
    {
        if (!s_Enabled || s_InstanceCount == 0) return;

        ctx.SetRootSignature(s_RootSig);
        // PSO의 TOPOLOGY_TYPE_LINE과 짝을 맞춰 LINELIST 지정
        ctx.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);

        D3D12_VERTEX_BUFFER_VIEW vbv;
        vbv.BufferLocation = s_VertexBuffer.GetGpuVirtualAddress();
        vbv.SizeInBytes = sizeof(s_ArrowVerts);
        vbv.StrideInBytes = sizeof(Vertex);
        ctx.SetVertexBuffer(0, vbv);

        Matrix4 vpT = Transpose(viewProj);
        ctx.SetDynamicConstantBufferView(0, sizeof(Matrix4), &vpT);
        ctx.SetBufferSRV(1, s_InstanceBuffer);

        ctx.SetPipelineState(s_PSO);
        // 인덱스 버퍼 없음 - 정점 6개를 순서대로 읽어 선분 3개를 만듦
        ctx.DrawInstanced(6, s_InstanceCount, 0, 0);
    }

    void SetEnabled(bool enabled) { s_Enabled = enabled; }
    bool IsEnabled() { return s_Enabled; }
}
