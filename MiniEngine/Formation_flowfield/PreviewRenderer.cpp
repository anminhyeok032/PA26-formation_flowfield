#include "PreviewRenderer.h"
#include "CommandContext.h"
#include "BufferManager.h"
#include "GraphicsCommon.h"
#include "GraphicsCore.h"

// VoxelRenderer와 동일한 셰이더를 공유 (InstanceData 레이아웃 일치)
#include "CompiledShaders/CubeVS.h"
#include "CompiledShaders/CubePS.h"

using namespace Math;
using namespace Graphics;

namespace PreviewRenderer
{
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
        0,2,1, 2,0,3,  4,5,6, 6,7,4,  0,4,7, 7,3,0,
        1,6,5, 6,1,2,  3,7,6, 6,2,3,  0,1,5, 5,4,0,
    };

    static RootSignature     s_RootSig;
    static GraphicsPSO       s_PreviewPSO(L"Preview Translucent PSO");
    static ByteAddressBuffer s_VertexBuffer;
    static ByteAddressBuffer s_IndexBuffer;
    static StructuredBuffer  s_InstanceBuffer;
    static uint32_t          s_InstanceCount = 0;

    void Initialize()
    {
        // VoxelRenderer와 동일한 루트시그니처 (b0 CBV, t0 SRV, b1 상수)
        // CubePS가 b1(IsWireframePass)을 요구하므로 슬롯2를 반드시 유지
        s_RootSig.Reset(3, 0);
        s_RootSig[0].InitAsConstantBuffer(0, D3D12_SHADER_VISIBILITY_VERTEX);
        s_RootSig[1].InitAsBufferSRV(0, D3D12_SHADER_VISIBILITY_VERTEX);
        s_RootSig[2].InitAsConstants(1, 1, D3D12_SHADER_VISIBILITY_PIXEL);
        s_RootSig.Finalize(L"PreviewRootSig",
            D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

        D3D12_INPUT_ELEMENT_DESC layout[] =
        {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,
              D3D12_APPEND_ALIGNED_ELEMENT,
              D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        };

        D3D12_RASTERIZER_DESC solid = {};
        solid.FillMode = D3D12_FILL_MODE_SOLID;
        solid.CullMode = D3D12_CULL_MODE_BACK;
        solid.FrontCounterClockwise = FALSE;
        solid.DepthClipEnable = TRUE;

        DXGI_FORMAT colorFmt = g_SceneColorBuffer.GetFormat();
        DXGI_FORMAT depthFmt = g_SceneDepthBuffer.GetFormat();

        s_PreviewPSO.SetRootSignature(s_RootSig);
        s_PreviewPSO.SetRasterizerState(solid);
        s_PreviewPSO.SetBlendState(BlendTraditional);        // SrcA, 1-SrcA (표준 알파)
        s_PreviewPSO.SetDepthStencilState(DepthStateReadOnly); // 깊이 테스트 O, 쓰기 X
        s_PreviewPSO.SetInputLayout(_countof(layout), layout);
        s_PreviewPSO.SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);
        s_PreviewPSO.SetRenderTargetFormats(1, &colorFmt, depthFmt);
        s_PreviewPSO.SetVertexShader(g_pCubeVS, sizeof(g_pCubeVS));
        s_PreviewPSO.SetPixelShader(g_pCubePS, sizeof(g_pCubePS));
        s_PreviewPSO.Finalize();

        s_VertexBuffer.Create(L"Preview VB", 8, sizeof(Vertex), s_CubeVerts);
        s_IndexBuffer.Create(L"Preview IB", 36, sizeof(uint16_t), s_CubeIndices);
        s_InstanceBuffer.Create(L"Preview Instance Buffer", MAX_INSTANCES, sizeof(InstanceData));
    }

    void Shutdown()
    {
        s_VertexBuffer.Destroy();
        s_IndexBuffer.Destroy();
        s_InstanceBuffer.Destroy();
        s_InstanceCount = 0;
    }

    void UpdateInstances(const std::vector<InstanceData>& instances)
    {
        if (instances.empty()) { s_InstanceCount = 0; return; }

        s_InstanceCount = (uint32_t)instances.size();
        // 상수 크기라 재생성 경로는 사실상 안 타지만 방어적으로 유지
        if (s_InstanceCount > s_InstanceBuffer.GetElementCount())
        {
            s_InstanceBuffer.Create(L"Preview Instance Buffer",
                s_InstanceCount, sizeof(InstanceData), instances.data());
        }
        else
        {
            CommandContext::InitializeBuffer(s_InstanceBuffer,
                instances.data(), s_InstanceCount * sizeof(InstanceData));
        }
    }

    void Render(GraphicsContext& ctx, const Matrix4& viewProj)
    {
        if (s_InstanceCount == 0) return;   // GroupMove 모드/허공 조준 시 비용 0

        ctx.SetRootSignature(s_RootSig);
        ctx.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        D3D12_VERTEX_BUFFER_VIEW vbv;
        vbv.BufferLocation = s_VertexBuffer.GetGpuVirtualAddress();
        vbv.SizeInBytes = sizeof(s_CubeVerts);
        vbv.StrideInBytes = sizeof(Vertex);
        ctx.SetVertexBuffer(0, vbv);

        D3D12_INDEX_BUFFER_VIEW ibv;
        ibv.BufferLocation = s_IndexBuffer.GetGpuVirtualAddress();
        ibv.SizeInBytes = sizeof(s_CubeIndices);
        ibv.Format = DXGI_FORMAT_R16_UINT;
        ctx.SetIndexBuffer(ibv);

        Matrix4 vpTransposed = Transpose(viewProj);
        ctx.SetDynamicConstantBufferView(0, sizeof(Matrix4), &vpTransposed);
        ctx.SetBufferSRV(1, s_InstanceBuffer);

        // CubePS의 b1(IsWireframePass)을 0으로 고정 — 미리보기는 와이어 패스 없음.
        // 이 세팅을 빼면 루트파라미터 미바인딩으로 GPU 검증 에러.
        uint32_t isWire = 0;
        ctx.SetConstantArray(2, 1, &isWire);

        ctx.SetPipelineState(s_PreviewPSO);
        ctx.DrawIndexedInstanced(36, s_InstanceCount, 0, 0, 0);
    }

    uint32_t GetInstanceCount() { return s_InstanceCount; }
}
