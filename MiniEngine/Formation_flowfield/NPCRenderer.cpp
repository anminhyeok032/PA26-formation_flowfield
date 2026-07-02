#include "NPCRenderer.h"
#include "CommandContext.h"
#include "BufferManager.h"
#include "GraphicsCommon.h"
#include "GraphicsCore.h"
#include "CompiledShaders/NpcVS.h"
#include "CompiledShaders/NpcPS.h"
#include <vector>
#include <cmath>

using namespace Math;
using namespace Graphics;

namespace NPCRenderer
{
    struct Vertex { float x, y, z; };

    static RootSignature     s_RootSig;
    static GraphicsPSO       s_PSO(L"NPC PSO");
    static ByteAddressBuffer s_VertexBuffer;
    static ByteAddressBuffer s_IndexBuffer;
    static StructuredBuffer  s_InstanceBuffer;
    static uint32_t          s_InstanceCount = 0;
    static uint32_t          s_IndexCount = 0;

    static void GenerateSphere(int segments, int rings, std::vector<Vertex>& outVerts, std::vector<uint16_t>& outIndices)
    { 
        outVerts.clear();
        outIndices.clear();

        // 버텍스 생성 — 위에서 아래로 링 단위
        for (int r = 0; r <= rings; r++)
        {
            // phi: 0(북극) ~ PI(남극)
            float phi = XM_PI * r / rings;
            float sinPhi = sinf(phi);
            float cosPhi = cosf(phi);

            for (int s = 0; s <= segments; s++)
            {
                // theta: 0 ~ 2PI (한 바퀴)
                float theta = XM_2PI * s / segments;
                float sinTheta = sinf(theta);
                float cosTheta = cosf(theta);

                Vertex v;
                v.x = sinPhi * cosTheta; // 반지름 1 기준
                v.y = cosPhi;
                v.z = sinPhi * sinTheta;
                outVerts.push_back(v);
            }
        }

        // 인덱스 생성 — 각 쿼드를 삼각형 2개로 분할
        for (int r = 0; r < rings; r++)
        {
            for (int s = 0; s < segments; s++)
            {
                uint16_t v0 = (uint16_t)(r * (segments + 1) + s);
                uint16_t v1 = (uint16_t)(r * (segments + 1) + s + 1);
                uint16_t v2 = (uint16_t)((r + 1) * (segments + 1) + s);
                uint16_t v3 = (uint16_t)((r + 1) * (segments + 1) + s + 1);

                // 삼각형 1
                outIndices.push_back(v0);
                outIndices.push_back(v2);
                outIndices.push_back(v1);

                // 삼각형 2
                outIndices.push_back(v1);
                outIndices.push_back(v2);
                outIndices.push_back(v3);
            }
        }
    }

    void Initialize()
    {
        // 루트 시그니처 — VoxelRenderer와 동일한 슬롯 구조
        s_RootSig.Reset(2, 0);
        s_RootSig[0].InitAsConstantBuffer(0, D3D12_SHADER_VISIBILITY_VERTEX);
        s_RootSig[1].InitAsBufferSRV(0, D3D12_SHADER_VISIBILITY_VERTEX);
        s_RootSig.Finalize(L"NPCRootSig",
            D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

        // 인풋 레이아웃
        D3D12_INPUT_ELEMENT_DESC layout[] =
        {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,
              D3D12_APPEND_ALIGNED_ELEMENT,
              D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        };

        // 래스터라이저 — 솔리드, 뒷면 컬링
        D3D12_RASTERIZER_DESC solid = {};
        solid.FillMode = D3D12_FILL_MODE_SOLID;
        solid.CullMode = D3D12_CULL_MODE_BACK;
        solid.FrontCounterClockwise = FALSE;
        solid.DepthBias = D3D12_DEFAULT_DEPTH_BIAS;
        solid.DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP;
        solid.SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;
        solid.DepthClipEnable = TRUE;

        DXGI_FORMAT colorFmt = g_SceneColorBuffer.GetFormat();
        DXGI_FORMAT depthFmt = g_SceneDepthBuffer.GetFormat();

        s_PSO.SetRootSignature(s_RootSig);
        s_PSO.SetRasterizerState(solid);
        s_PSO.SetBlendState(BlendDisable);
        s_PSO.SetDepthStencilState(DepthStateReadWrite);
        s_PSO.SetInputLayout(_countof(layout), layout);
        s_PSO.SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);
        s_PSO.SetRenderTargetFormats(1, &colorFmt, depthFmt);
        s_PSO.SetVertexShader(g_pNpcVS, sizeof(g_pNpcVS));
        s_PSO.SetPixelShader(g_pNpcPS, sizeof(g_pNpcPS));
        s_PSO.Finalize();

        // 구 메시 생성 — segments=8, rings=6
        // NPC 2000개 기준 폴리곤 수: 8×6×2 = 96 삼각형/개 × 2000 = 192,000
        // 낮은 폴리곤으로 성능 확보
        std::vector<Vertex>   verts;
        std::vector<uint16_t> indices;
        GenerateSphere(8, 6, verts, indices);

        s_IndexCount = (uint32_t)indices.size();

        s_VertexBuffer.Create(L"NPC VB",
            (uint32_t)verts.size(), sizeof(Vertex), verts.data());
        s_IndexBuffer.Create(L"NPC IB",
            (uint32_t)indices.size(), sizeof(uint16_t), indices.data());

        s_InstanceBuffer.Create(L"NPC Instance Buffer",
            MAX_NPCS, sizeof(InstanceData));
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
        if (s_InstanceCount > s_InstanceBuffer.GetElementCount())
        {
            s_InstanceBuffer.Create(L"NPC Instance Buffer",
                s_InstanceCount, sizeof(InstanceData), instances.data());
        }
        else
        {
            CommandContext::InitializeBuffer(
                s_InstanceBuffer,
                instances.data(),
                s_InstanceCount * sizeof(InstanceData));
        }
    }

    void Render(GraphicsContext& ctx, const Matrix4& viewProj)
    {
        if (s_InstanceCount == 0) return;

        ctx.SetRootSignature(s_RootSig);
        ctx.SetPipelineState(s_PSO);
        ctx.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        D3D12_VERTEX_BUFFER_VIEW vbv;
        vbv.BufferLocation = s_VertexBuffer.GetGpuVirtualAddress();
        vbv.SizeInBytes = sizeof(Vertex) * (s_VertexBuffer.GetBufferSize() / sizeof(Vertex));
        vbv.StrideInBytes = sizeof(Vertex);
        ctx.SetVertexBuffer(0, vbv);

        D3D12_INDEX_BUFFER_VIEW ibv;
        ibv.BufferLocation = s_IndexBuffer.GetGpuVirtualAddress();
        ibv.SizeInBytes = s_IndexBuffer.GetBufferSize();
        ibv.Format = DXGI_FORMAT_R16_UINT;
        ctx.SetIndexBuffer(ibv);

        Matrix4 vpT = Transpose(viewProj);
        ctx.SetDynamicConstantBufferView(0, sizeof(Matrix4), &vpT);
        ctx.SetBufferSRV(1, s_InstanceBuffer);

        ctx.DrawIndexedInstanced(s_IndexCount, s_InstanceCount, 0, 0, 0);
    }
}
