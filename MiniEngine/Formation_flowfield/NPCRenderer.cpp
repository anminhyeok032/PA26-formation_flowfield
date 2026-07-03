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
    static GraphicsPSO       s_SolidPSO(L"NPC Solid PSO");              // 솔리드 전용
    static GraphicsPSO       s_WireframePSO(L"Voxel Wireframe PSO");    // 와이어 전용
    static ByteAddressBuffer s_VertexBuffer;
    static ByteAddressBuffer s_IndexBuffer;
    static StructuredBuffer  s_InstanceBuffer;
    static uint32_t          s_InstanceCount = 0;
    static uint32_t          s_IndexCount = 0;
    static bool              s_IsWireframe = false;                     // F1키 누르면 중첩 렌더

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
        // 슬롯 0: b0 CBV  — ViewProj 행렬 (버텍스 셰이더 전용)
        // 슬롯 1: t0 SRV  — 인스턴스 StructuredBuffer (버텍스 셰이더 전용)
        s_RootSig.Reset(3, 0);
        s_RootSig[0].InitAsConstantBuffer(0, D3D12_SHADER_VISIBILITY_VERTEX);
        s_RootSig[1].InitAsBufferSRV(0, D3D12_SHADER_VISIBILITY_VERTEX);
        s_RootSig[2].InitAsConstants(1, 1, D3D12_SHADER_VISIBILITY_PIXEL); // b1: 1개 상수
        s_RootSig.Finalize(L"NPCRootSig",
            D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

        // 인풋 레이아웃
        D3D12_INPUT_ELEMENT_DESC layout[] =
        {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,
              D3D12_APPEND_ALIGNED_ELEMENT,
              D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        };

        // -------------------------------------------------
        // 래스터라이저 ① 솔리드 (기본 NPC 표시)
        // -------------------------------------------------
        D3D12_RASTERIZER_DESC solid = {};
        solid.FillMode = D3D12_FILL_MODE_SOLID;
        solid.CullMode = D3D12_CULL_MODE_BACK;
        solid.FrontCounterClockwise = FALSE;
        solid.DepthBias = D3D12_DEFAULT_DEPTH_BIAS;
        solid.DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP;
        solid.SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;
        solid.DepthClipEnable = TRUE;
        solid.MultisampleEnable = FALSE;
        solid.AntialiasedLineEnable = FALSE;
        solid.ForcedSampleCount = 0;
        solid.ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF;

        // -------------------------------------------------
        // 래스터라이저 ② 와이어프레임 (경계선 오버레이)
        // solid를 복사한 뒤 다른 부분만 덮어씀
        // -------------------------------------------------
        D3D12_RASTERIZER_DESC wireframe = solid;
        wireframe.FillMode = D3D12_FILL_MODE_WIREFRAME;     // 선만
        wireframe.CullMode = D3D12_CULL_MODE_NONE;          // 양면 (뒷면 모서리도 표시)
        wireframe.DepthBias = -4;                           // 솔리드 위로 살짝 당겨서 z-fighting 방지
        wireframe.SlopeScaledDepthBias = 1.0f;             // 경사면 추가 보정
        wireframe.DepthBiasClamp = -0.005f;


        // -------------------------------------------------
        // PSO
        // -------------------------------------------------
        DXGI_FORMAT colorFmt = g_SceneColorBuffer.GetFormat();
        DXGI_FORMAT depthFmt = g_SceneDepthBuffer.GetFormat();

        s_SolidPSO.SetRootSignature(s_RootSig);
        s_SolidPSO.SetRasterizerState(solid);
        s_SolidPSO.SetBlendState(BlendDisable);
        s_SolidPSO.SetDepthStencilState(DepthStateReadWrite);
        s_SolidPSO.SetInputLayout(_countof(layout), layout);
        s_SolidPSO.SetPrimitiveTopologyType(D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);
        s_SolidPSO.SetRenderTargetFormats(1, &colorFmt, depthFmt);
        s_SolidPSO.SetVertexShader(g_pNpcVS, sizeof(g_pNpcVS));
        s_SolidPSO.SetPixelShader(g_pNpcPS, sizeof(g_pNpcPS));
        s_SolidPSO.Finalize();

        // 와이어프레임 PSO — 솔리드를 복사한 뒤 래스터라이저만 교체
        s_WireframePSO = s_SolidPSO;                        // 모든 설정 복사
        s_WireframePSO.SetRasterizerState(wireframe);       // 래스터라이저만 덮어씀
        s_WireframePSO.Finalize();                          // 와이어프레임 GPU 등록

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
        s_IsWireframe = false;
    }

    void UpdateInstances(const std::vector<InstanceData>& instances)
    {
        if (instances.empty()) 
        { 
            s_InstanceCount = 0; 
            return; 
        }

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
        ctx.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        // 버텍스 버퍼
        D3D12_VERTEX_BUFFER_VIEW vbv;
        vbv.BufferLocation = s_VertexBuffer.GetGpuVirtualAddress();
        vbv.SizeInBytes = static_cast<UINT>(sizeof(Vertex) * (s_VertexBuffer.GetBufferSize() / sizeof(Vertex)));
        vbv.StrideInBytes = static_cast<UINT>(sizeof(Vertex));
        ctx.SetVertexBuffer(0, vbv);

        // 인덱스 버퍼
        D3D12_INDEX_BUFFER_VIEW ibv;
        ibv.BufferLocation = s_IndexBuffer.GetGpuVirtualAddress();
        ibv.SizeInBytes = static_cast<UINT>(s_IndexBuffer.GetBufferSize());
        ibv.Format = DXGI_FORMAT_R16_UINT;
        ctx.SetIndexBuffer(ibv);

        // 슬롯 0: ViewProj (HLSL column-major 해석 고려해서 mul 순서로 보정됨)
        Matrix4 vpT = Transpose(viewProj);
        ctx.SetDynamicConstantBufferView(0, sizeof(Matrix4), &vpT);
        // 슬롯 1: 인스턴스 StructuredBuffer (셰이더에서 SV_InstanceID로 인덱싱)
        ctx.SetBufferSRV(1, s_InstanceBuffer);


        // 단 1번의 드로우콜로 s_InstanceCount개 전부 그리기 (gpu 인스턴싱)
        // 1 - 솔리드 패스
        uint32_t isWire = 0;
        ctx.SetConstantArray(2, 1, &isWire);                // 슬롯2, 1개, 값=0
        ctx.SetPipelineState(s_SolidPSO);
        ctx.DrawIndexedInstanced(s_IndexCount, s_InstanceCount, 0, 0, 0);

        // 2 - 와이어프레임 패스
        if (true == s_IsWireframe)
        {
            isWire = 1;
            ctx.SetConstantArray(2, 1, &isWire);            // 슬롯2, 1개, 값=1
            ctx.SetPipelineState(s_WireframePSO);
            ctx.DrawIndexedInstanced(s_IndexCount, s_InstanceCount, 0, 0, 0);
        }
    }

    void ToggleWireframe()
    {
        s_IsWireframe = !s_IsWireframe;
    }

    bool IsWireframe()
    {
        return s_IsWireframe;
    }
}
