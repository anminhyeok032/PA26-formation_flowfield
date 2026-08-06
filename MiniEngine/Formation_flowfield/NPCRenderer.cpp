#include "NpcRenderer.h"
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

namespace NpcRenderer
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
    static bool              s_IsWireframe = true;                     // F1키 누르면 중첩 렌더

    static StructuredBuffer  s_PlayerBuffer;
    static uint32_t          s_PlayerCount = 0;

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


    // 두 선분(p1~q1, p2~q2) 사이의 최단 거리 제곱을 구하고, 각 선분 위의
    // 매개변수(s,t ∈ [0,1])와 최근접점(c1,c2)을 채움
    // 레이-캡슐, 나중에 캡슐-캡슐(NPC 충돌) 판정 모두 이 함수 하나로 처리 가능.
    float ClosestPtSegmentSegment(const Math::Vector3& p1, const Math::Vector3& q1,
        const Math::Vector3& p2, const Math::Vector3& q2,
        float& s, float& t, Math::Vector3& c1, Math::Vector3& c2)
    {
        using namespace Math;
        Vector3 d1 = q1 - p1; // 첫 번째 선분 방향
        Vector3 d2 = q2 - p2; // 두 번째 선분 방향
        Vector3 r = p1 - p2;

        float a = (float)Dot(d1, d1);
        float e = (float)Dot(d2, d2);
        float f = (float)Dot(d2, r);

        const float EPS = 1e-8f;

        if (a <= EPS && e <= EPS)
        {
            s = 0.0f; t = 0.0f;
            c1 = p1; c2 = p2;
            Vector3 diff = c1 - c2;
            return (float)Dot(diff, diff);
        }

        if (a <= EPS)
        {
            s = 0.0f;
            t = Clamp(f / e, 0.0f, 1.0f);
        }
        else
        {
            float c = (float)Dot(d1, r);
            if (e <= EPS)
            {
                t = 0.0f;
                s = Clamp(-c / a, 0.0f, 1.0f);
            }
            else
            {
                float b = (float)Dot(d1, d2);
                float denom = a * e - b * b;

                s = (denom != 0.0f) ? Clamp((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
                t = (b * s + f) / e;

                if (t < 0.0f)
                {
                    t = 0.0f;
                    s = Clamp(-c / a, 0.0f, 1.0f);
                }
                else if (t > 1.0f)
                {
                    t = 1.0f;
                    s = Clamp((b - c) / a, 0.0f, 1.0f);
                }
            }
        }

        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;
        Vector3 diff = c1 - c2;
        return (float)Dot(diff, diff);
    }

    Capsule MakeCapsule(const InstanceData& inst)
    {
        using namespace Math;
        Vector3 center(inst.position[0], inst.position[1], inst.position[2]);
        float radius = inst.scaleXZ;

        // 원통 구간의 절반 길이. scaleY가 radius보다 작으면(예: 구에 가까운 형태)
        // 0으로 클램프되어 캡슐이 자연스럽게 구 하나로 축소됨.
        float halfSegment = std::max(0.0f, inst.scaleY - radius);

        Capsule cap;
        cap.p0 = center - Vector3(0.0f, halfSegment, 0.0f);
        cap.p1 = center + Vector3(0.0f, halfSegment, 0.0f);
        cap.radius = radius;
        return cap;
    }

    bool RayIntersectsCapsule(const Math::Vector3& rayOrigin, const Math::Vector3& rayDir,
        float maxDistance, const Capsule& capsule, float& outT)
    {
        Math::Vector3 rayEnd = rayOrigin + rayDir * maxDistance;

        float s, t;
        Math::Vector3 c1, c2;
        float distSq = ClosestPtSegmentSegment(rayOrigin, rayEnd, capsule.p0, capsule.p1, s, t, c1, c2);

        if (distSq > capsule.radius * capsule.radius)
            return false;

        outT = s * maxDistance; // s(0~1)를 실제 레이 거리로 환산
        return true;
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

        s_PlayerBuffer.Create(L"Player Instance Buffer", 1, sizeof(InstanceData));
    }

    void Shutdown()
    {
        s_VertexBuffer.Destroy();
        s_IndexBuffer.Destroy();
        s_InstanceBuffer.Destroy();
        s_PlayerBuffer.Destroy();
        s_InstanceCount = 0;
        s_PlayerCount = 0;
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

    void UpdatePlayerInstance(const InstanceData& inst, bool enable)
    {
        s_PlayerCount = enable ? 1u : 0u;
        if (0 == s_PlayerCount) return;

        CommandContext::InitializeBuffer(s_PlayerBuffer, &inst, sizeof(InstanceData));
    }

    // 인스턴스 집합 하나를 솔리드 + 와이어 2패스로 그린다
    static void DrawSet(GraphicsContext& ctx, StructuredBuffer& buf, uint32_t count)
    {
        if (0 == count) return;

        ctx.SetBufferSRV(1, buf);   // 슬롯 1: SV_InstanceID로 인덱싱

        uint32_t isWire = 0;
        ctx.SetConstantArray(2, 1, &isWire);
        ctx.SetPipelineState(s_SolidPSO);
        ctx.DrawIndexedInstanced(s_IndexCount, count, 0, 0, 0);

        if (true == s_IsWireframe)
        {
            isWire = 1;
            ctx.SetConstantArray(2, 1, &isWire);
            ctx.SetPipelineState(s_WireframePSO);
            ctx.DrawIndexedInstanced(s_IndexCount, count, 0, 0, 0);
        }
    }
    
    void Render(GraphicsContext& ctx, const Matrix4& viewProj)
    {
        if(0 == s_InstanceCount && 0 == s_PlayerCount) return;

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


        DrawSet(ctx, s_InstanceBuffer, s_InstanceCount);
        DrawSet(ctx, s_PlayerBuffer, s_PlayerCount);
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
