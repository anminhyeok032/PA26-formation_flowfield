#include "VoxelRenderer.h"
#include "CommandContext.h"
#include "BufferManager.h"
#include "GraphicsCommon.h"
#include "GraphicsCore.h"

// 셰이더 바이트코드 — 인스턴싱 버전 (SV_InstanceID, StructuredBuffer 포함)
#include "CompiledShaders/CubeVS.h"
#include "CompiledShaders/CubePS.h"

using namespace Math;
using namespace Graphics;

namespace VoxelRenderer
{
    // =========================================================
    // Cube 지오메트리 — 위치값만 (단색 와이어프레임)
    // =========================================================
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

    // =========================================================
    // GPU 리소스
    // =========================================================
    static RootSignature     s_RootSig;
    static GraphicsPSO       s_SolidPSO(L"Voxel Solid PSO");            // 기존 s_PSO 대체
    static GraphicsPSO       s_WireframePSO(L"Voxel Wireframe PSO");    // 와이어 전용
    static ByteAddressBuffer s_VertexBuffer;
    static ByteAddressBuffer s_IndexBuffer;
    static StructuredBuffer  s_InstanceBuffer;
    static bool              s_IsWireframe = false;               // F1키 누르면 중첩 렌더

    // CPU 사이드 인스턴스 목록
    static uint32_t s_InstanceCount = 0;


    void Initialize()
    {
        // -------------------------------------------------
        // 루트 시그니처
        // 슬롯 0: b0 CBV  — ViewProj 행렬 (버텍스 셰이더 전용)
        // 슬롯 1: t0 SRV  — 인스턴스 StructuredBuffer (버텍스 셰이더 전용)
        // -------------------------------------------------
        s_RootSig.Reset(3, 0);
        s_RootSig[0].InitAsConstantBuffer(0, D3D12_SHADER_VISIBILITY_VERTEX);
        s_RootSig[1].InitAsBufferSRV(0, D3D12_SHADER_VISIBILITY_VERTEX);
        s_RootSig[2].InitAsConstants(1, 1, D3D12_SHADER_VISIBILITY_PIXEL); // b1: 1개 상수
        s_RootSig.Finalize(L"VoxelRootSig",
            D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

        // -------------------------------------------------
        // 인풋 레이아웃 — 위치만
        // -------------------------------------------------
        D3D12_INPUT_ELEMENT_DESC layout[] =
        {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0,
              D3D12_APPEND_ALIGNED_ELEMENT,
              D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        };

        // -------------------------------------------------
        // 래스터라이저 ① 솔리드 (기본 큐브 표시)
        // -------------------------------------------------
        D3D12_RASTERIZER_DESC solid = {};
        solid.FillMode = D3D12_FILL_MODE_SOLID;   // 면 채우기
        solid.CullMode = D3D12_CULL_MODE_BACK;    // 뒷면 제거
        solid.FrontCounterClockwise = FALSE;
        solid.DepthBias = D3D12_DEFAULT_DEPTH_BIAS;      // 0 — bias 없음
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
        wireframe.FillMode = D3D12_FILL_MODE_WIREFRAME; // 선만
        wireframe.CullMode = D3D12_CULL_MODE_NONE;      // 양면 (뒷면 모서리도 표시)
        wireframe.DepthBias = -1;      // 솔리드 위로 살짝 당겨서 z-fighting 방지
        wireframe.SlopeScaledDepthBias = -0.5f;  // 경사면 추가 보정
        wireframe.DepthBiasClamp = 0.0f;

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
        s_SolidPSO.SetVertexShader(g_pCubeVS, sizeof(g_pCubeVS));
        s_SolidPSO.SetPixelShader(g_pCubePS, sizeof(g_pCubePS));
        s_SolidPSO.Finalize();  // ← 솔리드 GPU 등록

        // 와이어프레임 PSO — 솔리드를 복사한 뒤 래스터라이저만 교체
        s_WireframePSO = s_SolidPSO;                        // 모든 설정 복사
        s_WireframePSO.SetRasterizerState(wireframe);       // 래스터라이저만 덮어씀
        s_WireframePSO.Finalize();                          // ← 와이어프레임 GPU 등록

        // -------------------------------------------------
        // 큐브 메시 버퍼 — 인스턴싱에서도 메시는 하나
        // -------------------------------------------------
        s_VertexBuffer.Create(L"Voxel VB",
            8, sizeof(Vertex), s_CubeVerts);
        s_IndexBuffer.Create(L"Voxel IB",
            36, sizeof(uint16_t), s_CubeIndices);

        // -------------------------------------------------
        // 인스턴스 버퍼 — MAX_INSTANCES 크기로 미리 예약
        // 실제 데이터는 UpdateInstances()로 채움
        // -------------------------------------------------
        s_InstanceBuffer.Create(L"Voxel Instance Buffer",
            MAX_INSTANCES, sizeof(InstanceData));

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

        // 크기 초과 시 버퍼 재생성
        if (s_InstanceCount > s_InstanceBuffer.GetElementCount())
        {
            s_InstanceBuffer.Create(L"Voxel Instance Buffer",
                s_InstanceCount, sizeof(InstanceData), instances.data());
        }
        else
        {
            // 기존 버퍼 재사용 — 새 데이터 덮어쓰기
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
        vbv.SizeInBytes = sizeof(s_CubeVerts);
        vbv.StrideInBytes = sizeof(Vertex);
        ctx.SetVertexBuffer(0, vbv);

        // 인덱스 버퍼
        D3D12_INDEX_BUFFER_VIEW ibv;
        ibv.BufferLocation = s_IndexBuffer.GetGpuVirtualAddress();
        ibv.SizeInBytes = sizeof(s_CubeIndices);
        ibv.Format = DXGI_FORMAT_R16_UINT;
        ctx.SetIndexBuffer(ibv);

        // 슬롯 0: ViewProj (HLSL column-major 해석 고려해서 mul 순서로 보정됨)
        Matrix4 vpTransposed = Transpose(viewProj);
        ctx.SetDynamicConstantBufferView(0, sizeof(Matrix4), &vpTransposed);
        // 슬롯 1: 인스턴스 StructuredBuffer (셰이더에서 SV_InstanceID로 인덱싱)
        ctx.SetBufferSRV(1, s_InstanceBuffer);



        // 단 1번의 드로우콜로 s_InstanceCount개 전부 그리기 (gpu 인스턴싱)
        // 1 - 솔리드 패스
        uint32_t isWire = 0;
        ctx.SetConstantArray(2, 1, &isWire);  // 슬롯2, 1개, 값=0
        ctx.SetPipelineState(s_SolidPSO);
        ctx.DrawIndexedInstanced(36, s_InstanceCount, 0, 0, 0);

        // 2 - 와이어프레임 패스
        if (true == s_IsWireframe)
        {
            isWire = 1;
            ctx.SetConstantArray(2, 1, &isWire);  // 슬롯2, 1개, 값=1
            ctx.SetPipelineState(s_WireframePSO);
            ctx.DrawIndexedInstanced(36, s_InstanceCount, 0, 0, 0);
        }
    }

    uint32_t GetInstanceCount()
    {
        return s_InstanceCount;
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


