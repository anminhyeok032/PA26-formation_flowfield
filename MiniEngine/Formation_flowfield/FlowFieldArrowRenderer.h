#pragma once

#include "VectorMath.h"
#include "GpuBuffer.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include <vector>
#include <cstdint>

class GraphicsContext;

// FlowField의 direction 벡터를 화살표(LINELIST)로 시각화.
// 각 표면 셀에 하나씩, GPU 인스턴싱으로 한 번의 드로우콜에 전부 그림.
namespace FlowFieldArrowRenderer
{
    // 셰이더 StructuredBuffer<InstanceData>와 레이아웃 정확히 일치 필수.
    struct InstanceData
    {
        float position[3];   // offset  0 (float3: 0~12)
        float length;        // offset 12 (12~16)  -> 여기까지 정확히 16B
        float direction[3];  // offset 16 (float3: 16~28) -> 16의 배수에서 시작
        float pad;           // offset 28 (28~32)
    };
    static_assert(sizeof(InstanceData) == 32, "InstanceData must be exactly 32 bytes");

    static const uint32_t MAX_ARROWS = 65536;

    void Initialize();
    void Shutdown();
    void UpdateInstances(const std::vector<InstanceData>& instances);
    void Render(GraphicsContext& ctx, const Math::Matrix4& viewProj);

    void SetEnabled(bool enabled);
    bool IsEnabled();
}
