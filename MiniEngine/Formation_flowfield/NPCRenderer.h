#pragma once
#include "VectorMath.h"
#include "GpuBuffer.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include <vector>

class GraphicsContext;

namespace NPCRenderer
{
    struct InstanceData
    {
        float    position[3];       // 월드 위치 (타원체 중심)
        float    scaleXZ;           // XZ축 반지름 (가로 크기)
        float    scaleY;            // Y축 반지름 (세로 크기, 클수록 길쭉)
        uint32_t colorType;         // 0=기본, 1=선택됨
        uint32_t pad[2];            // 16바이트 정렬
    };
    static_assert(sizeof(InstanceData) % 16 == 0, "NPCInstanceData must be 16-byte aligned");

    static const uint32_t MAX_NPCS = 2000;

    void Initialize();
    void Shutdown();
    void UpdateInstances(const std::vector<InstanceData>& instances);
    void Render(GraphicsContext& ctx, const Math::Matrix4& viewProj);
}
