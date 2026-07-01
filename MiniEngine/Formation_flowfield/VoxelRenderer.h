#pragma once

#include "VectorMath.h"
#include "GpuBuffer.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include <vector>
#include <cstdint>

class GraphicsContext;

namespace VoxelRenderer
{
    // 셰이더 StructuredBuffer<InstanceData>와 메모리 레이아웃이 정확히 일치해야 함
    struct InstanceData
    {
        float    position[3]; // 월드 위치 xyz
        float    scale;       // 복셀 크기
        uint32_t colorType;   // 0=흰색, 1=빨강, 2=초록, 3=파랑
        uint32_t pad[3];      // 16바이트 정렬 패딩 - 32byte 맞춰줄라고 (추후 값 추가시, 바뀔수 있다.)
    };
    static_assert(sizeof(InstanceData) % 16 == 0, "InstanceData must be 16-byte aligned");

    // GPU 버퍼 최대 예약 크기 (복셀 시각화를 위한 큐브 개수)
    static const uint32_t MAX_INSTANCES = 100000;

    // 초기화 / 종료
    void Initialize();
    void Shutdown();

    // CPU 인스턴스 목록 → GPU 업로드
    // 지형 변경 시 또는 NPC 위치 갱신 시 호출
    void UpdateInstances(const std::vector<InstanceData>& instances);

    // 매 프레임 렌더
    // viewProj: m_Camera.GetViewProjMatrix() 그대로 넘기면 됨. 내부에서 Transpose 해줌
    void Render(GraphicsContext& ctx, const Math::Matrix4& viewProj);

    // 현재 활성 인스턴스 수 조회
    uint32_t GetInstanceCount();
    void ToggleWireframe();
    bool IsWireframe();
}

