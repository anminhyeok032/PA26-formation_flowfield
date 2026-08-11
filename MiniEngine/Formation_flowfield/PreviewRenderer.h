#pragma once
#include "VectorMath.h"
#include "GpuBuffer.h"	
#include "RootSignature.h"
#include "PipelineState.h"
#include <vector>
#include <cstdint>

class GraphicsContext;

// 지형 미리보기 렌더러
namespace PreviewRenderer
{
	// VoxelRenderer::InstanceData 와 완전히 동일한 레이아웃 (셰이더 공유)
	struct InstanceData
	{
		float position[3];
		float scale;
		uint32_t colorType;
		uint32_t pad[3];
	};
	static_assert(sizeof(InstanceData) % 16 == 0, "InstanceData must be 16-byte aligned");

	// 박스 상수 크기라 소량이면 충분 (3x3x3=27, 여유분 포함)
	static const uint32_t MAX_INSTANCES = 64;

	// 편집 미리보기(s_InstanceBuffer)와 별개로 유지되는 오버레이 세트
	static const uint32_t MAX_OVERLAY_INSTANCES = 2048;

	void UpdateOverlayInstances(const std::vector<InstanceData>& instances);


	void Initialize();
	void Shutdown();
	void UpdateInstances(const std::vector<InstanceData>& instances);
	void Render(GraphicsContext& ctx, const Math::Matrix4& viewProj);
	uint32_t GetInstanceCount();
}
