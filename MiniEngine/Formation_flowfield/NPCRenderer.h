#pragma once
#include "VectorMath.h"
#include "GpuBuffer.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include <vector>

class GraphicsContext;

namespace NpcRenderer
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

    static const uint32_t MAX_NPCS = 10'0000;

    // -----------------------------------------------------------------------
    // Capsule: NPC의 충돌/피킹 판정용 바운딩 볼륨.
    // 중심선(p0~p1) 주변 radius 거리 안의 공간 전체가 캡슐 내부.
    // 지금은 레이 피킹(NPC 선택)에 사용하고, 추후 NPC-NPC 충돌 회피에서도
    // 같은 ClosestPtSegmentSegment 계산을 재사용해 캡슐-캡슐 겹침 판정에 확장 가능.
    // -----------------------------------------------------------------------
    struct Capsule
    {
        Math::Vector3 p0, p1; // 캡슐 중심선의 양 끝점 (아래쪽/위쪽 반구 중심)
        float radius;
    };

    // NPC 인스턴스 데이터로부터 캡슐 생성 (scaleXZ=반지름, scaleY=Y방향 절반 길이)
    Capsule MakeCapsule(const InstanceData& inst);

    float ClosestPtSegmentSegment(const Math::Vector3& p1, const Math::Vector3& q1,
        const Math::Vector3& p2, const Math::Vector3& q2,
        float& s, float& t, Math::Vector3& c1, Math::Vector3& c2);

    // 레이(origin, dir)가 캡슐과 maxDistance 이내에서 교차하는지 검사.
    // 교차하면 true와 함께 outT(레이 파라미터, 실제 거리)를 채움.
    bool RayIntersectsCapsule(const Math::Vector3& rayOrigin, const Math::Vector3& rayDir,
        float maxDistance, const Capsule& capsule, float& outT);


    void Initialize();
    void Shutdown();
    void UpdateInstances(const std::vector<InstanceData>& instances);
    void Render(GraphicsContext& ctx, const Math::Matrix4& viewProj);
    void ToggleWireframe();
    bool IsWireframe();
}
