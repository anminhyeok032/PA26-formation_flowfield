#pragma once
#include "CameraController.h"
#include "Camera.h"
#include <cmath>

// CORE::CameraController::OrbitCamera 상속받아 사용
// 회전/줌/감도 로직은 전부 물려받고, head focus Update 만 보완해서 사용
class PlayerOrbitCamera : public OrbitCamera
{
public:
    PlayerOrbitCamera(Math::Camera& camera, float focusRadius)
        : OrbitCamera(camera, Math::BoundingSphere(Math::Vector3(Math::kZero), focusRadius))
        , m_Camera(camera)
    {
        EnableMomentum(false);   // 셀 단위 조작이라 관성이 있으면 조준이 어긋난다
    }

    // 매 프레임 플레이어 위치를 넘긴다
    void SetFocus(const Math::Vector3& worldPos) { m_Focus = worldPos; }

    void Update(float dt) override
    {
        // 부모는 원점 기준으로 궤도를 계산한다(생성자에서 중심을 0으로 줬으므로)
        OrbitCamera::Update(dt);

        // 계산된 궤도를 플레이어 위치로 평행이동.
        // m_ModelBounds에 setter가 없어서 중심을 바꿀 수 없으므로 사후 보정한다
        m_Camera.SetPosition(m_Camera.GetPosition() + m_Focus);
        m_Camera.Update();
    }

    // 이동 기준용 yaw. m_CurrentHeading이 private이라 forward에서 역산한다.
    // 카메라가 보는 방향의 XZ 성분 각도 - 이게 곧 "화면에서 위"다
    float GetHeading() const
    {
        const Math::Vector3 fwd = m_Camera.GetForwardVec();
        return std::atan2((float)fwd.GetX(), (float)fwd.GetZ());
    }

private:
    Math::Camera& m_Camera;
    Math::Vector3 m_Focus{ Math::kZero };

};
