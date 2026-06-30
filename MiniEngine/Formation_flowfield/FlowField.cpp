#include "GameCore.h"
#include "CameraController.h"
#include "Camera.h"				// view/proj
#include "BufferManager.h"		// rendertarget/depthbuffer
#include "CommandContext.h"		// GraphicsContext/CommandContext
#include "SystemTime.h"
#include "TextRenderer.h"		// Text
#include "GameInput.h"

// model load
#include "glTF.h"
#include "Renderer.h"
#include "Model.h"
#include "ModelLoader.h"

#include "Display.h"


using namespace GameCore;
using namespace Math;
using namespace Graphics;
using namespace std;

using Renderer::MeshSorter;

class FlowField : public GameCore::IGameApp
{
public:
    virtual void Startup(void) override;
    virtual void Cleanup(void) override;
    virtual void Update(float deltaT) override;
    virtual void RenderScene(void) override;

private:
    Camera m_Camera;
    std::unique_ptr<CameraController> m_CameraController;
    D3D12_VIEWPORT m_MainViewport;
    D3D12_RECT m_MainScissor;

    // 모델 로딩이 필요해지면 그때 ModelInstance 또는 직접 만든 메시 구조체 추가
};

CREATE_APPLICATION(FlowField)