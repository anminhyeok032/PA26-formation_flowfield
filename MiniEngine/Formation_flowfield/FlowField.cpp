#include "FlowField.h"

#include "BufferManager.h"      // rendertarget/depthbuffer
#include "CommandContext.h"     // GraphicsContext/CommandContext
#include "SystemTime.h"
#include "TextRenderer.h"       // Text
#include "GameInput.h"

// model load
#include "glTF.h"
#include "Renderer.h"
#include "Model.h"
#include "ModelLoader.h"

#include "Display.h"
#include "GraphicsCore.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include "GraphicsCommon.h"



using namespace GameCore;
using namespace Math;
using namespace Graphics;
using namespace std;

namespace GameCore { extern HWND g_hWnd; }

constexpr float CAMERA_SPEED = 100.0f;
constexpr float MAX_HEIGHT = 10.0f;
constexpr float WORLD_SCALE = 1.0f;
constexpr float VOXEL_SIZE = 0.5f;

//constexpr float NPC_HEIGHT = (VOXEL_SIZE * 3.0f) / 2.0f;
//constexpr float NPC_WIDTH = VOXEL_SIZE / 10.0f;


CREATE_APPLICATION(FlowField);

void FlowField::Startup(void)
{
    m_Camera.SetZRange(1.0f, 20000.0f); // 1-20000까지만 그려짐

    m_CameraController.reset(
        new FlyingFPSCamera(m_Camera, Vector3(kYUnitVector)));

    auto* fpsCam = static_cast<FlyingFPSCamera*>(m_CameraController.get());
    
    // 카메라 속도 조정
    fpsCam->SetMoveSpeed(CAMERA_SPEED);
    fpsCam->SetStrafeSpeed(CAMERA_SPEED);

    // 뷰포트 / 시저
    m_MainViewport.TopLeftX = 0;
    m_MainViewport.TopLeftY = 0;
    m_MainViewport.Width = (float)g_SceneColorBuffer.GetWidth();
    m_MainViewport.Height = (float)g_SceneColorBuffer.GetHeight();
    m_MainViewport.MinDepth = 0.0f;
    m_MainViewport.MaxDepth = 1.0f;

    m_MainScissor.left = 0;
    m_MainScissor.top = 0;
    m_MainScissor.right = (LONG)g_SceneColorBuffer.GetWidth();
    m_MainScissor.bottom = (LONG)g_SceneColorBuffer.GetHeight();

    VoxelRenderer::Initialize();
    NpcRenderer::Initialize();
    FlowFieldArrowRenderer::Initialize();


    // BMP 로드 → 복셀 생성 → GPU 업로드
    // heightmap.bmp를 실행 파일과 같은 폴더에 두거나 경로 조정
    bool loaded = m_HeightMap.LoadFromBMP("Heightmap02.bmp",
        MAX_HEIGHT,     // 최대 높이 (월드 유닛)
        WORLD_SCALE,    // MapScale
        VOXEL_SIZE);    // 복셀 1개 크기 -> 복셀 수 = pow( (맵 크기 * MAP_SCALE) / VOXEL_SIZE), 2 )


    if (true == loaded)
    {
        DirectX::XMFLOAT3 StartPos { 70.0f, 0.0f, 90.0f };
        DirectX::XMFLOAT3 EndPos { 100.0f, 0.0f, 91.0f };
        // 관통 터널 생성
        m_VoxelGrid.BuildFromHeightMapWithTunnel(
            m_HeightMap,
            StartPos,
            EndPos,
            5.0f,               // 터널 반지름
            1.0,              
            true, true);        // 양쪽 다 개방

        
        m_VoxelGrid.BuildInstanceList(m_VoxelInstances, &m_VoxelCellCoords);
        VoxelRenderer::UpdateInstances(m_VoxelInstances);

        // 색인 구축: 여기서만 전체를 한 번 순회함 (이후로는 절대 이렇게 다시 순회 안 함)
        m_VoxelCoordToIndex.clear();
        m_VoxelCoordToIndex.reserve(m_VoxelCellCoords.size());

        // 청크 시각화용 자료 초기화
        m_ChunkToVoxelIndices.clear();
        int chunkSize = CorridorFlowField::CHUNK_SIZE;


        for (size_t i = 0; i < m_VoxelCellCoords.size(); i++)
        {
            auto& c = m_VoxelCellCoords[i];
            m_VoxelCoordToIndex[MakeCellKey(c.x, c.y, c.z)] = (int)i;

            // TODO : FlowField를 2d->3d로 바꿀시, 해당 부분도 수정
            int cx = c.x / chunkSize;
            int cz = c.z / chunkSize;
            m_ChunkToVoxelIndices[MakeChunkKey(cx, 0, cz)].push_back((int)i);
        }
    }
    else
    {
        // 임시용 1000x1000 그리드 인스턴스 생성
        std::vector<VoxelRenderer::InstanceData> instances;
        instances.reserve(1000 * 1000);
        // BMP 로드 실패 시 평면 그리드로 폴백
        for (int x = -500; x < 500; x++)
        {
            for (int z = -500; z < 500; z++)
            {
                VoxelRenderer::InstanceData inst;
                inst.position[0] = (float)x * 1.0f;
                inst.position[1] = 0.0f;
                inst.position[2] = (float)z * 1.0f;
                inst.scale = 1.0f;
                inst.colorType = ((x + z) & 1) == 0 ? 0 : 1;
                inst.pad[0] = inst.pad[1] = inst.pad[2] = 0;
                instances.push_back(inst);
            }
        }
        VoxelRenderer::UpdateInstances(instances);
    }

    float xz_position = m_HeightMap.GetWidth() * VOXEL_SIZE / 2;
    fpsCam->SetHeadingPitchAndPosition(
        XM_PI,                          // 180도 - +Z 방향(큐브 있는 곳)을 바라봄
        -90.0f,                         // 아래를 봄
        Vector3(xz_position, MAX_HEIGHT * 2.0f, xz_position)    // 카메라 위치
    );

    // npc 배치
    m_Npc.Init(m_VoxelGrid);

    // m_MouseCaptured 기본값(true)과 맞춰 커서를 처음부터 숨김 상태로 시작
    ShowCursor(FALSE);
    GameInput::SetMouseExclusiveMode(true);
}

void FlowField::Cleanup(void)
{
    VoxelRenderer::Shutdown();
    NpcRenderer::Shutdown();
    FlowFieldArrowRenderer::Shutdown();
}


bool FlowField::ScreenPointToRay(int mouseX, int mouseY, Math::Vector3& outOrigin, Math::Vector3& outDir) const
{
    // 뷰포트 크기 (m_MainViewport는 기존 RenderScene에서 쓰던 값 재사용)
    float screenW = m_MainViewport.Width;
    float screenH = m_MainViewport.Height;
    if (screenW <= 0 || screenH <= 0) return false;

    // 화면 좌표 -> NDC(-1~1) 변환. Y는 위아래가 뒤집힘에 주의.
    float ndcX = (2.0f * mouseX / screenW) - 1.0f;
    float ndcY = 1.0f - (2.0f * mouseY / screenH);

    Math::Matrix4 invViewProj = Math::Invert(m_Camera.GetViewProjMatrix());

    // 
    // near/far 두 점을 NDC(정규화된 장치 좌표)에서 월드로 역변환해서 방향 벡터를 구함
    Math::Vector4 nearPoint = invViewProj * Math::Vector4(ndcX, ndcY, 1.0f, 1.0f); // ReverseZ라 near=1
    Math::Vector4 farPoint = invViewProj * Math::Vector4(ndcX, ndcY, 0.0f, 1.0f); // far=0

    Math::Vector3 nearWorld = Math::Vector3(nearPoint) / nearPoint.GetW();
    Math::Vector3 farWorld = Math::Vector3(farPoint) / farPoint.GetW();

    outOrigin = nearWorld;
    outDir = Math::Normalize(farWorld - nearWorld);
    return true;
}



void FlowField::HandlePicking()
{
    // Win32 절대 마우스 좌표 획득 (GameInput은 상대 델타만 제공하므로 별도 호출 필요)
    POINT pt;
    GetCursorPos(&pt);
    ScreenToClient(GameCore::g_hWnd, &pt);

    Math::Vector3 rayOrigin, rayDir;
    bool hasRay = ScreenPointToRay(pt.x, pt.y, rayOrigin, rayDir);

    bool leftClicked = GameInput::IsFirstPressed(GameInput::kMouse0);
    bool rightClicked = GameInput::IsFirstPressed(GameInput::kMouse1);

    std::vector<int> changedVoxelIndices;

    // 1. 좌클릭: NPC 선택
    if (leftClicked && hasRay)
    {
        // 선택, 색상, 업로드
        m_Npc.TrySelectNpc(rayOrigin, rayDir);

        if (m_HoverCellIndex >= 0 && m_HoverCellIndex != m_ConfirmedCellIndex)
        {
            RestoreCellColor(m_HoverCellIndex);
            changedVoxelIndices.push_back(m_HoverCellIndex);
        }
        m_HoverCellIndex = -1;
        m_HoverActive = m_Npc.HasSelection();
    }

    // ---------- 2. 호버 프리뷰: NPC가 선택된 동안 매 프레임 갱신 ----------
    if (true == m_Npc.HasSelection() && m_HoverActive && hasRay)
    {
        int hx, hy, hz;
        bool hit = m_VoxelGrid.RaycastVoxel(rayOrigin, rayDir, 2000.0f, hx, hy, hz);

        int newHoverIndex = -1;
        if (hit)
        {
            auto it = m_VoxelCoordToIndex.find(MakeCellKey(hx, hy, hz));
            if (it != m_VoxelCoordToIndex.end())
                newHoverIndex = it->second;
        }

        if (newHoverIndex != m_HoverCellIndex)
        {
            // 이전 호버 셀 복원 (단, 그게 이미 확정된 목적지라면 초록 유지)
            if (m_HoverCellIndex >= 0 && m_HoverCellIndex != m_ConfirmedCellIndex)
            {
                RestoreCellColor(m_HoverCellIndex);
                changedVoxelIndices.push_back(m_HoverCellIndex);
            }

            m_HoverCellIndex = newHoverIndex;

            // 새 호버 셀 초록 표시 (그게 이미 확정 목적지라면 어차피 초록이라 중복이어도 무해)
            if (m_HoverCellIndex >= 0)
            {
                m_VoxelInstances[m_HoverCellIndex].colorType = kColorHover;
                changedVoxelIndices.push_back(m_HoverCellIndex);
            }
        }
    }
    else if (m_HoverCellIndex >= 0)
    {
        // NPC 선택이 해제된 상태라면 호버 프리뷰 종료 (확정 셀이 아니면 복원)
        if (m_HoverCellIndex != m_ConfirmedCellIndex)
        {
            RestoreCellColor(m_HoverCellIndex);
            changedVoxelIndices.push_back(m_HoverCellIndex);
        }
        m_HoverCellIndex = -1;
    }

    // ---------- 3. 우클릭: 지금 호버 중인 셀을 목적지로 확정 ----------
    if (rightClicked && m_Npc.HasSelection() && m_HoverCellIndex >= 0)
    {
        // 이전 확정 셀이 있었고, 지금 호버 셀과 다르면 원래 색으로 복원
        if (m_ConfirmedCellIndex >= 0 &&
            m_ConfirmedCellIndex != m_HoverCellIndex)
        {
            RestoreCellColor(m_ConfirmedCellIndex);
            changedVoxelIndices.push_back(m_ConfirmedCellIndex);
        }

        m_ConfirmedCellIndex = m_HoverCellIndex;
        m_VoxelInstances[m_ConfirmedCellIndex].colorType = kColorHover; // 이미 초록이지만 명시적으로 재확인
        changedVoxelIndices.push_back(m_HoverCellIndex);

        m_HoverActive = false; // 목적지 확정 -> 실시간 호버 종료


        // ---- 파이프라인: 선택된 NPC 위치 -> A* -> 마스크 -> CorridorFlowField ----
        auto& goalCoord = m_VoxelCellCoords[m_ConfirmedCellIndex];
        DirectX::XMINT3 goalCell{ goalCoord.x, goalCoord.y, goalCoord.z };

        std::vector<DirectX::XMINT3> path;
        if (m_Npc.SetGroupDestination(goalCell, &path))
        {
            // A* 경로 셀 -> 복셀 인스턴스 인덱스로 변환
            m_PathVoxelIndices.clear();
            m_PathVoxelIndices.reserve(path.size());
            for (const auto& node : path)
            {
                auto it = m_VoxelCoordToIndex.find(MakeCellKey(node.x, node.y, node.z));
                if (it != m_VoxelCoordToIndex.end())    m_PathVoxelIndices.push_back(it->second);
            }


            // 성공 시 디버그 색칠/화살표 갱신 (FlowField의 시각화 책임)
            std::vector<int64_t> prevKeys = m_OccupiedChunkKeys;
            ReleaseChunks(0);
            OccupyChunks(0);
            CollectDebugColorChanges(changedVoxelIndices, prevKeys);
            BuildArrowInstances();
        }
    }
    FlushVoxelInstanceChanges(changedVoxelIndices);
}



void FlowField::BuildArrowInstances()
{
    std::vector<FlowFieldArrowRenderer::InstanceData> instances;

    if (false == m_DebugShowArrows)   return;

    float cellSize = m_VoxelGrid.GetCellSize();
    const float ARROW_LENGTH = cellSize * 0.8f;     // 셀보다 살짝 작게함 -> 옆셀 침범 안하게
    const float HEIGHT_OFFSET = cellSize * 0.8f;    // 지면 띄울 높이

    for (int64_t key : m_OccupiedChunkKeys)
    {
        auto it = m_ChunkToVoxelIndices.find(key);
        if (it == m_ChunkToVoxelIndices.end())   continue;

        for (int idx : it->second)
        {
            const auto& c = m_VoxelCellCoords[idx];

            DirectX::XMFLOAT3 dir;
            if (false == m_Npc.GetFlowField().SampleDirection(m_VoxelGrid, c.x, c.y, c.z, dir))  continue;

            Math::Vector3 worldPos = m_VoxelGrid.GetWorldPos(c.x, c.y, c.z);

            FlowFieldArrowRenderer::InstanceData inst{};
            inst.position[0] = worldPos.GetX();
            inst.position[1] = worldPos.GetY() + HEIGHT_OFFSET;
            inst.position[2] = worldPos.GetZ();
            inst.length = ARROW_LENGTH;
            inst.direction[0] = dir.x;
            inst.direction[1] = dir.y;
            inst.direction[2] = dir.z;

            instances.emplace_back(inst);
        }
        
    }

    FlowFieldArrowRenderer::UpdateInstances(instances);
}

uint32_t FlowField::GetBaseColorType(int instanceIndex) const
{
    auto& c = m_VoxelCellCoords[instanceIndex];
    return (m_VoxelGrid.GetCell(c.x, c.y, c.z) == VoxelGrid::CellType::Walkable) ? 0u : 1u;
}

void FlowField::OccupyChunks(int groupId)
{
    ReleaseChunks(groupId);

    m_OccupiedChunkKeys.clear();
    for (const auto& kv : m_Npc.GetFlowField().GetChunks())
    {
        m_ChunkOccupants[kv.first].push_back(groupId);
        m_OccupiedChunkKeys.push_back(kv.first);
    }
}

void FlowField::ReleaseChunks(int groupId)
{
    for (auto key : m_OccupiedChunkKeys)
    {
        auto it = m_ChunkOccupants.find(key);
        if (it == m_ChunkOccupants.end())    continue;

        // 청크 vector에서 해당 groupId 지움
        auto& occupants = it->second;
        occupants.erase(std::remove(occupants.begin(), occupants.end(), groupId), occupants.end());

        // 지워서 해당 청크에 대한 점유 그룹 없음면 청크 자체(점유에대한)를 지움
        if (occupants.empty())
        {
            m_ChunkOccupants.erase(it);
        }
    }
    m_OccupiedChunkKeys.clear();
}


void FlowField::CollectDebugColorChanges(std::vector<int>& changed, const std::vector<int64_t>& extraKeys)
{
    // 예상 크기 계산
    size_t reserveCount = 0;
    for (auto key : m_OccupiedChunkKeys)
    {
        auto it = m_ChunkToVoxelIndices.find(key);
        if (it != m_ChunkToVoxelIndices.end())   reserveCount += it->second.size();
    }
    for (auto key : extraKeys)
    {
        auto it = m_ChunkToVoxelIndices.find(key);
        if (it != m_ChunkToVoxelIndices.end())   reserveCount += it->second.size();
    }
    changed.reserve(reserveCount + m_PathVoxelIndices.size() + 2);      // 청크 + 경로 + 호버링셀

    // 0. 리셋 - 기본 복셀 색 먼저 씌운다
    auto paintChunk = [&](int64_t key)
    {
        auto it = m_ChunkToVoxelIndices.find(key);
        if (it == m_ChunkToVoxelIndices.end()) return;

        bool useGroupColor = false;
        uint32_t groupColor = 0;

        if (true == m_DebugShowChunks)
        {
            auto occIt = m_ChunkOccupants.find(key);
            if (occIt != m_ChunkOccupants.end() && !occIt->second.empty())
            {
                useGroupColor = true;
                groupColor = 10u + (uint32_t)(occIt->second.back() % 8);
            }
        }

        for (int idx : it->second)
        {
            bool isVisit = false;
            if (true == useGroupColor)
            {
                const auto& c = m_VoxelCellCoords[idx];
                isVisit = m_Npc.GetFlowField().IsVisited(m_VoxelGrid, c.x, c.y, c.z);
            }

            m_VoxelInstances[idx].colorType = isVisit ? groupColor : GetBaseColorType(idx);
            changed.push_back(idx);
        }
    };

    //
    for (int64_t key : m_OccupiedChunkKeys) paintChunk(key);
    for (int64_t key : extraKeys)           paintChunk(key);

    // 1 - 호버 / 확정 목적지
    if (m_ConfirmedCellIndex >= 0)
    {
        m_VoxelInstances[m_ConfirmedCellIndex].colorType = kColorHover;
        changed.push_back(m_ConfirmedCellIndex);
    }
    if (m_HoverCellIndex >= 0)
    {
        m_VoxelInstances[m_HoverCellIndex].colorType = kColorHover;
        changed.push_back(m_HoverCellIndex);
    }


    // 2 - A* 경로
    for (int idx : m_PathVoxelIndices)
    {
        if (idx == m_ConfirmedCellIndex) continue;
        m_VoxelInstances[idx].colorType = kColorPath;        // 빨간색으로 지정
        changed.push_back(idx);
    }
}



void FlowField::RefreshDebugColors(const std::vector<int64_t>& extraKeys)
{
    std::vector<int> changed;
    CollectDebugColorChanges(changed, extraKeys);
    FlushVoxelInstanceChanges(changed);
}



void FlowField::RestoreCellColor(int instanceIndex)
{
    if (instanceIndex < 0 || instanceIndex >= (int)m_VoxelCellCoords.size()) return;
    m_VoxelInstances[instanceIndex].colorType = GetBaseColorType(instanceIndex);
}

// GPU 갱신 호출은 매번 CPU-GPU 동기화를 동반하므로, 호출 횟수가 곧 비용.
// 변경 인덱스가 배열 전체에 흩어져 있으면 연속 구간이 잘게 쪼개져 호출이 폭증함.
// 이 경우 차라리 전체를 한 번에 올리는 게 훨씬 빠름 (동기화 1번 vs N번)
void FlowField::FlushVoxelInstanceChanges(std::vector<int>& changedIndices)
{
    if (changedIndices.empty()) return;

    std::sort(changedIndices.begin(), changedIndices.end());
    // 중복 인덱스 제거 (한 프레임에 같은 셀이 두 번 바뀐 경우 대비)
    changedIndices.erase(std::unique(changedIndices.begin(), changedIndices.end()), changedIndices.end());

    // 구간 개수가 이 값을 넘으면 부분 갱신을 포기하고 전체 업로드로 전환.
    const int MAX_RANGES = 32;

    // 1단계: 실제 업로드 전에 구간 개수만 먼저 셈
    int rangeCount = 0;
    {
        size_t i = 0;
        while (i < changedIndices.size())
        {
            while (i + 1 < changedIndices.size() && changedIndices[i + 1] == changedIndices[i] + 1)
                i++;
            rangeCount++;
            i++;
        }
    }

    // 2단계: 구간이 너무 많으면 전체 업로드 1회로 대체
    if (rangeCount > MAX_RANGES)
    {
        VoxelRenderer::UpdateInstances(m_VoxelInstances);
        return;
    }

    // 구간이 적으면 부분로직
    size_t i = 0;
    while (i < changedIndices.size())
    {
        size_t runStart = i;
        // 인덱스가 연속(1씩 증가)인 동안 구간을 넓힘
        while (i + 1 < changedIndices.size() && changedIndices[i + 1] == changedIndices[i] + 1)
        {
            i++;
        }

        int startIndex = changedIndices[runStart];
        int count = changedIndices[i] - startIndex + 1;

        // m_VoxelInstances에서 해당 구간이 실제로도 메모리상 연속이므로
        // 포인터 하나로 count개를 통째로 넘길 수 있음
        VoxelRenderer::UpdateInstanceRange((uint32_t)startIndex, (uint32_t)count, &m_VoxelInstances[startIndex]);
        i++;

        rangeCount++;
    }

    //char buf[128]; 
    //sprintf_s(buf, "FlushVoxelInstanceChanges: %zu indices -> %d ranges\n",
    //    changedIndices.size(), rangeCount);
    //OutputDebugStringA(buf);
}


void FlowField::Update(float dt)
{
    // Tab: 인게임(카메라 조작) <-> 해제(마우스 보임, 피킹 가능) 모드 토글
    if (GameInput::IsFirstPressed(GameInput::kKey_tab))
    {
        m_MouseCaptured = !m_MouseCaptured;
        ShowCursor(m_MouseCaptured ? FALSE : TRUE);
        GameInput::SetMouseExclusiveMode(m_MouseCaptured);
    }

    if (m_MouseCaptured)
    {
        // 인게임 모드: 기존처럼 카메라만 조작됨 (마우스 회전 + WASD)
        m_CameraController->Update(dt);
    }
    else
    {
        // 해제 모드: 카메라는 멈추고, 클릭으로 NPC 선택/목적지 지정만 가능
        HandlePicking();
    }

    m_Npc.Update(dt); // 모드(카메라/피킹)와 무관하게 매 프레임 이동은 계속 갱신

    // F1: 솔리드 <-> 와이어프레임 토글
    // IsFirstPressed = 키를 막 누른 순간 한 번만 true
    if (GameInput::IsFirstPressed(GameInput::kKey_f1))
    {
        VoxelRenderer::ToggleWireframe();
        NpcRenderer::ToggleWireframe();
    }

    // F2 - flowfield 색 바꾸기
    if (GameInput::IsFirstPressed(GameInput::kKey_f2))
    {
        m_DebugShowChunks = !m_DebugShowChunks;
        RefreshDebugColors();   // 켜고 끌 때 즉시 반영
    }
    // F3 - flowfield dir 시각화
    if (GameInput::IsFirstPressed(GameInput::kKey_f3))
    {
        m_DebugShowArrows = !m_DebugShowArrows;
        FlowFieldArrowRenderer::SetEnabled(m_DebugShowArrows);
        BuildArrowInstances();
    }
}

void FlowField::RenderScene(void)
{
    GraphicsContext& ctx = GraphicsContext::Begin(L"Scene Render");

    // 렌더타겟 + 깊이버퍼 전이
    ctx.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET);
    ctx.TransitionResource(g_SceneDepthBuffer, D3D12_RESOURCE_STATE_DEPTH_WRITE);

    ctx.ClearColor(g_SceneColorBuffer);
    ctx.ClearDepth(g_SceneDepthBuffer);

    ctx.SetRenderTarget(
        g_SceneColorBuffer.GetRTV(),
        g_SceneDepthBuffer.GetDSV());
    ctx.SetViewportAndScissor(m_MainViewport, m_MainScissor);

    // 큐브 드로우
    VoxelRenderer::Render(ctx, m_Camera.GetViewProjMatrix());

    // NPC 드로우
    NpcRenderer::Render(ctx, m_Camera.GetViewProjMatrix());

    // flowfield 화살표
    FlowFieldArrowRenderer::Render(ctx, m_Camera.GetViewProjMatrix());

    // Present 전이
    ctx.TransitionResource(g_SceneColorBuffer, D3D12_RESOURCE_STATE_PRESENT);
    ctx.Finish();
}