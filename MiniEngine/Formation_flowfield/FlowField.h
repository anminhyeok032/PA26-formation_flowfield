#pragma once
#include "GameCore.h"
#include "Camera.h"
#include "CameraController.h"
#include "HeightMap.h"
#include "VoxelGrid.h"
#include "VoxelRenderer.h"
#include "NPCRenderer.h"
#include <d3d12.h>
#include <vector>
#include <unordered_map>
#include <memory>

// npc pathfinding
#include "AStarPathfinder.h"
#include "PathCorridor.h"
#include "CorridorFlowField.h"
#include "ChunkKey.h"

class FlowField : public GameCore::IGameApp
{
public:
    virtual void Startup(void) override;
    virtual void Cleanup(void) override;
    virtual void Update(float deltaT) override;
    virtual void RenderScene(void) override;

private:
    Camera                              m_Camera;
    std::unique_ptr<CameraController>   m_CameraController;
    D3D12_VIEWPORT                      m_MainViewport;
    D3D12_RECT                          m_MainScissor;

    HeightMap                           m_HeightMap;
    VoxelGrid                           m_VoxelGrid;

    // ---- NPC 선택 관련 ----
    std::vector<NPCRenderer::InstanceData>  m_NpcInstances;             // 지속 보관 (색상만 바꿔 재업로드하기 위해)
    int                                     m_SelectedNpcIndex = -1;    // 현재 선택된 NPC 인덱스, 없으면 -1

    // ---- 길찾기(A* -> 마스크 -> CorridorFlowField) ----
    // TODO: 지금은 NPC 1마리 선택만 가능하므로 인원수=1로 취급.
    //       나중에 그룹(여러 NPC 선택)이 추가되면 memberCount를 실제 그룹 크기로 교체.
    AStarPathfinder     m_Pathfinder;
    CorridorFlowField   m_CorridorField;
    bool                m_HasGoal = false;

    DirectX::XMINT3     m_NpcCurrentCell{ -1, -1, -1 };  // 이미 도착해서 확정된 셀
    DirectX::XMINT3     m_NpcTargetCell{ -1, -1, -1 };   // 지금 이동해서 향하고 있는 다음 셀
    bool                m_NpcCellInitialized = false;

    // ------------ 청크 시각화 관련 ----------------
    // 
    // 청크 디버그 시각화(f2)
    bool                m_DebugShowChunks = false;
    // 청크 key / 청크 내부 복셀 인스턴스 인덱스 목록
    std::unordered_map<int64_t, std::vector<int>> m_ChunkToVoxelIndices;

    // 청크 key / 청크 점유중인 그룹 ID (진입 순서순)
    // back()으로 색 표시를 기준 - 해당 그룹 도착시, erase해라.
    std::unordered_map<int64_t, std::vector<int>> m_ChunkOccupants;

    // 그룹들이 점유한 청크 키
    std::vector<int64_t> m_OccupiedChunkKeys;
    // A* 경로 복셀 인스턴스 인덱스 (진빨강 추가 표시용)
    std::vector<int64_t> m_PathVoxelIndices;

    // 특정 청크 최종 colorType 우선순위 저장
    uint32_t GetBaseColorType(int instanceIndex) const;

    // 점유 청크 등록 / 해제
    void OccupyChunks(int groupId);
    void ReleaseChunks(int groupId);

    // 레이어 순서대로 덮어써서 디버그 색을 갱신하고 GPU에 반영.
    // extraKeys: 이번에 해제되어 기본색으로 돌려놔야 하는 청크 키들 (없으면 비워서 호출)
    void RefreshDebugColors(const std::vector<int64_t>& extraKeys = {});


    // ----------------- 목적지 복셀 선택 관련 --------------
    std::vector<VoxelRenderer::InstanceData> m_VoxelInstances;  // 지속 보관
    std::vector<VoxelGrid::CellCoord>        m_VoxelCellCoords; // m_VoxelInstances와 1:1 대응

    // (x,y,z) 좌표 -> m_VoxelInstances 배열의 인덱스로 즉시 찾아가기 위한 색인.
    // BuildInstanceList 직후 딱 한 번만 만들고, 그 이후로는 조회만 함.
    std::unordered_map<int64_t, int> m_VoxelCoordToIndex;

    // 좌표 세 개(x,y,z)를 해시맵 키로 쓸 숫자 하나로 합치는 함수
    static int64_t MakeVoxelCoordKey(int x, int y, int z)
    {
        return (int64_t)(x & 0x1FFFFF) |
            ((int64_t)(y & 0x1FFFFF) << 21) |
            ((int64_t)(z & 0x1FFFFF) << 42);
    }

    int  m_HoverCellIndex = -1;      // 지금 마우스가 가리키는 셀 (매 프레임 갱신)
    int  m_ConfirmedCellIndex = -1;  // 우클릭으로 확정된 목적지 (다음 확정 전까지 유지)
    bool m_HoverActive = false;      // NPC 선택 시 true, 우클릭으로 확정되는 순간 false (호버 정지)

    void RestoreCellColor(int instanceIndex); // 헬퍼: 실제 셀 타입 기준으로 색 복원

    // true = 인게임(카메라 조작 모드, 커서 숨김) / false = 해제(커서 보임, 피킹 가능)
    // 기본값 true로 시작 -> Startup()에서 커서 숨김과 짝을 맞춰야 함
    bool m_MouseCaptured = true;

    // ---- 피킹(레이캐스팅) 관련 헬퍼 ----
    // 화면 좌표(마우스 클라이언트 좌표)를 월드 공간 레이(origin, dir)로 변환
    bool ScreenPointToRay(int mouseX, int mouseY, Math::Vector3& outOrigin, Math::Vector3& outDir) const;

    // 매 프레임 좌/우클릭을 검사해서 NPC 선택 / 목적지 지정 처리
    void HandlePicking();


    // --------------NPC 이동----------------
    // 확정된 목적지를 향해 선택된 NPC를 매 프레임 이동시킴
    void UpdateNpcMovement(float dt);
    // 격자 셀 좌표 -> NPC가 실제로 서 있어야 할 월드 좌표.
    Math::Vector3 GetNpcStandPos(const DirectX::XMINT3& cell, float npcHalfHeight) const;


    // 바뀐 복셀 gpu 업로드
    void FlushVoxelInstanceChanges(std::vector<int>& changedIndices);
};