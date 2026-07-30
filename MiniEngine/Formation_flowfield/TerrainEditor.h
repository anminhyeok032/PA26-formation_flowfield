#pragma once
#include "VoxelGrid.h"
#include "VoxelInstanceStore.h"
#include "DebugVisualizer.h"
#include "ChunkGraph.h"
#include <DirectXMath.h>
#include <vector>

// 동적 지형 생성(3x3x3 박스)
// 미리보기 렌더와 실제 커밋이 ComputeBoxCells 하나를 공유
//
// 편집 모드 상태(EditMode)는 앱 셸(FlowField)이 소유하고,
// 이 클래스는 활성/비활성 신호와 피킹 결과만 받는다 — 입력 방식과 독립
class TerrainEditor
{
public:
    void Initialize(VoxelGrid* grid, VoxelInstanceStore* store, 
        DebugVisualizer* debug, ChunkGraph* chunkGraph);

    // 지형 생성용 피킹(미리보기 + 우클릭시 커밋)
    // 지형 편집 모드일 때 매 프레임 호출.
    // hit=false면 허공 조준 -> 미리보기 끔. commitPressed면 현재 박스를 커밋
    bool HandlePicking(bool hit, const DirectX::XMINT3& cell, bool commitPressed);

    const std::vector<DirectX::XMINT3>& GetLastEditedCells() const { return m_LastEdited; }

    // 편집 모드에서 빠져나갈 때 호출 — 미리보기 잔상 제거
    void OnDeactivate();

private:
    // 피킹 셀 기준 3x3x3 박스 셀 집합. 바닥 y = hy+1 (피킹 셀 위에 얹힘)
    // 미리보기와 커밋이 반드시 이 함수 하나를 공유해야 함
    std::vector<DirectX::XMINT3> ComputeBoxCells(int hx, int hy, int hz) const;

    void UpdatePreview(const DirectX::XMINT3& anchor);
    void HidePreview();
    void ApplyTerrainEdit(const std::vector<DirectX::XMINT3>& cells);   // N x N x N 박스 지형에 커밋

    VoxelGrid* m_Grid = nullptr;
    VoxelInstanceStore* m_Store = nullptr;
    DebugVisualizer* m_Debug = nullptr;
    ChunkGraph* m_ChunkGraph = nullptr;

    // 앵커가 안 바뀌면 미리보기 GPU 업로드를 건너뛰기 위한 캐시
    DirectX::XMINT3 m_PreviewAnchor{ INT32_MIN, INT32_MIN, INT32_MIN }; // 마지막으로 미리보기 그린셀
    bool            m_PreviewVisible = false;                           // 마지막으로 그린곳 있는지 확인용

    std::vector<DirectX::XMINT3> m_LastEdited;              // 매 프레임 벡터 반환을 피하려 멤버로 보관
};