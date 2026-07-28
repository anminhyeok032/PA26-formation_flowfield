#include "TerrainEditor.h"
#include "PreviewRenderer.h"

void TerrainEditor::Initialize(VoxelGrid* grid, VoxelInstanceStore* store, DebugVisualizer* debug)
{
    m_Grid = grid;
    m_Store = store;
    m_Debug = debug;
}

std::vector<DirectX::XMINT3> TerrainEditor::ComputeBoxCells(int hx, int hy, int hz) const
{
    constexpr int R = 1;   // 반경 1 -> 3x3x3
    std::vector<DirectX::XMINT3> cells;
    cells.reserve((2 * R + 1) * (2 * R + 1) * (2 * R + 1));

    for (int y = hy + 1; y <= hy + 1 + (2 * R); ++y)
    {
        for (int z = hz - R; z <= hz + R; ++z)
        {
            for (int x = hx - R; x <= hx + R; ++x)
            {
                cells.push_back({ x, y, z });
            }
        }
    }
    return cells;
}

void TerrainEditor::HidePreview()
{
    if (false == m_PreviewVisible) return;

    PreviewRenderer::UpdateInstances({});
    m_PreviewVisible = false;
    m_PreviewAnchor = { INT32_MIN, INT32_MIN, INT32_MIN };
}

void TerrainEditor::UpdatePreview(const DirectX::XMINT3& anchor)
{
    std::vector<DirectX::XMINT3> boxCells = ComputeBoxCells(anchor.x, anchor.y, anchor.z);

    std::vector<PreviewRenderer::InstanceData> previewInstances;
    previewInstances.reserve(boxCells.size());

    const float cellSize = m_Grid->GetCellSize();
    for (const auto& c : boxCells)
    {
        // 맵 밖 셀은 미리보기에서도 제외 (SetCell의 bounds 필터와 동일 기준)
        if (!m_Grid->IsInBounds(c.x, c.y, c.z))  continue;

        Math::Vector3 worldPos = m_Grid->GetWorldPos(c.x, c.y, c.z);

        PreviewRenderer::InstanceData inst{};
        inst.position[0] = worldPos.GetX();
        inst.position[1] = worldPos.GetY();
        inst.position[2] = worldPos.GetZ();
        inst.scale = cellSize;
        inst.colorType = kColorPredEdit;   // 반투명 노랑 (CubePS)
        previewInstances.push_back(inst);
    }

    PreviewRenderer::UpdateInstances(previewInstances);
    m_PreviewVisible = !previewInstances.empty();
    m_PreviewAnchor = anchor;
}

void TerrainEditor::HandlePicking(bool hit, const DirectX::XMINT3& cell, bool commitPressed)
{
    // 허공 조준 -> 미리보기 끔 (이미 꺼져 있으면 재업로드 스킵)
    if (false == hit)
    {
        HidePreview();
        return;
    }

    // 앵커가 안 바뀌었으면 미리보기 GPU 업로드 스킵
    bool anchorChanged = (cell.x != m_PreviewAnchor.x ||
                          cell.y != m_PreviewAnchor.y ||
                          cell.z != m_PreviewAnchor.z);
    if (anchorChanged) UpdatePreview(cell);

    // 우클릭 - 현재 미리보기 박스를 실제로 커밋
    if (commitPressed && m_PreviewVisible)
    {
        ApplyTerrainEdit(ComputeBoxCells(cell.x, cell.y, cell.z));
        // 지형이 바뀌었으니 같은 앵커라도 다음 프레임에 강제 재평가
        m_PreviewAnchor = { INT32_MIN, INT32_MIN, INT32_MIN };
    }
}

void TerrainEditor::ApplyTerrainEdit(const std::vector<DirectX::XMINT3>& cells)
{
    VoxelGrid::TerrainEditDelta delta;
    m_Grid->OverwriteCells(cells, VoxelGrid::CellType::Blocked, delta);
    m_Store->ApplyDelta(delta);

    // 확정/경로는 좌표로 보관되므로 인덱스 재매핑이 필요 없음.
    // 호버만 리셋 — 다음 프레임 피킹이 재계산한다.
    m_Debug->ClearHover();
    m_Debug->RefreshDebugColors();
}

void TerrainEditor::OnDeactivate()
{
    HidePreview();
}