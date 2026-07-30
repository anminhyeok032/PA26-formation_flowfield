#include "TerrainEditor.h"
#include "PreviewRenderer.h"

void TerrainEditor::Initialize(VoxelGrid* grid, VoxelInstanceStore* store,
    DebugVisualizer* debug, ChunkGraph* chunkGraph)
{
    m_Grid = grid;
    m_Store = store;
    m_Debug = debug;
    m_ChunkGraph = chunkGraph;
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

bool TerrainEditor::HandlePicking(bool hit, const DirectX::XMINT3& cell, bool commitPressed)
{
    // 허공 조준 -> 미리보기 끔 (이미 꺼져 있으면 재업로드 스킵)
    if (false == hit)
    {
        HidePreview();
        return false;
    }

    // 앵커가 안 바뀌었으면 미리보기 GPU 업로드 스킵
    bool anchorChanged = (cell.x != m_PreviewAnchor.x ||
                          cell.y != m_PreviewAnchor.y ||
                          cell.z != m_PreviewAnchor.z);
    if (anchorChanged) UpdatePreview(cell);

    // 우클릭 - 현재 미리보기 박스를 실제로 커밋
    if (commitPressed && m_PreviewVisible)
    {
        m_LastEdited = ComputeBoxCells(cell.x, cell.y, cell.z);
        ApplyTerrainEdit(ComputeBoxCells(cell.x, cell.y, cell.z));
        // 지형이 바뀌었으니 같은 앵커라도 다음 프레임에 강제 재평가
        m_PreviewAnchor = { INT32_MIN, INT32_MIN, INT32_MIN };
        return true;
        
    }
    return false;
}

void TerrainEditor::ApplyTerrainEdit(const std::vector<DirectX::XMINT3>& cells)
{
    VoxelGrid::TerrainEditDelta delta;
    m_Grid->OverwriteCells(cells, VoxelGrid::CellType::Blocked, delta);

    // 반드시 OverwriteCells 이후
    // 라벨링이 GetSurfaceYList/IsWalkable을 읽는데, 그 값이 여기서 확정
    // 입력은 delta가 아니라 cells - delta.removed는 y를 0으로 뭉개고,
    // cells는 OverwriteCells가 touchedCol을 유도한 원본이라 항상 상위집합이다.
    m_ChunkGraph->RefreshAround(*m_Grid, cells);
    
    m_Store->ApplyDelta(delta);
    m_Debug->ClearHover();
    m_Debug->RefreshDebugColors();
}

void TerrainEditor::OnDeactivate()
{
    HidePreview();
}