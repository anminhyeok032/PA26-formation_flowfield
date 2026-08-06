#include "DebugVisualizer.h"
#include "FlowFieldArrowRenderer.h"
#include "ChunkKey.h"
#include <algorithm>

void DebugVisualizer::Initialize(VoxelInstanceStore* store, const VoxelGrid* grid,
    const NpcManager* npc)
{
    m_Store = store;
    m_Grid = grid;
    m_Npc = npc;
}

int DebugVisualizer::ConfirmedIndex() const
{
    if (!m_HasConfirmed) return -1;
    return m_Store->FindIndex(m_ConfirmedCoord.x, m_ConfirmedCoord.y, m_ConfirmedCoord.z);
}

bool DebugVisualizer::IsPathCoord(const VoxelGrid::CellCoord& c) const
{
    for (const auto& p : m_PathCoords)
        if (p.x == c.x && p.y == c.y && p.z == c.z) return true;
    return false;
}

void DebugVisualizer::OccupyChunks(int groupId)
{
    ReleaseChunks(groupId);
    m_OccupiedChunkKeys.clear();

    // 모든 leaf 청크를 합집합으로 수집 (갈라져도 칠해지게)
    std::unordered_set<int64_t> seen;
    const int leafCount = m_Npc->GetLeafCount();
    for (int li = 0; li < leafCount; ++li)
    {
        for (const auto& kv : m_Npc->GetLeafField(li).GetChunks())
        {
            if (false == seen.insert(kv.first).second)   continue;  // 중복 청크 스킵
            m_ChunkOccupants[kv.first].push_back(groupId);
            m_OccupiedChunkKeys.push_back(kv.first);
        }
    }
}

void DebugVisualizer::ReleaseChunks(int groupId)
{
    for (auto key : m_OccupiedChunkKeys)
    {
        auto it = m_ChunkOccupants.find(key);
        if (it == m_ChunkOccupants.end())    continue;

        // 청크 vector에서 해당 groupId 삭제
        auto& occupants = it->second;
        occupants.erase(std::remove(occupants.begin(), occupants.end(), groupId), occupants.end());

        // 지워서 해당 청크에 대한 점유 그룹 없음면 청크 자체(점유에대한)를 지움
        if (occupants.empty())  m_ChunkOccupants.erase(it);
    }
    m_OccupiedChunkKeys.clear();
}

bool DebugVisualizer::IsConfirmedIndex(int index) const
{
    if (!m_HasConfirmed || !m_Store->IsValidIndex(index)) return false;
    const auto& c = m_Store->CoordAt(index);
    return c.x == m_ConfirmedCoord.x && c.y == m_ConfirmedCoord.y && c.z == m_ConfirmedCoord.z;

}

uint32_t DebugVisualizer::GetLayeredColorType(int index) const
{
    if (!m_Store->IsValidIndex(index)) return kColorDefault;

    const auto& c = m_Store->CoordAt(index);

    // 경로 루프가 확정 셀을 skip하므로 확정이 경로보다 우선
    if (m_HasConfirmed && c.x == m_ConfirmedCoord.x && c.y == m_ConfirmedCoord.y && c.z == m_ConfirmedCoord.z)
        return kColorHover;

    if (true == IsPathCoord(c)) return kColorPath;

    if (true == m_ShowChunks)
    {
        auto occIt = m_ChunkOccupants.find(ChunkKeyOf(c.x, c.z));
        if (occIt != m_ChunkOccupants.end() && !occIt->second.empty())
        {
            if (m_Npc->IsVisitedAny(*m_Grid, c.x, c.y, c.z)) return 10u + (uint32_t)(occIt->second.back() % 8);
        }
    }

    return m_Store->GetBaseColorType(index);
}

void DebugVisualizer::RestoreCellColor(int index)
{
    if (!m_Store->IsValidIndex(index)) return;
    m_Store->SetColor(index, GetLayeredColorType(index));
}

void DebugVisualizer::CollectDebugColorChanges(std::vector<int>& changed,
    const std::vector<int64_t>& extraKeys)
{
    size_t reserveCount = 0;
    for (auto key : m_OccupiedChunkKeys)
    {
        const std::vector<int>* indices = m_Store->IndicesInChunk(key);
        if (indices)    reserveCount += indices->size();
    }
    for (auto key : extraKeys)
    {
        const std::vector<int>* indices = m_Store->IndicesInChunk(key);
        if (indices)    reserveCount += indices->size();
    }
    changed.reserve(reserveCount + m_PathCoords.size() + 2);    // 청크 + 경로 + 호버링셀

    // 0 리셋 - 기본 복셀 색 먼저 씌우기
    auto paintChunk = [&](int64_t key)
    {
        const std::vector<int>* indices = m_Store->IndicesInChunk(key);
        if (nullptr == indices) return;

        bool     useGroupColor = false;
        uint32_t groupColor = 0;

        if (true == m_ShowChunks)
        {
            auto occIt = m_ChunkOccupants.find(key);
            if (occIt != m_ChunkOccupants.end() && !occIt->second.empty())
            {
                useGroupColor = true;
                groupColor = 10u + (uint32_t)(occIt->second.back() % 8);
            }
        }

        for (int idx : *indices)
        {
            bool isVisit = false;
            if (true == useGroupColor)
            {
                const auto& c = m_Store->CoordAt(idx);
                isVisit = m_Npc->IsVisitedAny(*m_Grid, c.x, c.y, c.z);
            }
            m_Store->SetColor(idx, isVisit ? groupColor : m_Store->GetBaseColorType(idx));
            changed.push_back(idx);
        }
    };

    for (int64_t key : m_OccupiedChunkKeys) paintChunk(key);
    for (int64_t key : extraKeys)           paintChunk(key);

    // 1 - 호버 / 확정 목적지
    const int confirmedIdx = ConfirmedIndex();
    if (confirmedIdx >= 0)
    {
        m_Store->SetColor(confirmedIdx, kColorHover);
        changed.push_back(confirmedIdx);
    }
    if (m_HoverIndex >= 0)
    {
        m_Store->SetColor(m_HoverIndex, kColorHover);
        changed.push_back(m_HoverIndex);
    }

    // 2 - A* 경로 (좌표 -> 현재 인덱스 조회, 편집으로 사라진 셀은 -1이라 자연히 skip)
    for (const auto& p : m_PathCoords)
    {
        int idx = m_Store->FindIndex(p.x, p.y, p.z);
        if (idx < 0 || idx == confirmedIdx) continue;
        m_Store->SetColor(idx, kColorPath);
        changed.push_back(idx);
    }
}

void DebugVisualizer::RefreshDebugColors(const std::vector<int64_t>& extraKeys)
{
    std::vector<int> changed;
    CollectDebugColorChanges(changed, extraKeys);
    m_Store->Flush(changed);
}

void DebugVisualizer::BuildArrowInstances()
{
    std::vector<FlowFieldArrowRenderer::InstanceData> instances;
    if (false == m_ShowArrows)
    {
        FlowFieldArrowRenderer::UpdateInstances(instances);
        return;
    }

    const float cellSize = m_Grid->GetCellSize();
    const float ARROW_LENGTH = cellSize * 0.8f;         // 셀보다 살짝 작게함 -> 옆셀 침범 안하게
    const float HEIGHT_OFFSET = cellSize * 0.8f;        // 지면 띄울 높이

    for (int64_t key : m_OccupiedChunkKeys)
    {
        const std::vector<int>* indices = m_Store->IndicesInChunk(key);
        if (nullptr == indices)   continue;

        for (int idx : *indices)
        {
            const auto& c = m_Store->CoordAt(idx);

            DirectX::XMINT3 d;
            if (false == m_Npc->SampleDirectionAny(*m_Grid, c.x, c.y, c.z, d))  continue;

            // 정수 델타 -> 단위벡터. GetWorldPos가 등방이라 기존 실수 방향과 동일 결과
            Math::Vector3 dirVec = Math::Normalize(Math::Vector3((float)d.x, (float)d.y, (float)d.z));
            Math::Vector3 worldPos = m_Grid->GetWorldPos(c.x, c.y, c.z);

            FlowFieldArrowRenderer::InstanceData inst{};
            inst.position[0] = worldPos.GetX();
            inst.position[1] = worldPos.GetY() + HEIGHT_OFFSET;
            inst.position[2] = worldPos.GetZ();
            inst.length = ARROW_LENGTH;
            inst.direction[0] = dirVec.GetX();
            inst.direction[1] = dirVec.GetY();
            inst.direction[2] = dirVec.GetZ();

            instances.emplace_back(inst);
        }
    }

    FlowFieldArrowRenderer::UpdateInstances(instances);
}

void DebugVisualizer::RefreshFieldVisuals()
{
    const std::vector<int64_t> prevKeys = m_OccupiedChunkKeys;

    OccupyChunks(0);
    RefreshDebugColors(prevKeys);   // gpu 업로드
    BuildArrowInstances();
}

void DebugVisualizer::OnGroupArrived()
{
    std::vector<int> changed;

    // 1. 경로 / 확정 목적지 레이어 해제 (색 계산 전에 먼저 지워야 기본색이 나옴)
    for (const auto& p : m_PathCoords)
    {
        int idx = m_Store->FindIndex(p.x, p.y, p.z);
        if (idx >= 0) changed.push_back(idx);
    }
    m_PathCoords.clear();

    const int confirmedIdx = ConfirmedIndex();
    if (confirmedIdx >= 0) changed.push_back(confirmedIdx);
    m_HasConfirmed = false;

    // 2. 청크 점유 해제 (해제 전 키를 남겨야 그 청크들도 다시 칠할 수 있음)
    std::vector<int64_t> prevKeys = m_OccupiedChunkKeys;
    ReleaseChunks(0);

    // 3. 레이어가 전부 사라진 상태에서 색 재계산
    for (int idx : changed)   RestoreCellColor(idx);
    CollectDebugColorChanges(changed, prevKeys);

    // 4. 화살표 제거 (m_OccupiedChunkKeys가 비었으므로 빈 인스턴스로 갱신됨)
    BuildArrowInstances();

    m_Store->Flush(changed);
}
