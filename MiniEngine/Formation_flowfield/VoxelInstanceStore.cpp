#include "VoxelInstanceStore.h"
#include "ChunkKey.h"
#include <algorithm>

void VoxelInstanceStore::Build()
{
    m_Grid->BuildInstanceList(m_Instances, &m_Coords);
    VoxelRenderer::UpdateInstances(m_Instances);
    RebuildIndices();
}

//-------------------------------------
//
//  인스턴스 배열 일부 갱신용 헬퍼
//
//-------------------------------------
void VoxelInstanceStore::RebuildIndices()
{
    m_CoordToIndex.clear();
    m_CoordToIndex.reserve(m_Coords.size());
    m_ChunkToIndices.clear();

    for (size_t i = 0; i < m_Coords.size(); i++)
    {
        const auto& c = m_Coords[i];
        m_CoordToIndex[MakeCellKey(c.x, c.y, c.z)] = (int)i;
        m_ChunkToIndices[ChunkKeyOf(c.x, c.z)].push_back((int)i);
    }
}

int VoxelInstanceStore::FindIndex(int x, int y, int z) const
{
    auto it = m_CoordToIndex.find(MakeCellKey(x, y, z));
    return (it != m_CoordToIndex.end()) ? it->second : -1;
}
int VoxelInstanceStore::FindIndex(DirectX::XMINT3 v) const
{
    auto it = m_CoordToIndex.find(MakeCellKey(v));
    return (it != m_CoordToIndex.end()) ? it->second : -1;
}


const std::vector<int>* VoxelInstanceStore::IndicesInChunk(int64_t chunkKey) const
{
    auto it = m_ChunkToIndices.find(chunkKey);
    return (it != m_ChunkToIndices.end()) ? &it->second : nullptr;
}

uint32_t VoxelInstanceStore::GetBaseColorType(int index) const
{
    const auto& c = m_Coords[index];
    return (m_Grid->GetCell(c.x, c.y, c.z) == VoxelGrid::CellType::Walkable) ? 0u : 1u;
}

void VoxelInstanceStore::EraseFromChunkIndex(int64_t chunkKey, int idx)
{
    auto it = m_ChunkToIndices.find(chunkKey);
    if (it == m_ChunkToIndices.end())   return;

    auto& vec = it->second;
    vec.erase(std::remove(vec.begin(), vec.end(), idx), vec.end());
    if (vec.empty())    m_ChunkToIndices.erase(it);
}

void VoxelInstanceStore::ReindexInChunkIndex(int64_t chunkKey, int oldIdx, int newIdx)
{
    auto it = m_ChunkToIndices.find(chunkKey);
    if (it == m_ChunkToIndices.end())   return;

    for (int& v : it->second)
    {
        if (v == oldIdx) 
        { 
            v = newIdx; 
            break;
        }
    }
}

void VoxelInstanceStore::RemoveInstanceAt(int idx, std::vector<int>& dirty)
{
    const VoxelGrid::CellCoord co = m_Coords[idx];

    m_CoordToIndex.erase(MakeCellKey(co.x, co.y, co.z));
    EraseFromChunkIndex(ChunkKeyOf(co.x, co.z), idx);

    int lastIdx = (int)m_Instances.size() - 1;
    if (idx != lastIdx)
    {
        // 배열 끝 원소를 빈 자리로 이동 — 이 한 원소만 인덱스가 바뀐다
        m_Instances[idx] = m_Instances[lastIdx];
        m_Coords[idx] = m_Coords[lastIdx];

        const auto& sc = m_Coords[idx];
        m_CoordToIndex[MakeCellKey(sc.x, sc.y, sc.z)] = idx;
        ReindexInChunkIndex(ChunkKeyOf(sc.x, sc.z), lastIdx, idx);

        dirty.push_back(idx);
    }

    m_Instances.pop_back();
    m_Coords.pop_back();
}

void VoxelInstanceStore::AppendInstance(const VoxelGrid::TerrainEditDelta::AddedCell& cell,
    std::vector<int>& dirty)
{
    const float cellSize = m_Grid->GetCellSize();
    int idx = (int)m_Instances.size();

    VoxelRenderer::InstanceData inst{};
    inst.position[0] = cell.x * cellSize;
    inst.position[1] = cell.y * cellSize;
    inst.position[2] = cell.z * cellSize;
    inst.scale = cellSize;
    // BuildInstanceList와 동일한 기본색 규칙 (디버그 덧칠은 상위 계층 소관)
    inst.colorType = (cell.type == VoxelGrid::CellType::Walkable) ? 0u : 1u;

    m_Instances.push_back(inst);
    m_Coords.push_back({ cell.x, cell.y, cell.z });

    m_CoordToIndex[MakeCellKey(cell.x, cell.y, cell.z)] = idx;
    m_ChunkToIndices[ChunkKeyOf(cell.x, cell.z)].push_back(idx);

    dirty.push_back(idx);
}





//-------------------------------------
//
//  동적 지형 편집
//
//-------------------------------------
// 동적 지형 편집
void VoxelInstanceStore::ApplyDelta(const VoxelGrid::TerrainEditDelta& delta)
{
    std::vector<int> dirty;
    dirty.reserve(delta.removed.size() + delta.added.size());

    // 1 제거 — 인덱스 내림차순 필수
    // 오름차순이면 끝에서 스왑해온 원소가 이후 제거 대상일 때 인덱스가 어긋난다
    std::vector<int> removeIndices;
    removeIndices.reserve(delta.removed.size());
    for (const auto& co : delta.removed)
    {
        int idx = FindIndex(co.x, co.y, co.z);
        if (idx >= 0)   removeIndices.push_back(idx);
    }
    std::sort(removeIndices.begin(), removeIndices.end(), std::greater<int>());
    removeIndices.erase(std::unique(removeIndices.begin(), removeIndices.end()), removeIndices.end());
    for (int idx : removeIndices)
    {
        RemoveInstanceAt(idx, dirty);
    }

    // 2 추가 — 배열 끝에 append
    for (const auto& cell : delta.added)
    {
        AppendInstance(cell, dirty);
    }

    // 3 축소로 범위를 벗어난 dirty 제거 (스왑 대상이 뒤이어 pop된 경우)
    const int finalSize = (int)m_Instances.size();
    dirty.erase(std::remove_if(dirty.begin(), dirty.end(), [finalSize](int i) { 
        return i >= finalSize; 
        }),
        dirty.end());

    // 4 카운트 갱신 후 변경분만 업로드
    VoxelRenderer::SetInstanceCount((uint32_t)finalSize);
    Flush(dirty);
}


// GPU 갱신 호출은 매번 CPU-GPU 동기화를 동반하므로, 호출 횟수가 곧 비용
// 변경 인덱스가 흩어져 있으면 연속 구간이 잘게 쪼개져 호출이 폭증 -> 전체 업로드가 오히려 빠름
void VoxelInstanceStore::Flush(std::vector<int>& changedIndices)
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
            {
                i++;
            }

            rangeCount++;
            i++;
        }
    }

    // 2단계: 구간이 너무 많으면 전체 업로드 1회로 대체
    if (rangeCount > MAX_RANGES)
    {
        VoxelRenderer::UpdateInstances(m_Instances);
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
        VoxelRenderer::UpdateInstanceRange((uint32_t)startIndex, (uint32_t)count, &m_Instances[startIndex]);
        i++;

        rangeCount++;
    }
}
