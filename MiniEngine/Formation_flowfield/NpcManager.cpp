#include "NpcManager.h"
#include "ChunkKey.h"
#include "GridNeighbors.h"
#include <cmath>
#include <queue>

void NpcManager::Init(const VoxelGrid& grid)
{
    m_Grid = &grid;
    const float voxelSize = grid.GetCellSize();
    const float NPC_WIDTH = voxelSize / 2.0f;
    const float NPC_HEIGHT = voxelSize * 3.0f / 2.0f;

    m_NpcInstances.clear();
    const int TARGET_COUNT = 1'00;
    //m_NpcInstances.reserve(TARGET_COUNT);

    const int baseX = 100;
    const int baseZ = 100;

    // 시작 셀이 자체적으로 걸을 수 있는 표면인지 확인 후, 그 y를 구함
    VoxelGrid::SurfaceSpan startSurfaces = grid.GetSurfaceYList(baseX, baseZ);
    if (startSurfaces.count == 0)
    {
        // 시작점 자체가 허공/막힘이면 배치 불가 — 조용히 종료
        NpcRenderer::UpdateInstances(m_NpcInstances);
        return;
    }
    int startY = startSurfaces.data[0];   // 여러 표면이면 가장 아래(첫 번째)를 기준으로

    // BFS: 시작 셀에서 실제로 걸어서(GetWalkableNeighbors) 도달 가능한 컬럼만 방문.
    // 컬럼(x,z) 단위로 중복 방문 방지 (같은 컬럼의 다른 y는 배치 목적상 굳이 재방문 안 함).
    std::unordered_set<int64_t> visitedColumns;
    std::queue<DirectX::XMINT3> q;

    DirectX::XMINT3 startCell{ baseX, startY, baseZ };
    visitedColumns.insert(MakeCellKey(baseX, 0, baseZ));   // y는 0 고정 — 컬럼 단위 방문 표시용
    q.push(startCell);

    int placed = 0;
    std::vector<NeighborInfo> neighbors;

    while (!q.empty() && placed < TARGET_COUNT)
    {
        DirectX::XMINT3 cur = q.front();
        q.pop();

        // 현재 셀에 NPC 배치
        float surfY = (float)cur.y * voxelSize;
        NpcRenderer::InstanceData inst = {};
        inst.scaleXZ = NPC_WIDTH;
        inst.scaleY = NPC_HEIGHT;
        inst.position[0] = cur.x * voxelSize;
        inst.position[1] = surfY + (voxelSize / 2.0f) + inst.scaleY + 0.1f;
        inst.position[2] = cur.z * voxelSize;
        inst.colorType = 0;
        m_NpcInstances.push_back(inst);
        placed++;

        if (placed >= TARGET_COUNT) break;

        // 실제로 걸어서 갈 수 있는 이웃만 확장 (연결성 보장 — 고립 지형 배제)
        GetWalkableNeighbors(*m_Grid, cur, neighbors);
        for (const auto& n : neighbors)
        {
            int64_t colKey = MakeCellKey(n.pos.x, 0, n.pos.z);
            if (visitedColumns.insert(colKey).second)   // 처음 방문하는 컬럼만
                q.push(n.pos);
        }
    }
    int size = (int)m_NpcInstances.size();
    NpcRenderer::UpdateInstances(m_NpcInstances);
}

bool NpcManager::TrySelectNpc(const Math::Vector3& rayOrigin, const Math::Vector3& rayDir)
{
    int hitIndex = -1;
    float closestDist = FLT_MAX;
    const float PICK_MAX_DISTANCE = 500.0f;

    for (size_t i = 0; i < m_NpcInstances.size(); i++)
    {
        NpcRenderer::Capsule cap = NpcRenderer::MakeCapsule(m_NpcInstances[i]);
        float t;
        if (NpcRenderer::RayIntersectsCapsule(rayOrigin, rayDir, PICK_MAX_DISTANCE, cap, t))
        {
            if (t >= 0.0f && t < closestDist) { closestDist = t; hitIndex = (int)i; }
        }
    }

    // 아무도 안 맞았으면 기존 선택 상태 유지 (클릭이 허공을 쳤을 뿐, 해제 의도 아님)
    if (hitIndex < 0)
        return m_GroupSelected;

    // 맞았으면 그룹 선택 토글 (누구를 맞췄는지는 무관 — 그룹 전체가 한 단위)
    m_GroupSelected = !m_GroupSelected;
    m_SelectedNpcIndex = m_GroupSelected ? hitIndex : -1;   // 필요 시 참조용으로만 보관

    uint32_t color = m_GroupSelected ? 2u : 0u;
    for (auto& inst : m_NpcInstances)
        inst.colorType = color;

    NpcRenderer::UpdateInstances(m_NpcInstances);
    return m_GroupSelected;
}

bool NpcManager::SetGroupDestination(const DirectX::XMINT3& goalCell,
    std::vector<DirectX::XMINT3>* outPath)
{
    m_HasGoal = false;
    const size_t n = m_NpcInstances.size();
    if (n == 0) return false;

    // --- 1. 각 NPC의 시작 셀 수집 (마스크 시드 + 이동 초기화에 공유) ---
    // FindNearestWalkable을 여기서 1회만 돌리고, 결과를 멤버에 저장해 InitGroupMovement가 재사용.
    m_StartCells.clear();
    m_StartCells.resize(n, { -1,-1,-1 });

    Math::Vector3 centroid(0.0f, 0.0f, 0.0f);
    int validCount = 0;
    for (size_t i = 0; i < n; ++i)
    {
        const auto& inst = m_NpcInstances[i];
        Math::Vector3 p(inst.position[0], inst.position[1], inst.position[2]);
        int sx, sy, sz;
        if (m_Grid->FindNearestWalkable(p, sx, sy, sz))
        {
            m_StartCells[i] = { sx, sy, sz };
            centroid += Math::Vector3((float)sx, (float)sy, (float)sz);
            validCount++;
        }
    }
    if (validCount == 0) return false;
    centroid = Math::Vector3(centroid) / (float)validCount;   // 셀 좌표 평균
    

    // --- 2. 무게중심 셀에서 A* (경로 대표성: 그룹 한가운데를 관통) ---
    int cx = (int)std::round(centroid.GetX());
    int cy = (int)std::round(centroid.GetY());
    int cz = (int)std::round(centroid.GetZ());
    int asx, asy, asz;
    Math::Vector3 centroidWorld = m_Grid->GetWorldPos(cx, cy, cz);
    if (!m_Grid->FindNearestWalkable(centroidWorld, asx, asy, asz)) return false;

    std::vector<DirectX::XMINT3> path;
    if (!m_Pathfinder.FindPath(*m_Grid, { asx,asy,asz }, goalCell, path)) return false;
    if (outPath) *outPath = path;

    // --- 3. margin을 "그룹 분포 반경"으로 계산 (시드 간 공백 방지) ---
    // 무게중심에서 가장 먼 NPC까지의 셀 거리 → 그만큼 margin을 확보해야
    // 가장자리 NPC 시드와 경로 시드가 마스크 안에서 이어짐.
    int maxDistFromCentroid = 0;
    for (const auto& c : m_StartCells)
    {
        if (c.x < 0) continue;
        int dx = c.x - cx, dz = c.z - cz;
        int d = (int)std::round(std::sqrt((float)(dx * dx + dz * dz)));
        maxDistFromCentroid = std::max(maxDistFromCentroid, d);
    }
    // 대형 폭(ComputeMarginCells)과 분포 반경 중 큰 값 사용 + 약간의 버퍼
    int formationMargin = ComputeMarginCells((int)n);
    int margin = std::max(formationMargin, maxDistFromCentroid) + 2;

    // --- 4. 마스크: A* 경로 + 모든 NPC 시작 셀을 시드로 ---
    auto mask = BuildLayerMask(*m_Grid, path, m_StartCells, margin);

    // --- 5. FlowField 계산 ---
    m_CorridorField.Build(*m_Grid, goalCell, mask);

    // 무게중심 월드 좌표 (이미 계산된 centroid 셀 좌표 활용)
    Math::Vector3 centroidWorld2 = m_Grid->GetWorldPos(cx, cy, cz);
    BuildArrivalRegion(goalCell, (int)n, centroidWorld2);

    m_HasGoal = true;
    InitGroupMovement();   // m_StartCells 재사용
    return true;
}


void NpcManager::Update(float dt)
{
    if (false == m_HasGoal) return;

    const float NPC_SPEED = 3.0f;
    const float ARRIVE_EPS_SQ = 0.05f * 0.05f;
    const size_t n = m_Move.count;

    bool anyMoved = false;
    bool anyActive = false;

    for (size_t i = 0; i < n; ++i)
    {
        if (!m_Move.active[i]) continue;
        anyActive = true;       // 살아있는지 체크

        Math::Vector3 pos = m_Move.position[i];
        Math::Vector3 tgt = m_Move.targetWorldPos[i];
        Math::Vector3 delta = tgt - pos;

        if (Math::LengthSquare(delta) < ARRIVE_EPS_SQ)
        {
            AdvanceCell(i);   // 도착 -> 셀 전환
        }
        else
        {
            Math::Vector3 d = Math::Normalize(delta);
            m_Move.position[i] = pos + d * (NPC_SPEED * dt);
        }
        anyMoved = true;
    }

    if (true == anyMoved)
    {
        SyncInstances();
        NpcRenderer::UpdateInstances(m_NpcInstances);
    }

    // 전원 도착시 목표 종료
    if (false == anyActive)
    {
        m_HasGoal = false;
    }
}

NpcManager::SlotState NpcManager::GetSlotState(int slotIdx) const
{
    // 이 슬롯을 청구한 NPC를 찾는다 (선형 탐색 — 시각화용이라 매 프레임 아님)
    for (size_t i = 0; i < m_Move.count; ++i)
    {
        if (m_Move.claimedSlot[i] != slotIdx) continue;

        const auto& slot = m_Arrival.slots[slotIdx];
        bool arrived = (m_Move.currCell[i] == slot) && (m_Move.active[i] == 0);
        return arrived ? SlotState::Arrived : SlotState::Claimed;
    }
    return SlotState::Unclaimed;
}

bool NpcManager::TryAdvanceTo(size_t i, const DirectX::XMINT3& next)
{
    const auto& curr = m_Move.currCell[i];
    if (true == IsMoveCross(i, curr, next))  return false;           // 다른 npc와 크로스함
    if (false == m_Reserve.TryReserve(MakeCellKey(next), (int)i))    // 다른 npc가 이미 점유
        return false;

    // 기존 셀 예약 즉시 해제
    m_Reserve.Release(MakeCellKey(curr));

    m_Move.targetCell[i] = next;
    m_Move.targetWorldPos[i] = GetNpcStandPos(next, m_Move.halfHeight[i]);
    return true;
}

bool NpcManager::IsMoveCross(size_t i, const DirectX::XMINT3& curr, const DirectX::XMINT3& next) const
{
    int dx = next.x - curr.x;
    int dz = next.z - curr.z;
    if (dx == 0 || dz == 0)  return false;

    const int sx[2] = { curr.x + dx, curr.x };
    const int sz[2] = { curr.z,      curr.z + dz };
    const int ys[2] = { curr.y,      next.y };

    for (int a = 0; a < 2; ++a)
    {
        int b = 1 - a;
        for (int t = 0; t < 2; ++t)
        {
            int occ = m_Reserve.Find(MakeCellKey(sx[a], ys[t], sz[a]));
            if (occ < 0 || occ == (int)i)    continue;

            const auto& tc = m_Move.targetCell[occ];
            if (tc.x == sx[b] && tc.z == sz[b])  return true;
        }
    }
    return false;
}

void NpcManager::InitGroupMovement()
{
    const size_t n = m_NpcInstances.size();
    m_Move.Resize(n);
    m_Reserve.Reset(n);

    for (size_t i = 0; i < n; ++i)
    {
        m_Move.halfHeight[i] = m_NpcInstances[i].scaleY;

        const auto& startCell = m_StartCells[i];   // FindNearestWalkable 재호출 안 함
        if (startCell.x < 0)
        { 
            m_Move.active[i] = 0; 
            continue; 
        }

        if (false == m_Reserve.TryReserve(MakeCellKey(startCell), (int)i))
        {
            m_Move.active[i] = 0;   // 두 npc가 같은 시작 셀로 수렴한 경우
            continue;
        }

        m_Move.currCell[i] = startCell;
        m_Move.position[i] = GetNpcStandPos(startCell, m_Move.halfHeight[i]);

        // 첫 목표는 advanceCell에서
        m_Move.targetCell[i] = startCell;
        m_Move.targetWorldPos[i] = m_Move.position[i];
        m_Move.active[i] = 1;
    }

    // flowfield cost가 작은곳에 있는 npc를 우선순위화
    m_MoveOrder.clear();
    m_MoveOrder.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
        if (m_Move.active[i])    m_MoveOrder.push_back((int)i);
    }

    // 각각 npc의 서있는 비용 넣기
    std::vector<float> cost(n, FLT_MAX);
    for (int i : m_MoveOrder)
    {
        const auto& c = m_Move.currCell[i];
        float v;
        if (m_CorridorField.SampleCost(*m_Grid, c.x, c.y, c.z, v))   cost[i] = v;
    }
    // 비용이 작은대로 오름차순
    std::sort(m_MoveOrder.begin(), m_MoveOrder.end(), [&](int a, int b) {
        return cost[a] < cost[b];
        });
}


void NpcManager::AdvanceCell(size_t i)
{
    // 목표 셀에 정확히 스냅 (연속 이동 누적 오차 제거 + 층 플립 방지)
    m_Move.position[i] = m_Move.targetWorldPos[i];
    m_Move.currCell[i] = m_Move.targetCell[i];

    const auto& curr = m_Move.currCell[i];

    // 도착영역 첫 진입시, 슬롯을 청구
    if (m_Move.claimedSlot[i] < 0 && m_Arrival.valid)
    {
        if (m_Arrival.slotKeys.count(MakeCellKey(curr)) && m_Arrival.nextSlot < (int)m_Arrival.slots.size())
        {
            m_Move.claimedSlot[i] = m_Arrival.nextSlot++;
        }
    }
    DirectX::XMINT3 next;
    bool hasNext = false;
    if (m_Move.claimedSlot[i] >= 0)
    {
        const auto& goal = m_Arrival.slots[m_Move.claimedSlot[i]];
        // 자기 슬롯 도달
        if (curr == goal)
        {
            m_Move.active[i] = 0; 
            return;
        }
        hasNext = StepTowardSlot(i, next);  // false면 갇힘 상태
    }

    // 아니면 이동
    else
    {
        DirectX::XMFLOAT3 dir;
        if (m_CorridorField.SampleDirection(*m_Grid, curr.x, curr.y, curr.z, dir))
        {
            if (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z >= 1e-6f)
            {
                next = { curr.x + (int)std::round(dir.x),
                         curr.y + (int)std::round(dir.y),
                         curr.z + (int)std::round(dir.z) };
                hasNext = true;
               
            }
        }
    }

    if (false == hasNext)
    {
        m_Move.active[i] = 0;
        return;
    }

    // 막혀서 제자리 대기
    if (false == TryAdvanceTo(i, next))
    {
        m_Move.targetCell[i] = curr;
        m_Move.targetWorldPos[i] = m_Move.position[i];
    }

}

void NpcManager::SyncInstances()
{
    for (size_t i = 0; i < m_Move.count; ++i)
    {
        m_NpcInstances[i].position[0] = m_Move.position[i].GetX();
        m_NpcInstances[i].position[1] = m_Move.position[i].GetY();
        m_NpcInstances[i].position[2] = m_Move.position[i].GetZ();
    }
}

Math::Vector3 NpcManager::GetNpcStandPos(const DirectX::XMINT3& cell, float halfHeight) const
{
    Math::Vector3 cellCenter = m_Grid->GetWorldPos(cell.x, cell.y, cell.z);
    float cellSize = m_Grid->GetCellSize();

    // Startup()의 배치 공식과 동일: 셀 중심 -> 셀 윗면(절반) -> NPC 반높이만큼 더 위로
    float standY = cellCenter.GetY() + (cellSize * 0.5f) + halfHeight + 0.1f;

    return Math::Vector3(cellCenter.GetX(), standY, cellCenter.GetZ());
}

void NpcManager::BuildArrivalRegion(const DirectX::XMINT3& goal, int npcCount, const Math::Vector3& groupCenter)
{
    m_Arrival.Clear();
    constexpr float MARGIN_NPC = 1.0f;

    const int target = static_cast<int>(npcCount * MARGIN_NPC) + 1;

    // BFS - 슬롯과 홉수 수집
    struct Entry
    {
        DirectX::XMINT3 cell;
        int hop;
    };
    std::vector<Entry> collected;
    std::queue<Entry> q;
    std::unordered_set<int64_t> visited;

    collected.reserve(target);
    q.push({ goal, 0 });
    visited.insert(MakeCellKey(goal));


    // bfs로 주변 갈수 있는 복셀 추가
    std::vector<NeighborInfo> neighbors;
    while (!q.empty() && (int)collected.size() < target)
    {
        Entry curr = q.front(); q.pop();
        collected.push_back(curr);

        GetWalkableNeighbors(*m_Grid, curr.cell, neighbors);
        for (const auto& n : neighbors)
        {
            int64_t key = MakeCellKey(n.pos);
            if (visited.insert(key).second)
            {
                q.push({ n.pos, curr.hop + 1 });
            }
        }
    }

    if (collected.empty())   return;

    // bfs로 찾은 범위에 대한 순서 정하기
    Math::Vector3 goalPos = m_Grid->GetWorldPos(goal.x, goal.y, goal.z);
    // TODO - 그룹이 어디 방향에서 오는지 정의인데, 현재는 하나의 방향으로만 정함
    float ax = goalPos.GetX() - groupCenter.GetX();
    float az = goalPos.GetZ() - groupCenter.GetZ();
    float len = std::sqrt(ax * ax + az * az);
    bool hasAxis = (len > 1e-3f);   // 근거리 확인용
    if (hasAxis)
    {
        ax /= len;
        az /= len;
    }

    std::stable_sort(collected.begin(), collected.end(),
        [&](const Entry& a, const Entry& b)
        {
            if (a.hop != b.hop) return a.hop < b.hop;       // 1차 - 목적지에 가까운거부터
            if (false == hasAxis) return false;

            // 축에다 위치 투영해서
            Math::Vector3 pa = m_Grid->GetWorldPos(a.cell.x, a.cell.y, a.cell.z);
            Math::Vector3 pb = m_Grid->GetWorldPos(b.cell.x, b.cell.y, b.cell.z);
            float da = (pa.GetX() - goalPos.GetX()) * ax + (pa.GetZ() - goalPos.GetZ()) * az;
            float db = (pb.GetX() - goalPos.GetX()) * ax + (pb.GetZ() - goalPos.GetZ()) * az;
            return da < db; // 투영값 작은거(진입방향) 우선순위
        });
    
    m_Arrival.slots.reserve(collected.size());
    for (const auto& e : collected)
    {
        m_Arrival.slots.push_back(e.cell);
        m_Arrival.slotKeys.insert(MakeCellKey(e.cell));
    }
    m_Arrival.valid = true;

}

bool NpcManager::StepTowardSlot(size_t i, DirectX::XMINT3& outNext) const
{
    const auto& curr = m_Move.currCell[i];
    const auto& goal = m_Arrival.slots[m_Move.claimedSlot[i]];
    
    auto dist2 = [](const DirectX::XMINT3& a, const DirectX::XMINT3& b)
    {
        float dx = (float)(a.x - b.x);
        float dy = (float)(a.y - b.y);
        float dz = (float)(a.z - b.z);
        return dx * dx + dy * dy + dz * dz;
    };

    std::vector<NeighborInfo> neighbors;
    GetWalkableNeighbors(*m_Grid, curr, neighbors);

    float best = dist2(curr, goal);
    bool found = false;
    for (const auto& n : neighbors)
    {
        float d = dist2(n.pos, goal);
        if (d < best)
        {
            best = d;
            outNext = n.pos;
            found = true;
        }
    }
    return found;
}

