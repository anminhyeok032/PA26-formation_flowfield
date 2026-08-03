#include "NpcManager.h"
#include "ChunkKey.h"
#include "GridNeighbors.h"
#include <cmath>
#include <queue>

constexpr float NPC_SPEED = 3.5f;

const int TARGET_COUNT = 1000;

NpcManager::NpcManager()
    : m_MovementSolver(m_Move, m_Reserve)
    , m_SplitController(m_Leaves, m_Move, m_Group, m_StartCells)
{
}

void NpcManager::Init(const VoxelGrid& grid, const ChunkGraph& chunkgraph)
{
    m_Grid = &grid;
    m_ChunkGraph = &chunkgraph;
    const float voxelSize = grid.GetCellSize();
    const float NPC_WIDTH = voxelSize / 2.5f;
    const float NPC_HEIGHT = voxelSize * 3.0f / 2.0f;

    m_NpcInstances.clear();

    //m_NpcInstances.reserve(TARGET_COUNT);

    const int baseX = 100;
    const int baseZ = 100;

    // 시작 셀이 자체적으로 걸을 수 있는 표면인지 확인 후, 그 y를 구함
    VoxelGrid::SurfaceSpan startSurfaces = grid.GetSurfaceYList(baseX, baseZ);
    if (startSurfaces.count == 0)
    {
        // 시작점 자체가 허공/막힘이면 배치 불가 - 조용히 종료
        NpcRenderer::UpdateInstances(m_NpcInstances);
        return;
    }
    int startY = startSurfaces.data[0];   // 여러 표면이면 가장 아래(첫 번째)를 기준으로

    // BFS: 시작 셀에서 실제로 걸어서(GetWalkableNeighbors) 도달 가능한 컬럼만 방문.
    // 컬럼(x,z) 단위로 중복 방문 방지 (같은 컬럼의 다른 y는 배치 목적상 굳이 재방문 안 함).
    std::unordered_set<int64_t> visitedColumns;
    std::queue<DirectX::XMINT3> q;

    DirectX::XMINT3 startCell{ baseX, startY, baseZ };
    visitedColumns.insert(MakeCellKey(baseX, 0, baseZ));   // y는 0 고정 - 컬럼 단위 방문 표시용
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

        // 실제로 걸어서 갈 수 있는 이웃만 확장 (연결성 보장 - 고립 지형 배제)
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

    // head == leaf[0]
    m_Leaves.clear();
    m_Leaves.push_back(std::make_unique<LeafGroup>());
    m_Leaves[0]->leafId = 0;
    m_Leaves[0]->parentId = 0;
    m_Leaves[0]->active = false;
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

    // 맞았으면 그룹 선택 토글 (누구를 맞췄는지는 무관 - 그룹 전체가 한 단위)
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
    const size_t n = m_NpcInstances.size();
    if (n == 0) return false;

    // --- 1. 이전 분리 상태 초기화 - 새 목적지는 항상 단일 그룹에서 시작 ---
    for (auto& leaf : m_Leaves) leaf->Reset();
    m_Leaves.resize(1);
    if (!m_Leaves[0]) m_Leaves[0] = std::make_unique<LeafGroup>();   // resize가 만든 빈 슬롯 방어
    m_Group.Reset();

    LeafGroup& leaf = *m_Leaves[0];

    // --- 2. 각 NPC의 시작 셀 수집 (마스크 시드 + 이동 초기화에 공유) ---
    // FindNearestWalkable을 여기서 1회만 돌리고 InitGroupMovement가 재사용
    m_StartCells.clear();
    m_StartCells.resize(n, { -1, -1, -1 });

    const bool hasPrevMove = (m_Move.size() > 0);
    for (size_t i = 0; i < n; ++i)
    {
        // 이전에 움직이던 중이면 예약된 목표 셀을 그대로 시작점으로
        if (hasPrevMove)
        {
            const auto& tc = m_Move.targetCell[i];
            if (m_Reserve.Find(MakeCellKey(tc)) == (int)i)
            {
                m_StartCells[i] = tc;
                continue;
            }
        }

        // 아니면 렌더 좌표에서 역산
        const auto& inst = m_NpcInstances[i];
        Math::Vector3 p(inst.position[0], inst.position[1], inst.position[2]);
        int sx, sy, sz;
        if (m_Grid->FindNearestWalkable(p, sx, sy, sz))
        {
            m_StartCells[i] = { sx, sy, sz };
        }
    }

    InitGroupMovement();   // m_StartCells 재사용

    // --- 3. 루트 leaf 구성 - 전원이 leaf 0 소속 ---
    leaf.leafId = 0;
    leaf.parentId = m_Group.groupId;
    leaf.depth = 0;
    leaf.members.reserve(m_Move.size());
    for (size_t i = 0; i < m_Move.size(); ++i)
    {
        m_Move.leafId[i] = 0;
        leaf.members.push_back((int)i);
    }

    // --- 4. 청크 경로 -> 마스크 -> FlowField ---
    std::vector<uint32_t> nodePath;
    if (!m_SplitController.BuildLeafCorridor(*m_Grid, *m_ChunkGraph, leaf, goalCell, &nodePath))  return false;

    // --- 5. 마스크가 실제 통로를 담았는지 확인 ---
    // 청크 그래프는 청크 내부가 벽/절벽으로 갈라진 경우를 모른다.
    // 조용히 깨지지 않도록 여기서 잡는다. 뜨기 시작하면 margin 확대나 청크 세분화를 검토.
    for (size_t i = 0; i < n; ++i)
    {
        const auto& c = m_StartCells[i];
        if (c.x < 0)    continue;
        if (!leaf.field.IsVisited(*m_Grid, c.x, c.y, c.z))
        {
            DEBUGPRINT("ChunkGraph: mask miss - npc %zu at (%d,%d,%d)\n", i, c.x, c.y, c.z);
            break;   // 한 번만 알리면 충분
        }
    }

    leaf.active = true;

    m_Group.goal = goalCell;
    m_Group.hasGoal = true;
    m_Group.leafIds.clear();
    m_Group.leafIds.push_back(0);

    // --- 6. 디버그 시각화용 셀 변환 ---
    if (outPath) m_ChunkGraph->NodePathToCells(*m_Grid, leaf.field, nodePath, *outPath);

    return true;
}


void NpcManager::Update(float dt)
{
    if (false == m_Group.hasGoal) return;

    const float ARRIVE_EPS_SQ = 0.05f * 0.05f;
    const size_t n = m_Move.size();

    bool anyMoved = false;
    bool anyActive = false;

    for (int i : m_MoveOrder)
    {
        if (!m_Move.active[i]) continue;
        anyActive = true;       // 살아있는지 체크

        Math::Vector3 pos = m_Move.position[i];
        Math::Vector3 tgt = m_Move.targetWorldPos[i];
        Math::Vector3 delta = tgt - pos;

        if (Math::LengthSquare(delta) < ARRIVE_EPS_SQ)
        {

            m_MovementSolver.AdvanceCell(*m_Grid, m_Leaves, i, dt);   // 도착 -> 셀 전환
        }
        else
        {
            float distSq = Math::LengthSquare(delta);
            float step = NPC_SPEED * dt;

            if (step * step >= distSq)
            {
                // 이번 프레임 이동량이 남은 거리 이상 -> 목표에 정확히 스냅 (오버슈트 방지)
                m_Move.position[i] = tgt;
            }
            else
            {
                Math::Vector3 d = Math::Normalize(delta);
                m_Move.position[i] = pos + d * step;
            }
        }
        anyMoved = true;
    }

    for (auto& leaf : m_Leaves)
    {
        if (false == leaf->active)    continue;
        bool allDone = true;
        for (int idx : leaf->members)
        {
            if (m_Move.active[idx] != 0)
            {
                allDone = false;
                break;
            }
        }
        // 필드는 leaf 해제시 진행
        if (true == allDone)
        {
            leaf->active = false;
        }
    }

    for (const auto& leaf : m_Leaves)
    {
        if (true == leaf->active)
        {
            anyActive = true;
            break;
        }
    }
    // 전원 도착시 목표 종료
    if (false == anyActive)
    {
        m_Group.hasGoal = false;
    }

    // m_SplitController.CheckSplitTriggers(*m_Grid);   // 루프 종료 후 - 여기서 leaf가 늘어날 수 있음

    // 누군가 움직였을때만 이동
    if (true == anyMoved)
    {
        SyncInstances();
        NpcRenderer::UpdateInstances(m_NpcInstances);
    }
}

bool NpcManager::IsVisitedAny(const VoxelGrid& grid, int x, int y, int z) const
{
    for (const auto& leafPtr : m_Leaves)
    {
        if (leafPtr->field.IsVisited(grid, x, y, z)) return true;
    }
    return false;
}

bool NpcManager::SampleDirectionAny(const VoxelGrid& grid, int x, int y, int z, DirectX::XMINT3& outDir) const
{
    for (const auto& leafPtr : m_Leaves)
    {
        if (leafPtr->field.SampleDirection(grid, x, y, z, outDir))   return true;
    }
    return false;
}

void NpcManager::OnTerrainChanged(const std::vector<DirectX::XMINT3>& editedCells)
{
    if (!m_Group.hasGoal || editedCells.empty()) return;

    // 영향 청크 = 편집 청크 + 8이웃.
    // 편집 셀 바로 옆(다른 청크)의 셀은 이제 방향이 벽을 가리키므로 이웃까지 포함해야 한다.
    std::unordered_set<int64_t> affected;
    for (const auto& c : editedCells)
    {
        const int ccx = c.x / ChunkGraph::CHUNK_SIZE;
        const int ccz = c.z / ChunkGraph::CHUNK_SIZE;
        for (int dz = -1; dz <= 1; ++dz)
            for (int dx = -1; dx <= 1; ++dx)
                affected.insert(MakeChunkKey(ccx + dx, 0, ccz + dz));
    }

    // NPC가 새로 막힌 셀 위에 서 있을 수 있으므로 현재 위치를 다시 스냅한다.
    // m_StartCells는 SetGroupDestination 시점 값이라 그대로 쓰면 옛 좌표로 필드를 만든다.
    const size_t n = m_Move.size();
    for (size_t i = 0; i < n; ++i)
    {
        const auto& curr = m_Move.currCell[i];
        if (curr.x < 0) { m_StartCells[i] = { -1,-1,-1 }; continue; }

        if (m_Grid->IsWalkable(curr.x, curr.y, curr.z))
        {
            m_StartCells[i] = curr;
            continue;
        }
        // 서 있던 자리가 막혔다 - 가장 가까운 walkable로 밀어낸다
        int sx, sy, sz;
        Math::Vector3 p = m_Grid->GetWorldPos(curr.x, curr.y, curr.z);
        m_StartCells[i] = m_Grid->FindNearestWalkable(p, sx, sy, sz)
            ? DirectX::XMINT3{ sx, sy, sz } : DirectX::XMINT3{ -1,-1,-1 };
    }

    for (auto& leafPtr : m_Leaves)
    {
        LeafGroup& leaf = *leafPtr;
        if (!leaf.active || leaf.members.empty()) continue;

        // 이 필드가 영향 청크를 덮고 있었나 - LeafGroup에 별도 상태를 두지 않아도
        // CorridorFlowField가 이미 자기 청크 목록을 들고 있다
        bool overlaps = false;
        for (const auto& entry : leaf.field.GetChunks())
        {
            if (affected.count(entry.first) > 0) { overlaps = true; break; }
        }
        if (!overlaps) continue;   // 안 겹치면 그 필드는 여전히 유효하다

        if (!m_SplitController.BuildLeafCorridor(*m_Grid, *m_ChunkGraph, leaf, m_Group.goal, nullptr))
        {
            // 목적지로 가는 길이 완전히 막혔다 - 이 leaf는 정지
            leaf.active = false;
            for (int idx : leaf.members)
            {
                m_Move.active[idx] = 0;
                m_Move.stopReason[idx] = 1;
            }
        }
    }
}


void NpcManager::InitGroupMovement()
{
    const size_t n = m_NpcInstances.size();
    const bool firstTimeMove = (m_Move.size() == 0);   // Resize 전에 움직인적 있는지 판별

    m_Move.Resize(n);
    m_Reserve.Reset(n);

    for (size_t i = 0; i < n; ++i)
    {
        m_Move.halfHeight[i] = m_NpcInstances[i].scaleY;

        // 최초 호출: m_Move.position이 아직 유효값 없음(0벡터) -> 실제 배치 위치로 시드
        if (firstTimeMove)
        {
            const auto& inst = m_NpcInstances[i];
            m_Move.position[i] = Math::Vector3(inst.position[0], inst.position[1], inst.position[2]);
        }
        // 두 번째

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
        m_Move.targetCell[i] = startCell;
        m_Move.targetWorldPos[i] = GetNpcStandPos(startCell, m_Move.halfHeight[i]);
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
        if (m_Leaves[0]->field.SampleCost(*m_Grid, c.x, c.y, c.z, v))   cost[i] = v;
    }
    // 비용이 작은대로 오름차순 (거리 가까운게 그룹의 앞이 되도록)
    std::sort(m_MoveOrder.begin(), m_MoveOrder.end(), [&](int a, int b) {
        return cost[a] < cost[b];
        });
}

void NpcManager::SyncInstances()
{
    for (size_t i = 0; i < m_Move.size(); ++i)
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
