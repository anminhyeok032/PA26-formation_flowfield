#include "NpcManager.h"
#include "ChunkKey.h"
#include "GridNeighbors.h"
#include <cmath>
#include <queue>
#include "NpcConstants.h"

NpcManager::NpcManager()
    : m_MovementSolver(m_Move, m_Reserve)
    , m_SplitController(m_Leaves, m_Move, m_Group, m_StartCells)
{
}

void NpcManager::Init(const VoxelGrid& grid, const ChunkGraph& chunkgraph, const DirectX::XMINT3& playerStartCell)
{
    m_Grid = &grid;
    m_ChunkGraph = &chunkgraph;
    const float voxelSize = grid.GetCellSize();
    const float NPC_WIDTH = voxelSize / 2.5f;
    const float NPC_HEIGHT = voxelSize * 3.0f / 2.0f;

    m_NpcInstances.clear();
    m_BehaviorGroups.clear();

    // TODO : 생성 임시 시드화 - 추후 I/O를 이용해서 따로 저장해 읽어오도록 교체할것
     // ---------- 1. 다중 시드 스폰 ----------
    std::unordered_set<int64_t> usedColumns;
    std::vector<DirectX::XMINT3> seedCells;
    uint32_t rng = 0x9E3779B9u;   // 고정 시드 - 재현 가능한 배치

    const int sizeX = grid.GetSizeX();
    const int sizeZ = grid.GetSizeZ();

    for (int i = 0; i < SPAWN_SEED_ATTEMPTS && (int)m_NpcInstances.size() < TARGET_COUNT; ++i)
    {
        const int sx = (int)(NextRand(rng) % (uint32_t)sizeX);
        const int sz = (int)(NextRand(rng) % (uint32_t)sizeZ);

        // 플레이어 주변이면 스킵
        const int pdx = sx - playerStartCell.x, pdz = sz - playerStartCell.z;
        if (pdx * pdx + pdz * pdz < SPAWN_PLAYER_EXCLUSION_CELLS * SPAWN_PLAYER_EXCLUSION_CELLS)
            continue;


        // 그 컬럼에서 실제로 설 수 있는 가장 아래 표면
        VoxelGrid::SurfaceSpan surf = grid.GetSurfaceYList(sx, sz);
        int sy = -1;
        for (int k = 0; k < surf.count; ++k)
        {
            if (grid.IsWalkable(sx, surf.data[k], sz)) { sy = surf.data[k]; break; }
        }
        if (sy < 0) continue;

        // 기존 시드와 너무 가까우면 버린다 - 그룹이 뭉치면 산개 자체가 성립x
        bool tooClose = false;
        for (const auto& s : seedCells)
        {
            const int dx = s.x - sx, dz = s.z - sz;
            if (dx * dx + dz * dz < SPAWN_MIN_SEED_DIST * SPAWN_MIN_SEED_DIST)
            {
                tooClose = true;
                break;
            }
        }
        if (tooClose) continue;

        // 그룹 크기 랜덤 
        int roll = (int)(NextRand(rng) % 100u), want = GROUP_SIZE_BUCKETS[0], acc = 0;
        for (int b = 0; b < 4; ++b)
        {
            acc += GROUP_SIZE_WEIGHTS[b];
            if (roll < acc) { want = GROUP_SIZE_BUCKETS[b]; break; }
        }
        const int remain = TARGET_COUNT - (int)m_NpcInstances.size();
        if (want > remain) want = remain;

        if (TrySpawnGroup(grid, sx, sy, sz, want, NPC_WIDTH, NPC_HEIGHT, usedColumns))
            seedCells.push_back({ sx, sy, sz });
    }

    // ---------- 2. 이동 데이터 초기화 ----------
    const size_t n = m_NpcInstances.size();
    m_Move.Resize(n);        
    m_Reserve.Reset(n);      
    m_StartCells.assign(n, { -1, -1, -1 });   // SplitController가 참조로 들고 있어 크기 유지 필요

    for (size_t i = 0; i < n; ++i)
    {
        const auto& inst = m_NpcInstances[i];
        m_Move.halfHeight[i] = inst.scaleY;
        m_Move.position[i] = Math::Vector3(inst.position[0], inst.position[1], inst.position[2]);

        int cx, cy, cz;
        if (!grid.FindNearestWalkable(m_Move.position[i], cx, cy, cz))
        {
            m_Move.active[i] = 0;
            continue;   // 셀이 없으면 state는 Idle이지만 currCell.x < 0이라 모든 루프가 걸러낸다
        }

        const DirectX::XMINT3 cell{ cx, cy, cz };
        if (!m_Reserve.TryReserve(MakeCellKey(cell), (int)i))
        {
            m_Move.active[i] = 0;
            continue;   // 두 NPC가 같은 셀로 수렴 - 드물지만 컬럼 중복 방지가 y까지는 못 막는다
        }

        m_Move.currCell[i] = cell;
        m_Move.targetCell[i] = cell;
        m_Move.anchorCell[i] = cell;   // 배회 중심 = 복귀 목표
        m_Move.targetWorldPos[i] = GetNpcStandPos(cell, m_Move.halfHeight[i]);
        m_Move.state[i] = NPC_STATE_IDLE;
        m_Move.active[i] = 0;      // active는 Chase 전용 - Idle/Lost는 state로만 판단
        m_StartCells[i] = cell;

        // 첫 걸음 시각을 흩뿌린다. 전원이 같은 프레임에 움직이기 시작하면 기계적으로 보인다
        const float t = (float)(NextRand(m_Move.noiseSeed[i]) & 0xFFFF) / 65535.0f;
        m_Move.stateTimer[i] = WANDER_PAUSE_MIN_SEC
            + t * (WANDER_PAUSE_MAX_SEC - WANDER_PAUSE_MIN_SEC);
    }

    // ---------- 3. 순회 순서 ----------
    // 전원 포함. Idle도 매 프레임 배회 판정을 받아야 한다.
    // 기존의 cost 오름차순 정렬은 제거 - 필드가 비어 있는 시점에 SampleCost를 부르므로
    // cost가 전부 FLT_MAX였고 비교자가 항상 false라 처음부터 무효였다
    m_MoveOrder.clear();
    m_MoveOrder.reserve(n);
    for (size_t i = 0; i < n; ++i) m_MoveOrder.push_back((int)i);

    // ---------- 4. 추격 필드 ----------
    // leaf는 하나만 유지한다. FieldWorker의 m_Pending/m_Ready가 단일 슬롯이라
    // leaf가 둘 이상이면 요청이 조용히 유실된다
    m_Leaves.clear();
    m_Leaves.push_back(std::make_unique<LeafGroup>());
    m_Leaves[0]->leafId = 0;
    m_Leaves[0]->parentId = 0;
    m_Leaves[0]->active = false;

    NpcRenderer::UpdateInstances(m_NpcInstances);

    // 워커는 grid/chunkGraph를 참조로만 들고 있으므로 포인터 확정 후 기동
    m_FieldWorker.Start(grid, chunkgraph);

    // 리전 버킷 배정
    m_RegionCountX = (grid.GetSizeX() + REGION_SIZE_CELLS - 1) / REGION_SIZE_CELLS;
    m_RegionCountZ = (grid.GetSizeZ() + REGION_SIZE_CELLS - 1) / REGION_SIZE_CELLS;
    m_Regions.assign((size_t)m_RegionCountX* m_RegionCountZ, RegionCell{});

    m_GroupOrder.clear();
    m_GroupSlot.assign(m_BehaviorGroups.size(), -1);

    for (size_t i = 0; i < m_BehaviorGroups.size(); ++i)
    {
        auto& group = m_BehaviorGroups[i];
        const int cx = (group.aabbMin.x + group.aabbMax.x) / 2;
        const int cz = (group.aabbMin.z + group.aabbMax.z) / 2;
        const int rx = std::min(std::max(cx / REGION_SIZE_CELLS, 0), m_RegionCountX - 1);
        const int rz = std::min(std::max(cz / REGION_SIZE_CELLS, 0), m_RegionCountZ - 1);

        group.regionIndex = rz * m_RegionCountX + rx;
        m_Regions[group.regionIndex].groupIndices.push_back((int)i);

        m_GroupSlot[i] = (int)m_GroupOrder.size();
        m_GroupOrder.push_back((int)i);
    }
    m_ActiveGroupCount = 0;

}

bool NpcManager::TrySpawnGroup(const VoxelGrid& grid, int seedX, int seedY, int seedZ,
    int wantCount, float npcWidth, float npcHeight,
    std::unordered_set<int64_t>& usedColumns)
{
    if (wantCount <= 0) return false;

    const float voxelSize = grid.GetCellSize();
    const int   R2 = SPAWN_MIN_SEED_DIST * SPAWN_MIN_SEED_DIST;

    // 시드 컬럼이 이미 다른 그룹에 쓰였으면 포기 - 그룹 간 중복 배치 방지
    if (!usedColumns.insert(MakeCellKey(seedX, 0, seedZ)).second) return false;

    BehaviorGroup grp;
    grp.aabbMin = { seedX, seedY, seedZ };
    grp.aabbMax = { seedX, seedY, seedZ };

    std::queue<DirectX::XMINT3> q;
    q.push({ seedX, seedY, seedZ });

    std::vector<NeighborInfo> neighbors;
    int placed = 0;

    while (!q.empty() && placed < wantCount)
    {
        const DirectX::XMINT3 cur = q.front();
        q.pop();

        // --- 배치 ---
        NpcRenderer::InstanceData inst = {};
        inst.scaleXZ = npcWidth;
        inst.scaleY = npcHeight;
        inst.position[0] = cur.x * voxelSize;
        inst.position[1] = (float)cur.y * voxelSize + (voxelSize / 2.0f) + inst.scaleY + 0.1f;
        inst.position[2] = cur.z * voxelSize;
        inst.colorType = 0;

        grp.members.push_back((int)m_NpcInstances.size());
        m_NpcInstances.push_back(inst);
        ++placed;

        if (cur.x < grp.aabbMin.x) grp.aabbMin.x = cur.x;
        if (cur.z < grp.aabbMin.z) grp.aabbMin.z = cur.z;
        if (cur.x > grp.aabbMax.x) grp.aabbMax.x = cur.x;
        if (cur.z > grp.aabbMax.z) grp.aabbMax.z = cur.z;

        if (placed >= wantCount) break;

        // --- 확장 ---
        // 실제로 걸어서 갈 수 있는 이웃만 - 고립 지형에 뿌리지 않기 위해
        GetWalkableNeighbors(grid, cur, neighbors);
        for (const auto& n : neighbors)
        {
            // 시드에서 너무 멀어지면 확장하지 않는다.
            // 좁은 통로를 따라 한 줄로 늘어지면 AABB가 터져 브로드페이즈가 무의미해진다
            const int dx = n.pos.x - seedX, dz = n.pos.z - seedZ;
            if (dx * dx + dz * dz > R2) continue;

            // 컬럼 단위 중복 방지 - 같은 컬럼의 다른 y는 배치 목적상 재방문 불필요
            if (!usedColumns.insert(MakeCellKey(n.pos.x, 0, n.pos.z)).second) continue;
            q.push(n.pos);
        }
    }

    if (grp.members.empty()) return false;

    // 배회 반경만큼 여유. 배회가 이 반경을 못 벗어나므로 이후 AABB 갱신이 불필요하다
    grp.aabbMin.x -= WANDER_RADIUS_CELLS; grp.aabbMin.z -= WANDER_RADIUS_CELLS;
    grp.aabbMax.x += WANDER_RADIUS_CELLS; grp.aabbMax.z += WANDER_RADIUS_CELLS;

    m_BehaviorGroups.push_back(std::move(grp));
    return true;
}

void NpcManager::ActivateGroup(int g)
{
    const int slot = m_GroupSlot[g];
    if (slot < m_ActiveGroupCount)   return;    // 이미 활성화됨

    const int boundary = m_ActiveGroupCount;
    std::swap(m_GroupOrder[slot], m_GroupOrder[boundary]);
    m_GroupSlot[m_GroupOrder[slot]] = slot;
    m_GroupSlot[m_GroupOrder[boundary]] = boundary;
    ++m_ActiveGroupCount;
}

void NpcManager::DeactivateGroup(int g)
{
    const int slot = m_GroupSlot[g];
    if (slot >= m_ActiveGroupCount)  return;    // sleep 상태

    // 잠들기 전 셀 스냅
    for (int i : m_BehaviorGroups[g].members)
    {
        m_Move.position[i] = m_Move.targetWorldPos[i];
        m_Move.currCell[i] = m_Move.targetCell[i];

        // 휴면 색을 여기서 한 번만 칠하기
        // SyncInstances가 휴면 그룹을 건너뛰므로 다음 프레임에 덮이지 않는다
        m_NpcInstances[i].position[0] = m_Move.position[i].GetX();
        m_NpcInstances[i].position[1] = m_Move.position[i].GetY();
        m_NpcInstances[i].position[2] = m_Move.position[i].GetZ();
        m_NpcInstances[i].colorType = NpcRenderer::kNpcColorDormant;
    }
    m_VisualDirty = true;

    const int last = m_ActiveGroupCount - 1;
    std::swap(m_GroupOrder[slot], m_GroupOrder[last]);
    m_GroupSlot[m_GroupOrder[slot]] = slot;
    m_GroupSlot[m_GroupOrder[last]] = last;
    --m_ActiveGroupCount;
}

void NpcManager::RefreshActiveSet(const DirectX::XMINT3& playerCell)
{
    const int prx = playerCell.x / REGION_SIZE_CELLS;
    const int prz = playerCell.z / REGION_SIZE_CELLS;

    // region 안 바뀌었으면 activate 그대로
    if (prx != m_LastRegionX || prz != m_LastRegionZ)
    {
        m_LastRegionX = prx;
        m_LastRegionZ = prz;

        // 전체 재구성
        while (m_ActiveGroupCount > 0)
        {
            DeactivateGroup(m_GroupOrder[m_ActiveGroupCount - 1]);
        }

        const int pr = (WAKEUP_RADIUS_CELLS + REGION_SIZE_CELLS - 1) / REGION_SIZE_CELLS;

        for (int dz = -pr; dz <= pr; ++dz)
        {
            for (int dx = -pr; dx <= pr; ++dx)
            {
                const int rx = prx + dx, rz = prz + dz;
                if (rx < 0 || rx >= m_RegionCountX)   continue;
                if (rz < 0 || rz >= m_RegionCountZ)   continue;

                for (int g : m_Regions[rz * m_RegionCountX + rx].groupIndices)
                {
                    ActivateGroup(g);
                }
            }
        }
    }

    // 추격 그룹 다시 활성
    for (int g : m_PersistentGroups) ActivateGroup(g);
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

    // 색은 SyncInstances가 상태 기준으로 계산 
    m_VisualDirty = true;
    SyncInstances();
    NpcRenderer::UpdateInstances(m_NpcInstances);
    return m_GroupSelected;
}

bool NpcManager::SetGroupDestination(const DirectX::XMINT3& goalCell,
    std::vector<DirectX::XMINT3>* outPath)
{
    if (m_Move.size() == 0)  return false;

    // 디버그용 - 전체 즉시 chase로 전환

    for (size_t i = 0; i < m_Move.size(); ++i)
    {
        if (m_Move.currCell[i].x < 0) continue;
        m_Move.state[i] = NPC_STATE_CHASE;
        m_Move.propagationTimer[i] = 0.0f;
        m_Move.active[i] = 1;
        m_Move.stopReason[i] = 0;
    }
    for (auto& grp : m_BehaviorGroups) grp.deagroTimer = 0.0f;

    m_Group.goal = goalCell;
    m_FieldRequestTimer = 0.0f;          // 쿨다운 무시하고 즉시
    RebuildChaseLeaf();
    m_ChaseSetDirty = false;

    return true;
}


void NpcManager::Update(float dt, const DirectX::XMINT3& playerCell, bool playerValid)
{
    // --- 1. 워커 결과 수령 ---
    FieldBuildResult res = m_FieldWorker.TryAcquire();
    if (res.success)
    {
        for (auto& leafPtr : m_Leaves)
        {
            if (leafPtr->leafId != res.leafId)   continue;
            leafPtr->field = std::move(res.field);
            if (!res.nodePath.empty())  leafPtr->path = std::move(res.nodePath);
            leafPtr->active = true;
            m_FieldSwapped = true;
            break;
        }
    }

    if (m_Move.size() == 0) return;   // hasGoal 조기 이탈 제거 - Idle도 움직여야 한다

    UpdateAgro();                   // 트리거 해소

    // --- 2. 상태 전이 ---
    if (playerValid)
    {
        RefreshActiveSet(playerCell);   // 활성 집합 확정
        UpdateDeagro(playerCell, dt);   // 해제는 플레이어 거리를 직접 보니까 냅두기

        // 목표 셀이 바뀌면 필드를 다시 요청 - 플레이어 추종
        if (m_Group.isChasing &&
            (playerCell.x != m_LastRequestedGoal.x ||
                playerCell.y != m_LastRequestedGoal.y ||
                playerCell.z != m_LastRequestedGoal.z))
        {
            m_ChaseSetDirty = true;
        }

        m_Group.goal = playerCell;
    }
    PropagateAgro(dt);

    // --- 3. 필드 요청 ---
    // 파동을 매 프레임 요청하지 않고, 쌓았다가 한번에 걸기
    if(m_FieldRequestTimer > 0.0f)  m_FieldRequestTimer -= dt;
    if (true == m_ChaseSetDirty && m_FieldRequestTimer <= 0.0f)
    {
        RebuildChaseLeaf();
        m_ChaseSetDirty = false;
        m_FieldRequestTimer = FIELD_REQUEST_COOLDOWN_SEC;
    }


    // --- 4. 개체 이동 (상태별 분기) ---
    const float ARRIVE_EPS_SQ = 0.0001f;
    bool anyMoved = false;


    for (int k = 0; k < m_ActiveGroupCount; ++k)
    {
        for (int i : m_BehaviorGroups[m_GroupOrder[k]].members)
        {
            Math::Vector3 delta = m_Move.targetWorldPos[i] - m_Move.position[i];
            const float distSq = Math::LengthSquare(delta);

            // 셀에 도착한 프레임에만 다음 셀 정한다
            if (distSq < ARRIVE_EPS_SQ)
            {
                switch (m_Move.state[i])
                {
                case NPC_STATE_IDLE:
                    m_MovementSolver.AdvanceWanderCell(*m_Grid, i, dt);
                    break;

                case NPC_STATE_ALERTED:
                    break;  // 반응 대기 - 제자리

                case NPC_STATE_LOST:
                    m_Move.stateTimer[i] += dt;
                    // 막혔거나 시간초과
                    if (!m_MovementSolver.AdvanceReturnCell(*m_Grid, i) ||
                        m_Move.stateTimer[i] >= LOST_TIMEOUT_SEC)
                    {
                        // 현재 위치를 새 anchor로 - 일단 흩어지게 하고, 복귀 필수면 로직 수정
                        m_Move.anchorCell[i] = m_Move.currCell[i];
                        m_Move.state[i] = NPC_STATE_IDLE;
                        m_VisualDirty = true;
                        m_Move.stateTimer[i] = 0.0f;
                    }
                    break;

                case NPC_STATE_CHASE:
                    if (m_Move.active[i])
                    {
                        m_MovementSolver.AdvanceCell(*m_Grid, m_Leaves, i, dt, m_Group.isChasing);
                    }
                    break;
                default:
                    break;
                }
            }
            // --- 4-2. 위치 보간 (상태 무관 - 모든 이동이 셀 스냅 + 보간) ---
            else
            {
                const float step = NPC_SPEED * dt;
                if (step * step >= distSq) m_Move.position[i] = m_Move.targetWorldPos[i];
                else                       m_Move.position[i] += Math::Normalize(delta) * step;
                anyMoved = true;
            }
        }
    }

    
    if (anyMoved) 
    { 
        SyncInstances(); 
        NpcRenderer::UpdateInstances(m_NpcInstances);
        m_VisualDirty = false;
    }
}

bool NpcManager::IsVisitedAny(const VoxelGrid& grid, int x, int y, int z) const
{
    for (const auto& leafPtr : m_Leaves)
    {
        if (leafPtr->field->IsVisited(grid, x, y, z)) return true;
    }
    return false;
}

bool NpcManager::SampleDirectionAny(const VoxelGrid& grid, int x, int y, int z, DirectX::XMINT3& outDir) const
{
    for (const auto& leafPtr : m_Leaves)
    {
        if (leafPtr->field->SampleDirection(grid, x, y, z, outDir))   return true;
    }
    return false;
}

void NpcManager::OnTerrainChanged(const std::vector<DirectX::XMINT3>& editedCells)
{
    if (editedCells.empty()) return;

    m_FieldWorker.CancelAndWait();     // 그리드 변경 전 필수 (기존과 동일)
    ++m_TerrainGeneration;


    // --- NPC 재스냅 + 예약 갱신 ---
    // InitGroupMovement의 m_Reserve.Reset()이 사라졌으므로 여기서 직접 처리해야 한다.
    // 안 하면 아무도 없는 셀이 점유 상태로 굳어 통로가 영구히 막힌다
    for (size_t i = 0; i < m_Move.size(); ++i)
    {
        const DirectX::XMINT3 curr = m_Move.currCell[i];
        if (curr.x < 0) continue;
        if (m_Grid->IsWalkable(curr.x, curr.y, curr.z)) continue;

        m_Reserve.Release(MakeCellKey(curr));           // 옛 예약 해제

        int sx, sy, sz;
        Math::Vector3 p = m_Grid->GetWorldPos(curr.x, curr.y, curr.z);
        if (m_Grid->FindNearestWalkable(p, sx, sy, sz) &&
            m_Reserve.TryReserve(MakeCellKey(sx, sy, sz), (int)i))
        {
            const DirectX::XMINT3 cell{ sx, sy, sz };
            m_Move.currCell[i] = cell;
            m_Move.targetCell[i] = cell;
            m_Move.targetWorldPos[i] = GetNpcStandPos(cell, m_Move.halfHeight[i]);
            m_Move.position[i] = m_Move.targetWorldPos[i];
        }
        else
        {
            m_Move.active[i] = 0;
            m_Move.stopReason[i] = 1;
        }

        // 배회 중심이 막혔으면 Lost가 영영 복귀 못 한다
        const DirectX::XMINT3 a = m_Move.anchorCell[i];
        if (a.x >= 0 && !m_Grid->IsWalkable(a.x, a.y, a.z))
            m_Move.anchorCell[i] = m_Move.currCell[i];
    }

    // --- 필드 재빌드는 워커에 맡긴다 ---
    // 낡은 필드를 한 프레임 더 쓰지만, 이동 합법성은 GetWalkableNeighbors(살아있는 그리드)가
    // 결정하므로 벽으로 걸어 들어가지 않는다. 방향만 잠시 부정확할 뿐이다
    if (m_Group.isChasing)
    {
        m_ChaseSetDirty = true;
        m_FieldRequestTimer = 0.0f;
    }
}




void NpcManager::SyncInstances()
{
    // 활성 그룹만 갱신한다. 휴면 그룹은 위치가 고정이고 색도
    // DeactivateGroup에서 한 번 칠해둔 회색이 그대로 유효
    for (int k = 0; k < m_ActiveGroupCount; ++k)
    {
        for (int i : m_BehaviorGroups[m_GroupOrder[k]].members)
        {
            m_NpcInstances[i].position[0] = m_Move.position[i].GetX();
            m_NpcInstances[i].position[1] = m_Move.position[i].GetY();
            m_NpcInstances[i].position[2] = m_Move.position[i].GetZ();

            // colorType = NPC_STATE
            m_NpcInstances[i].colorType = m_GroupSelected ?
                NpcRenderer::kNpcColorSelected : (uint32_t)m_Move.state[i];
        }
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



void NpcManager::UpdateAgro()
{
    for (const auto& stim : m_Stimulus)
    {
        if (stim.cell.x < 0 || stim.radiusCells <= 0)    continue;

        const int R = stim.radiusCells;
        const int R2 = R * R;

        for (int k = 0; k < m_ActiveGroupCount; ++k)
        {
            auto& group = m_BehaviorGroups[m_GroupOrder[k]];

            // 브로드페이즈 - AABB R만큼 부풀려서 자극 위치가 밖이면 멤버 안보기
            // TODO : 현재 자극수 x 그룹수만큼 도는데, 자극원이 늘어 병목이면 공간 해시로 교체
            if (stim.cell.x < group.aabbMin.x - R || stim.cell.x > group.aabbMax.x + R ||
                stim.cell.z < group.aabbMin.z - R || stim.cell.z > group.aabbMax.z + R)
                continue;
            
            for (int i : group.members)
            {
                if (m_Move.state[i] != NPC_STATE_IDLE && m_Move.state[i] != NPC_STATE_LOST)
                    continue;

                const auto& c = m_Move.currCell[i];
                if (c.x < 0) continue;

                // xz 평면 원
                // TODO : 층 구분 필요시, 변경할 것
                const int dx = c.x - stim.cell.x, dz = c.z - stim.cell.z;
                if (dx * dx + dz * dz > R2) continue;

                m_Move.state[i] = stim.targetState;
                m_VisualDirty = true;   // 시각화 플래그
                m_Move.propagationTimer[i] = 0.0f;
                m_ChaseSetDirty = true; // 마스크 앵커 필드 갱신
            }

        }
    }

    // 트리거 해소
    m_Stimulus.clear();
}

void NpcManager::PropagateAgro(float dt)
{
    int budget = MAX_PROPAGATIONS_PER_FRAME;    // 전체 확산 방지용 - 한번당 확산 제한
    const int PR2 = PROPAGATION_RADIUS_CELLS * PROPAGATION_RADIUS_CELLS;


    for (int k = 0; k < m_ActiveGroupCount; ++k)
    {
        auto& group = m_BehaviorGroups[m_GroupOrder[k]];

        for (int i : group.members)
        {
            if (m_Move.state[i] != NPC_STATE_ALERTED)   continue;

            m_Move.propagationTimer[i] += dt;

            // 개체별 지연 - 파동이 계단식으로 안보이게
            const float j = (float)(m_Move.noiseSeed[i] & 0xFF) / 255.0f;   // 0..1
            const float delay = PROPAGATION_DELAY_SEC
                * (1.0f - PROPAGATION_JITTER + 2.0f * PROPAGATION_JITTER * j);

            if (m_Move.propagationTimer[i] < delay)  continue;

            m_Move.state[i] = NPC_STATE_CHASE;
            m_VisualDirty = true;
            m_ChaseSetDirty = true;

            // 같은 그룹 내에서만 전파
            const auto& src = m_Move.currCell[i];
            int infected = 0;
            for (int k : group.members)
            {
                // 프레임 제한 인원 넘으면 멈추기
                if (infected >= PROPAGATION_FANOUT || budget <= 0)   break;
                if (m_Move.state[k] != NPC_STATE_IDLE)   continue;

                const auto& c = m_Move.currCell[k];
                if (c.x < 0) continue;
                const int dx = c.x - src.x, dz = c.z - src.z;
                if (dx * dx + dz * dz > PR2) continue;

                m_Move.state[k] = NPC_STATE_ALERTED;
                m_Move.propagationTimer[k] = 0.0f;
                ++infected;
                --budget;
                m_ChaseSetDirty = true;
            }
        }
    }
}

void NpcManager::UpdateDeagro(const DirectX::XMINT3& playerCell, float dt)
{
    const int D2 = DEAGRO_RADIUS_CELLS * DEAGRO_RADIUS_CELLS;

    for (int k = 0; k < m_ActiveGroupCount; ++k)
    {
        auto& group = m_BehaviorGroups[m_GroupOrder[k]];

        if (!group.HasAnyChasing(m_Move))
        {
            group.deagroTimer = 0.0f;
            continue;
        }

        // chase 그룹은 흩어져서 AABB 부정확 - 일단은 멤버 순회로 최근접
        int nearest = INT_MAX;
        for (int i : group.members)
        {
            if (m_Move.state[i] != NPC_STATE_CHASE && m_Move.state[i] != NPC_STATE_ALERTED)
                continue;

            const auto& c = m_Move.currCell[i];
            if (c.x < 0) continue;
            const int dx = c.x - playerCell.x, dz = c.z - playerCell.z;
            const int d = dx * dx + dz * dz;
            if (d < nearest) nearest = d;
        }

        if (nearest <= D2)
        {
            group.deagroTimer = 0.0f;
            continue;
        }

        group.deagroTimer += dt;
        if (group.deagroTimer < DEAGRO_HOLD_SEC) continue;

        // 그룹 일괄 해제 - 개체별로 하면 그룹이 찢어버림
        for (int i : group.members)
        {
            if (m_Move.state[i] == NPC_STATE_CHASE || m_Move.state[i] == NPC_STATE_ALERTED)
            {
                m_Move.state[i] = NPC_STATE_LOST;
                m_Move.stateTimer[i] = 0.0f;
                m_Move.active[i] = 0;
            }
        }
        group.deagroTimer = 0.0f;
        m_ChaseSetDirty = true;
        m_VisualDirty = true;
    }

}

void NpcManager::RebuildChaseLeaf()
{
    LeafGroup& leaf = *m_Leaves[0];
    leaf.members.clear();

    m_PersistentGroups.clear();
    for (size_t g = 0; g < m_BehaviorGroups.size(); ++g)
    {
        if (m_BehaviorGroups[g].HasAnyChasing(m_Move))
        {
            m_PersistentGroups.push_back((int)g);
        }
    }

    for (size_t i = 0; i < m_Move.size(); ++i)
    {
        const uint8_t st = m_Move.state[i];
        if (st != NPC_STATE_CHASE && st != NPC_STATE_ALERTED)
            continue;

        leaf.members.push_back((int)i);
        m_Move.leafId[i] = 0;

        // Chase 진입 개체는 이동 활성화. Alerted는 아직 안 움직인다
        if (st == NPC_STATE_CHASE)
        {
            m_Move.active[i] = 1;
            m_Move.stopReason[i] = 0;
            m_Move.blockedTime[i] = 0.0f;
        }
    }

    if (leaf.members.empty())
    {
        leaf.active = false;
        m_Group.hasGoal = false;
        m_Group.isChasing = false;
        return;
    }

    m_Group.hasGoal = true;
    m_Group.isChasing = true;

    FieldBuildRequest req = m_SplitController.MakeRequest(
        leaf, m_Group.goal, m_TerrainGeneration, m_Move.currCell, m_Move.state);
    req.mode = FieldBuildMode::ChaseIncremental;
    m_FieldWorker.Submit(std::move(req));

    m_LastRequestedGoal = m_Group.goal;
}

