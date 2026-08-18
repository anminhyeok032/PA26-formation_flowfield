#include "NpcManager.h"
#include "ChunkKey.h"
#include "GridNeighbors.h"
#include <cmath>
#include <queue>
#include "NpcConstants.h"
#include <algorithm>

NpcManager::NpcManager()
    : m_MovementSolver(m_Move, m_Reserve)
    , m_SplitController(m_Leaves, m_Move, m_Group, m_StartCells)
{
}

void NpcManager::Init(const VoxelGrid& grid, const ChunkGraph& chunkgraph, const DirectX::XMINT3& playerStartCell)
{
#ifdef _DEBUG
    // 플래그는 자유 조합이라 모순된 표도 컴파일이 통과한다.
    // 성립해야 할 함의를 여기서 강제한다
    for (int s = 0; s < NPC_STATE_COUNT; ++s)
    {
        const uint8_t f = NPC_STATE_FLAGS[s];

        // 필드를 구독하는데 마스크 앵커가 아니면, 자기 위치가 회랑에 없어
        // SampleCost가 실패하고 즉시 정지한다
        ASSERT(!(f & NSF_FIELD_USER) || (f & NSF_MASK_ANCHOR));

        // 감지 대상이면서 앵커면, 자극이 매 프레임 상태를 덮어써
        // propagationTimer가 리셋되고 CHASE에 영원히 도달하지 못한다
        ASSERT(!(f & NSF_AGRO_TARGET) || !(f & NSF_MASK_ANCHOR));

        // 전파로 감염될 수 있는데 이미 앵커면 위와 같은 문제
        ASSERT(!(f & NSF_PROPAGATABLE) || !(f & NSF_MASK_ANCHOR));
    }
#endif

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
        m_Move.targetWorldPos[i] = GetNpcStandPos(cell, m_Move.halfHeight[i]);

        // Resize가 이미 IDLE로 채웠으므로 SetNpcState를 부르지 않는다.
        // 부르면 조기 반환에 걸려 아래 흩뿌리기만 남는데, 의존이 우연이라 명시적으로 둔다
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

        // 거점 = 멤버들의 중심
        int sx = 0, sz = 0, cnt = 0;
        // anchor
        int ax, ay, az;
        for (int m : group.members)
        {
            const auto& c = m_Move.currCell[m];
            if (c.x < 0) continue;
            sx += c.x; sz += c.z; ++cnt;
        }

        if (cnt > 0)
        {

            Math::Vector3 p = grid.GetWorldPos(sx / cnt, 0, sz / cnt);
            if (grid.FindNearestWalkable(p, ax, ay, az))
                group.anchorCell = { ax, ay, az };
            else
                group.anchorCell = m_Move.currCell[group.members[0]];   // 폴백
        }

        group.wanderRadius = GroupWanderRadius((int)group.members.size());
        group.regionIndex = RegionIndexOf(ax, az);
        m_Regions[group.regionIndex].groupIndices.push_back((int)i);   // 처음엔 전부 휴면

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
    // BFS 확장 한계를 최종 배회 반경에 맞춘다
    const int wanderR = GroupWanderRadius(wantCount);
    const int R2 = wanderR * wanderR;

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
        inst.colorType = NpcRenderer::kNpcColorDormant;

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

    auto& group = m_BehaviorGroups[g];

    // --- 개체 정규화 ---
    int sx = 0, sz = 0, cnt = 0;
    for (int i : group.members)
    {
        // 보간 중이던 개체가 셀 사이에 얼어붙지 않도록 목표 셀로
        m_Move.position[i] = m_Move.targetWorldPos[i];
        m_Move.currCell[i] = m_Move.targetCell[i];

        // 휴면 중에는 상태를 되돌려주는 코드가 하나도 없다 -
        // 모든 루프(이동/감지/전파/해제)가 활성 그룹만 순회하기 때문이다.
        // IDLE 외의 상태로 잠들면 타이머가 멈춘 채 냉동되고,
        // 앵커 상태로 잠들면 마스크만 잡아당기며 움직이지도 해제되지도 않는다
        if (m_Move.state[i] != NPC_STATE_IDLE)
        {
            SetNpcState(i, NPC_STATE_IDLE);
        }

        const auto& c = m_Move.currCell[i];
        if (c.x >= 0) { sx += c.x; sz += c.z; ++cnt; }

        m_NpcInstances[i].position[0] = m_Move.position[i].GetX();
        m_NpcInstances[i].position[1] = m_Move.position[i].GetY();
        m_NpcInstances[i].position[2] = m_Move.position[i].GetZ();
        m_NpcInstances[i].colorType = NpcRenderer::kNpcColorDormant;
    }
    m_VisualDirty = true;

    // 버킷에는 휴면 그룹만
    if (cnt > 0)
    {
        group.regionIndex = RegionIndexOf(sx / cnt, sz / cnt);
        m_Regions[group.regionIndex].groupIndices.push_back(g);
    }


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
    if (prx == m_LastRegionX && prz == m_LastRegionZ) return;

    const int pr = ACTIVE_REGION_RINGS;
    const int minX = prx - pr, maxX = prx + pr;
    const int minZ = prz - pr, maxZ = prz + pr;

    // 활성 사각형(셀 단위). 그룹 AABB와 겹치는지 보기 위해 리전 -> 셀로 환산
    const int actMinX = minX * REGION_SIZE_CELLS;
    const int actMaxX = (maxX + 1) * REGION_SIZE_CELLS - 1;
    const int actMinZ = minZ * REGION_SIZE_CELLS;
    const int actMaxZ = (maxZ + 1) * REGION_SIZE_CELLS - 1;

    // ---- 1. 재우기 ----
    // ACTIVE_REGION_RINGS 안에 있는지 살아있는 그룹 aabb 와 겹칩으로 검사 판정
    for (int k = m_ActiveGroupCount - 1; k >= 0; --k)
    {
        const int g = m_GroupOrder[k];
        const auto& group = m_BehaviorGroups[g];

        // 겹침 판정. 중심점으로 하면 늘어진 그룹에서 깨진다 -
        // 앞쪽 멤버가 플레이어 옆인데 중심이 뒤에 있어 잠들어버린다
        const bool overlap =
            group.aabbMax.x >= actMinX && group.aabbMin.x <= actMaxX &&
            group.aabbMax.z >= actMinZ && group.aabbMin.z <= actMaxZ;
        if (overlap) continue;

        // 뒤에서부터 도는 이유: DeactivateGroup이 slot <-> last swap해서
        DeactivateGroup(g);
    }

    // ---- 2. 깨우기 - 활성 범위 리전 버킷 비우기 ----
    for (int rz = minZ; rz <= maxZ; ++rz)
    {
        for (int rx = minX; rx <= maxX; ++rx)
        {
            if (rx < 0 || rx >= m_RegionCountX) continue;
            if (rz < 0 || rz >= m_RegionCountZ) continue;

            // 버킷을 통째로 옮겨 비우기
            auto& bucket = m_Regions[rz * m_RegionCountX + rx].groupIndices;
            if (bucket.empty()) continue;

            std::vector<int> taken;
            taken.swap(bucket);
            for (int g : taken) ActivateGroup(g);
        }
    }

    m_LastRegionX = prx;
    m_LastRegionZ = prz;
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

    for (int k = 0; k < m_ActiveGroupCount; ++k)
    {
        for (int i : m_BehaviorGroups[m_GroupOrder[k]].members)
        {
            if (m_Move.currCell[i].x < 0) continue;
            SetNpcState(i, NPC_STATE_CHASE);
            m_Move.stopReason[i] = 0;
        }
    }


    m_Group.goal = goalCell;
    m_FieldRequestTimer = 0.0f;          // 쿨다운 무시하고 즉시
    RebuildChaseLeaf();
    m_ChaseSetDirty = false;

    return true;
}


void NpcManager::Update(float dt, const DirectX::XMINT3& playerCell, bool playerValid)
{
    // --- 1. 워커 결과 수령 ---
    // near/far는 슬롯이 분리돼 있으므로 각각 수령
    {
        FieldBuildResult nres = m_FieldWorker.TryAcquireNear();
        if (nres.success) 
        {
            m_NearField = std::move(nres.field);
            m_FieldSwapped = true;      // 시각화 갱신
        }
    }

    FieldBuildResult res = m_FieldWorker.TryAcquireFar();
    if (res.success)
    {
        for (auto& leafPtr : m_Leaves)
        {
            if (leafPtr->leafId != res.leafId)   continue;
            leafPtr->field = std::move(res.field);
            if (!res.nodePath.empty())  leafPtr->path = std::move(res.nodePath);
            leafPtr->active = true;
            m_FieldSwapped = true;

            // 설치 확정되면 원점 갱신
            m_FarFieldGoal = m_Group.goal;

            break;
        }
    }

    if (m_Move.size() == 0) return;   // hasGoal 조기 이탈 제거 - Idle도 움직여야 한다


    // 새로 들어온 그룹 활성화
    if (playerValid) RefreshActiveSet(playerCell);

    UpdateAgro();                   // 트리거 해소

    // --- 2. 상태 전이 ---
    if (playerValid)
    {
        UpdateDeagro(playerCell, dt);   // 해제는 플레이어 거리를 직접 보니까 냅두기

        // 목표 이동해서 far 재빌드 해야될지
        if (m_Group.isChasing && IsFarFieldGoalTooOld())
        {
            m_ChaseSetDirty = true;
        }

        m_Group.goal = playerCell;
    }
    PropagateAgro(dt);

    // --- 3. 필드 요청 ---
    // 파동을 매 프레임 요청하지 않고, 쌓았다가 한번에 걸기
    if (m_FieldRequestTimer > 0.0f)  m_FieldRequestTimer -= dt;

    if (m_FieldRequestTimer <= 0.0f)
    {
        m_FieldRequestTimer = FIELD_REQUEST_COOLDOWN_SEC;

        // (a) near - 목표가 있으면 무조건 매 쿨다운 갱신
        if (m_Group.isChasing && m_Group.goal.x >= 0)
        {
            FieldBuildRequest nreq;
            nreq.mode = FieldBuildMode::GoalBubble;
            nreq.leafId = 0;                    // IsValid 통과용 - near는 leaf에 안 붙는다
            nreq.goalCell = m_Group.goal;
            nreq.generation = m_TerrainGeneration;
            m_FieldWorker.Submit(std::move(nreq));
        }

        // (b) far - 모양 변화(앵커 집합) 또는 원점 노후화일 때
        if (true == m_ChaseSetDirty)
        {
            RebuildChaseLeaf();
            m_ChaseSetDirty = false;
        }
    }


    // --- 4. 개체 이동 (상태별 분기) ---
    const float ARRIVE_EPS_SQ = 0.0001f;
    bool anyMoved = false;


    for (int k = 0; k < m_ActiveGroupCount; ++k)
    {
        auto& group = m_BehaviorGroups[m_GroupOrder[k]];
        // 도착 판정 반경 - 배회 반경보다 1크게
        const int arriveR2 = (group.wanderRadius + 5) * (group.wanderRadius + 5);

        for (int i : group.members)
        {
            Math::Vector3 delta = m_Move.targetWorldPos[i] - m_Move.position[i];
            const float distSq = Math::LengthSquare(delta);

            // 셀에 도착한 프레임에만 다음 셀 정한다
            if (distSq < ARRIVE_EPS_SQ)
            {
                switch (m_Move.state[i])
                {
                case NPC_STATE_IDLE:
                    m_MovementSolver.AdvanceWanderCell(*m_Grid, i, dt, group.anchorCell, group.wanderRadius);
                    break;

                case NPC_STATE_ALERTED:
                    break;  // 반응 대기 - 제자리

                case NPC_STATE_LOST:
                {
                    const bool moved = m_MovementSolver.AdvanceReturnCell(*m_Grid, i, group.anchorCell);

                    // 막힌건 시간 기다렸다가 포기 - 한번만 기다리면 멈춤이 잦다
                    if (moved)   m_Move.blockedTime[i] = 0.0f;
                    else        m_Move.blockedTime[i] += dt;

                    // group anchor 들어왔는지 검사
                    const int dx = m_Move.currCell[i].x - group.anchorCell.x;
                    const int dz = m_Move.currCell[i].z - group.anchorCell.z;
                    const bool arrived = (dx * dx + dz * dz <= arriveR2);

                    if (arrived || m_Move.blockedTime[i] >= LOST_STUCK_GIVEUP_SEC)
                    {
                        SetNpcState(i, NPC_STATE_IDLE);
                    }

                    break;
                }
                case NPC_STATE_CHASE:
                    if (m_Move.active[i])
                    {
                        // near없으면 자동으로 far
                        m_MovementSolver.AdvanceCell(*m_Grid, m_Leaves, m_NearField.get(), i, dt, m_Group.isChasing);
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

    
    if (anyMoved || m_VisualDirty)
    { 
        SyncInstances(); 
        NpcRenderer::UpdateInstances(m_NpcInstances);
        m_VisualDirty = false;
    }

#ifdef _DEBUG
    // 휴면 그룹은 전원 IDLE이어야 한다.
    // 깨지면 그 개체는 타이머가 멈춘 채 영원히 그 상태로 남는다
    for (int k = m_ActiveGroupCount; k < (int)m_GroupOrder.size(); ++k)
        for (int i : m_BehaviorGroups[m_GroupOrder[k]].members)
            ASSERT(m_Move.state[i] == NPC_STATE_IDLE);
#endif
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
    // 거점이 막혔으면 그룹 단위로 한 번만 스냅
    // 개체별로 하면 anchor가 N개로 흩어져 그룹 응집이 깨진다
    for (auto& group : m_BehaviorGroups)
    {
        const auto& a = group.anchorCell;
        if (a.x < 0 || m_Grid->IsWalkable(a.x, a.y, a.z)) continue;

        int ax, ay, az;
        Math::Vector3 p = m_Grid->GetWorldPos(a.x, a.y, a.z);
        if (m_Grid->FindNearestWalkable(p, ax, ay, az))
        {
            group.anchorCell = { ax, ay, az };
        }
    }
    

    // --- 필드 재빌드는 워커에 맡긴다 ---
    // 낡은 필드를 한 프레임 더 쓰지만, 이동 합법성은 GetWalkableNeighbors(살아있는 그리드)가
    // 결정하므로 벽으로 걸어 들어가지 않는다. 방향만 잠시 부정확할 뿐이다
    if (m_Group.isChasing)
    {
        m_ChaseSetDirty = true;
        m_FieldRequestTimer = 0.0f;
    }

    // near 폐기
    m_NearField.reset();

    // far 원점 폐기
    m_FarFieldGoal = { -1, -1, -1 };
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
    // AABB 갱신은 자극 유무와 무관하다.
    // 자극 루프 안에 두면 자극이 없는 프레임에 갱신이 멈추고,
    // 자극이 둘이면 같은 계산을 두 번 한다
    for (int k = 0; k < m_ActiveGroupCount; ++k)
    {
        auto& group = m_BehaviorGroups[m_GroupOrder[k]];

        int minX = INT_MAX, maxX = INT_MIN, minZ = INT_MAX, maxZ = INT_MIN;
        for (int i : group.members)
        {
            const auto& c = m_Move.currCell[i];
            if (c.x < 0) continue;
            if (c.x < minX) minX = c.x;
            if (c.x > maxX) maxX = c.x;
            if (c.z < minZ) minZ = c.z;
            if (c.z > maxZ) maxZ = c.z;
        }
        if (minX <= maxX)
        {
            group.aabbMin = { minX - WANDER_RADIUS_CELLS, 0, minZ - WANDER_RADIUS_CELLS };
            group.aabbMax = { maxX + WANDER_RADIUS_CELLS, 0, maxZ + WANDER_RADIUS_CELLS };
        }
    }

    for (const auto& stim : m_Stimulus)
    {
        if (stim.cell.x < 0 || stim.radiusCells <= 0) continue;

        const int R = stim.radiusCells;
        const int R2 = R * R;

        for (int k = 0; k < m_ActiveGroupCount; ++k)
        {
            auto& group = m_BehaviorGroups[m_GroupOrder[k]];

            if (stim.cell.x < group.aabbMin.x - R || stim.cell.x > group.aabbMax.x + R ||
                stim.cell.z < group.aabbMin.z - R || stim.cell.z > group.aabbMax.z + R)
                continue;

            for (int i : group.members)
            {
                // 이미 각성한 개체는 건드리지 않는다.
                // CHASE/ALERTED를 다시 ALERTED로 덮으면 propagationTimer가 매 프레임
                // 리셋되어 PROPAGATION_DELAY를 영원히 못 채우고 그 자리에 굳는다
                if (!StateHas(m_Move.state[i], NSF_AGRO_TARGET)) continue;

                const auto& c = m_Move.currCell[i];
                if (c.x < 0) continue;

                const int dx = c.x - stim.cell.x, dz = c.z - stim.cell.z;
                if (dx * dx + dz * dz > R2) continue;

                SetNpcState(i, stim.targetState);
            }
        }
    }
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
            const float j = (float)(m_Move.noiseSeed[i] & 0xFF) / 255.0f;
            const float delay = PROPAGATION_DELAY_SEC
                * (1.0f - PROPAGATION_JITTER + 2.0f * PROPAGATION_JITTER * j);
            if (m_Move.propagationTimer[i] < delay)  continue;

            const auto src = m_Move.currCell[i];   // 전이 전에 복사 - 아래에서 참조 무효화 없음
            SetNpcState(i, NPC_STATE_CHASE);

            // 같은 그룹 내에서만 전파한다. 그룹 간 허용 시 맵 전체가 도미노로 켜져
            // 어그로 반경을 성능 예산으로 쓰는 설계가 무너진다
            int infected = 0;
            for (int j2 : group.members)
            {
                if (infected >= PROPAGATION_FANOUT || budget <= 0)   break;

                // LOST는 감염 대상이 아니다. 복귀 중 휩쓸리면 해제/재추격이 반복돼
                // 회랑이 계속 요동친다
                if (!StateHas(m_Move.state[j2], NSF_PROPAGATABLE)) continue;

                const auto& c = m_Move.currCell[j2];
                if (c.x < 0) continue;
                const int dx = c.x - src.x, dz = c.z - src.z;
                if (dx * dx + dz * dz > PR2) continue;

                SetNpcState(j2, NPC_STATE_ALERTED);
                ++infected;
                --budget;
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

        // 추격 중인 그룹만 검사
        if (!group.HasAnyChasing(m_Move)) continue;

        // 앵커 개체 중 플레이어에 가장 가까운 거리
        // Chase 그룹은 흩어져서 AABB가 부정확하므로 멤버를 직접 순회한다
        int nearest = INT_MAX;
        for (int i : group.members)
        {
            if (!StateHas(m_Move.state[i], NSF_MASK_ANCHOR)) continue;
            const auto& c = m_Move.currCell[i];
            if (c.x < 0) continue;
            const int dx = c.x - playerCell.x, dz = c.z - playerCell.z;
            const int d = dx * dx + dz * dz;
            if (d < nearest) nearest = d;
        }


        if (nearest == INT_MAX) continue;   // 앵커 멤버 없음 = 추격 중이 아닌 그룹
        if (nearest <= D2)      continue;


        // 해제된 개체는 LOST로 anchor 쪽으로 더 멀어짐
        for (int i : group.members)
        {
            // 움직이고 있으면 continue
            if (!StateHas(m_Move.state[i], NSF_MASK_ANCHOR)) continue;

            // ALERTED는 IDLE로 되돌린다.
            // LOST로 보내면 UpdateAgro가 다시 ALERTED로 올려(둘 다 AGRO_TARGET)
            // propagationTimer가 리셋되는 사이클이 생긴다.
            // IDLE도 AGRO_TARGET이지만 그 자리에서 배회하므로 복귀 이동이 없다
            SetNpcState(i, (m_Move.state[i] == NPC_STATE_ALERTED)
                ? NPC_STATE_IDLE : NPC_STATE_LOST);
        }
    }
    
}

void NpcManager::RebuildChaseLeaf()
{
    LeafGroup& leaf = *m_Leaves[0];
    leaf.members.clear();

    for (size_t i = 0; i < m_Move.size(); ++i)
    {
        // 마스크 앵커만 모은다. active/stopReason은 SetNpcState가 이미 처리했으므로
        // 여기서 만지지 않는다 - 필드 요청 함수가 이동 상태를 바꾸면
        // 쿨다운(0.2초)만큼 늦게 적용되고, dirty가 안 서면 아예 적용되지 않는다
        if (!StateHas(m_Move.state[i], NSF_MASK_ANCHOR)) continue;

        leaf.members.push_back((int)i);
        m_Move.leafId[i] = 0;
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

// far 원점이 near 밖으로 밀려났는지
bool NpcManager::IsFarFieldGoalTooOld() const
{
    if (m_FarFieldGoal.x < 0)    return true;   // far 없음
    if (m_Group.goal.x < 0)     return true;    // 목표 없음

    const int dx = m_Group.goal.x - m_FarFieldGoal.x;
    const int dz = m_Group.goal.z - m_FarFieldGoal.z;

    // 직선 거리로 판정
    const float d2 = float(dx * dx + dz * dz);
    return d2 >= (FAR_FIELD_REBUILD_DIST * FAR_FIELD_REBUILD_DIST);
}


void NpcManager::SetNpcState(size_t i, uint8_t next)
{
    const uint8_t prev = m_Move.state[i];
    if (prev == next) return;   // 중복 전이 - 타이머를 되돌리지 않는다

    m_Move.state[i] = next;
    m_VisualDirty = true;       // 색이 상태에서 파생되므로 항상 갱신 필요

    // 마스크 앵커 집합이 바뀌면 회랑을 다시 계산
    // 진입(IDLE->ALERTED)과 이탈(CHASE->LOST) 양쪽 모두
    if (StateHas(prev, NSF_MASK_ANCHOR) != StateHas(next, NSF_MASK_ANCHOR))
        m_ChaseSetDirty = true;

    // --- 공통 초기화 ---
    m_Move.propagationTimer[i] = 0.0f;
    m_Move.blockedTime[i] = 0.0f;

    // active는 필드를 따라 이동 중이라는 뜻이므로 NSF_FIELD_USER에서 파생
    m_Move.active[i] = StateHas(next, NSF_FIELD_USER) ? 1 : 0;

    // --- 상태별 초기화 ---
    switch (next)
    {
    case NPC_STATE_IDLE:
        // 배회 대기 카운트다운. 0이면 다음 도착 프레임에 즉시 첫 걸음
        m_Move.stateTimer[i] = 0.0f;
        break;

    case NPC_STATE_CHASE:
        // 새로 추격에 합류했으므로 이전 정지 사유는 무효
        m_Move.stopReason[i] = 0;
        break;

    default:
        break;
    }
}

