#include "NpcManager.h"
#include "ChunkKey.h"
#include "GridNeighbors.h"
#include <cmath>
#include <queue>

constexpr float NPC_SPEED = 10.5f;
constexpr float NPC_INER = 0.5f;                // 다른 방향 찾아갈때, 기존 방향으로 가게 하는 보정값


//static constexpr float NPC_WAIT_SECONDS = 0.167f / 2.0f; // 한 칸 통과 시간(~0.167초) 기준
constexpr float NPC_WAIT_RATIO = 0.3f;   // 1칸 움직이는데 비율 (x / 1.0f)%
constexpr float VOXEL_SIZE_REF = 0.5f;   // VoxelGrid::GetCellSize()와 반드시 일치해야 함
constexpr float NPC_CARDINAL_WAIT_SECONDS = (VOXEL_SIZE_REF / NPC_SPEED) * NPC_WAIT_RATIO;
constexpr float NPC_DIAGONAL_WAIT_SECONDS = (VOXEL_SIZE_REF * 1.41421356f / NPC_SPEED) * NPC_WAIT_RATIO;


const int TARGET_COUNT = 1000;

void NpcManager::Init(const VoxelGrid& grid)
{
    m_Grid = &grid;
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
    CORE_SCOPE(CorridorField_Build);
    MemoryProbe probe("SetGroupDestination");
    m_HasGoal = false;
    const size_t n = m_NpcInstances.size();
    if (n == 0) return false;

    // --- 1. 각 NPC의 시작 셀 수집 (마스크 시드 + 이동 초기화에 공유) ---
    // FindNearestWalkable을 여기서 1회만 돌리고, 결과를 멤버에 저장해 InitGroupMovement가 재사용
    m_StartCells.clear();
    m_StartCells.resize(n, { -1,-1,-1 });

    Math::Vector3 centroid(0.0f, 0.0f, 0.0f);
    int validCount = 0;
    {
        CORE_SCOPE(DestCollectStart);
        const bool hasPrevMove = (m_Move.size() > 0);   // 그룹 있는지 확인


        for (size_t i = 0; i < n; ++i)
        {
            DirectX::XMINT3 startCell{ -1, -1, -1 };
            bool resolved = false;

            // 1. 이전에 움직였으면, 기존의 타겟cell을 start로 설정
            if (true == hasPrevMove)
            {
                const auto& tc = m_Move.targetCell[i];
                if (m_Reserve.Find(MakeCellKey(tc)) == (int)i)
                {
                    startCell = tc;
                    resolved = true;
                }
            }

            // 2. 없으면 렌더 좌표로 역산하기
            if (false == resolved)
            {
                const auto& inst = m_NpcInstances[i];
                Math::Vector3 p(inst.position[0], inst.position[1], inst.position[2]);
                int sx, sy, sz;
                if (m_Grid->FindNearestWalkable(p, sx, sy, sz))
                {
                    startCell = { sx, sy, sz };
                    resolved = true;
                }
            }

            if (true == resolved)
            {
                m_StartCells[i] = startCell;
                centroid += Math::Vector3((float)startCell.x, (float)startCell.y, (float)startCell.z);
                validCount++;
            }

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
    { //CPU_SCOPE("Dest/AStar");
        
        CORE_SCOPE(DestAStar);
        if (!m_Pathfinder.FindPath(*m_Grid, { asx,asy,asz }, goalCell, path)) return false;
    }
    if (outPath) *outPath = path;

    // --- 3. margin을 "그룹 분포 반경"으로 계산 (시드 간 공백 방지) ---
    // 무게중심에서 가장 먼 NPC까지의 셀 거리 -> 그만큼 margin을 확보해야
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
    std::unordered_set<int64_t> mask;
    { 
        CORE_SCOPE(DestBuildMask);
        mask = BuildLayerMask(*m_Grid, path, m_StartCells, margin);
    }
    { 
        CORE_SCOPE(DestFieldBuild);
    // --- 5. FlowField 계산 ---
        m_CorridorField.Build(*m_Grid, goalCell, mask);
    }

    m_HasGoal = true;
    { CORE_SCOPE(DestInitMovement);
    InitGroupMovement();   // m_StartCells 재사용
    }

    probe.Report();
    m_CorridorField.ReportMemory();
    DEBUGPRINT("leaf chunks=%zu\n", leaf.field.GetChunks().size());

    return true;
}


void NpcManager::Update(float dt)
{
    //CPU_SCOPE("Frame/NpcUpdate");
    if (false == m_HasGoal) return;

    const float ARRIVE_EPS_SQ = 0.05f * 0.05f;
    const size_t n = m_Move.size();

    bool anyMoved = false;
    bool anyActive = false;
    {
        CORE_SCOPE(NpcUpdateCore);          // ← 여기서 열고
        for (int i : m_MoveOrder)
        {
            if (!m_Move.active[i]) continue;
            anyActive = true;       // 살아있는지 체크

            Math::Vector3 pos = m_Move.position[i];
            Math::Vector3 tgt = m_Move.targetWorldPos[i];
            Math::Vector3 delta = tgt - pos;

            if (Math::LengthSquare(delta) < ARRIVE_EPS_SQ)
            {

                AdvanceCell(i, dt);   // 도착 -> 셀 전환
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
    }
    // 누군가 움직였을때만 이동
    if (true == anyMoved)
    {
        {   SyncInstances(); }
        {  NpcRenderer::UpdateInstances(m_NpcInstances); }
    }

    // 전원 도착시 목표 종료
    if (false == anyActive)
    {
        m_HasGoal = false;
        MemoryProbe("전원 도착 후").Report();   // ReleaseChunks가 실제로 반납했는지
    }
}



bool NpcManager::IsMoveCross(size_t i, const DirectX::XMINT3& curr, const DirectX::XMINT3& next) const
{
    // 이동 방향(next)
    int dx = next.x - curr.x;
    int dz = next.z - curr.z;
    // 대각선 아니면 검사x
    if (dx == 0 || dz == 0)  return false;

    // 대각선 주변 복셀(어깨)
    const int sx[2] = { curr.x + dx, curr.x };
    const int sz[2] = { curr.z,      curr.z + dz };
    const int ys[2] = { curr.y,      next.y };

    for (int a = 0; a < 2; ++a)
    {
        // a는 지금 내 대각선 <-> b는 반대쪽 대각선
        int b = 1 - a;
        for (int t = 0; t < 2; ++t)
        {
            // 대각선에 점유한 npc가 없거나 자신이면 pass
            int occ = m_Reserve.Find(MakeCellKey(sx[a], ys[t], sz[a]));
            if (occ < 0 || occ == (int)i)    continue;
            // 
            const auto& tc = m_Move.targetCell[occ];
            if (tc.x == sx[b] && tc.z == sz[b])  return true;
        }
    }
    return false;
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
        if (m_CorridorField.SampleCost(*m_Grid, c.x, c.y, c.z, v))   cost[i] = v;
    }
    // 비용이 작은대로 오름차순 (거리 가까운게 그룹의 앞이 되도록)
    std::sort(m_MoveOrder.begin(), m_MoveOrder.end(), [&](int a, int b) {
        return cost[a] < cost[b];
        });

    ReportMemory();
}

//---------------------------------------------------------------------
//
//	이동시 규율
//
//---------------------------------------------------------------------
void NpcManager::AdvanceCell(size_t i, float dt)
{
    ++m_Stats.calls;

    SnapToTargetCell(i);
    const DirectX::XMINT3 curr = m_Move.currCell[i];

    float currCost;
    if (false == SampleCostCounted(curr.x, curr.y, curr.z, currCost))
    {
        m_Move.active[i] = 0;
        return;
    }
    if (currCost < 1e-4f)
    {
        m_Move.active[i] = 0;
        ++m_Stats.arrived;
        return;
    }

    m_NeighborScratch.clear();
    GetWalkableNeighbors(*m_Grid, curr, m_NeighborScratch);

    DirectX::XMINT3 best{ curr.x, curr.y, curr.z };

    bool waited = false;
    bool found = TryPrimaryDirection(i, curr, currCost, best, waited, dt);
    if (waited) { ++m_Stats.waited; return; }
    if (found) { ++m_Stats.primaryHit; }   // 1순위 or 성분분해 성공 (TryPrimaryDirection 내부에서 세분화 가능)

    bool hasActiveBlocker = false;
    if (!found)
    {
        found = PickFallbackCell(i, curr, currCost, best, hasActiveBlocker);
        if (found) ++m_Stats.fallbackHit;
    }

    if (!found)
    {
        if (!hasActiveBlocker)
        {
            m_Move.active[i] = 0;
            m_Move.blockedTime[i] = 0.0f;
            ++m_Stats.arrived;
            return;
        }
        HoldPosition(i, curr);
        ++m_Stats.stuck;
        return;
    }

    CommitMove(i, curr, best);

}

void NpcManager::SnapToTargetCell(size_t i)
{
    m_Move.position[i] = m_Move.targetWorldPos[i];
    m_Move.currCell[i] = m_Move.targetCell[i];
}

void NpcManager::HoldPosition(size_t i, const DirectX::XMINT3& curr)
{
    m_Move.targetCell[i] = curr;
    m_Move.targetWorldPos[i] = m_Move.position[i];
}

// 특정 이웃 셀이 지금 이동 가능한 유효 후보인지 검사
// (cost 낮아짐 + 미점유 + 비교차). walkable 여부는 m_NeighborScratch에 있는지로 판단
bool NpcManager::TryCandidate(size_t i, const DirectX::XMINT3& curr, float currCost, const DirectX::XMINT3& cand, DirectX::XMINT3& out) const
{
    bool isNeighbor = false;
    for (const auto& n : m_NeighborScratch)
    {
        if (n.pos.x == cand.x && n.pos.y == cand.y && n.pos.z == cand.z) { isNeighbor = true; break; }
    }
    if (!isNeighbor) return false;

    float nc;
    if (false == m_CorridorField.SampleCost(*m_Grid, cand.x, cand.y, cand.z, nc)) return false; // flowfield 아님
    if (nc >= currCost)                                                    return false; // 목적지로 안 가까워짐
    if (m_Reserve.Find(MakeCellKey(cand.x, cand.y, cand.z)) >= 0)          return false; // 점유됨
    if (true == IsMoveCross(i, curr, cand))                                return false; // 교차

    out = cand;
    return true;
}

bool NpcManager::TryPrimaryDirection(size_t i, const DirectX::XMINT3& curr, float currCost,
    DirectX::XMINT3& best, bool& outWaited, float dt)
{
    outWaited = false;

    DirectX::XMINT3 desired;
    if (false == m_CorridorField.SampleDirection(*m_Grid, curr.x, curr.y, curr.z, desired))
        return false;

    DirectX::XMINT3 desiredCell{ curr.x + desired.x, curr.y + desired.y, curr.z + desired.z };

    // (1) 1순위 방향이 바로 가능하면 그대로
    if (TryCandidate(i, curr, currCost, desiredCell, best)) return true;

    // (2) 막힌 이유가 곧 비킬 npc 때문이면 방향 안 바꾸고 대기
    int occ = m_Reserve.Find(MakeCellKey(desiredCell));
    bool waitable = (occ >= 0 && occ != (int)i && m_Move.active[occ] != 0);
    if (waitable)
    {
        bool isDiagonal = (desired.x != 0 && desired.z != 0);
        float waitLimit = isDiagonal ? NPC_DIAGONAL_WAIT_SECONDS : NPC_CARDINAL_WAIT_SECONDS;

        if (m_Move.blockedTime[i] < waitLimit)
        {
            m_Move.blockedTime[i] += dt;
            HoldPosition(i, curr);
            outWaited = true;
            return false;
        }
    }

    // (3) 1순위가 대각선인데 막혔으면 -> 성분 분해 대안
    if (desired.x == 0 || desired.z == 0)   return false;

    DirectX::XMINT3 zKeep{ curr.x,             curr.y + desired.y, curr.z + desired.z }; // x 버림 (직진 z)
    DirectX::XMINT3 xKeep{ curr.x + desired.x, curr.y + desired.y, curr.z };             // z 버림 (직진 x)

    bool xShoulderBlocked = (m_Reserve.Find(MakeCellKey(curr.x + desired.x, curr.y, curr.z)) >= 0);
    bool zShoulderBlocked = (m_Reserve.Find(MakeCellKey(curr.x, curr.y, curr.z + desired.z)) >= 0);

    // 막힘 유발 축의 반대 성분을 우선 시도 - 3갈래로 나뉘어 있던 분기를
    // 시도 순서(first/second)를 정하는 것"과 시도 자체로 분리해 중복 제거
    DirectX::XMINT3 first = zKeep;
    DirectX::XMINT3 second = xKeep;
    if (zShoulderBlocked && !xShoulderBlocked)
    {
        first = xKeep;
        second = zKeep;
    }
    // xShoulderBlocked && !zShoulderBlocked, 그리고 "둘 다 막힘/둘 다 안 막힘" 모두 zKeep 우선(기본값)

    if (TryCandidate(i, curr, currCost, first, best)) return true;
    if (TryCandidate(i, curr, currCost, second, best)) return true;
    return false;
}

bool NpcManager::PickFallbackCell(size_t i, const DirectX::XMINT3& curr, float currCost,
    DirectX::XMINT3& best, bool& outHasActiveBlocker)
{
    outHasActiveBlocker = false;    // 점유자 다 죽어있으면 도착 판정용
    float bestCost = currCost;
    bool found = false;

    for (const auto& n : m_NeighborScratch)
    {
        float nc;
        if (false == m_CorridorField.SampleCost(*m_Grid, n.pos.x, n.pos.y, n.pos.z, nc)) continue;
        if (nc >= currCost) continue;

        // 점유자가 살아있는지
        int occ = m_Reserve.Find(MakeCellKey(n.pos));
        if (occ >= 0)
        {
            if (occ != (int)i && m_Move.active[occ]) outHasActiveBlocker = true;
            continue;
        }
        // 교차 거부도 살아있는 점유자 때문
        if (true == IsMoveCross(i, curr, n.pos))
        {
            outHasActiveBlocker = true;
            continue;
        }

        int dx = n.pos.x - curr.x, dz = n.pos.z - curr.z;
        const auto& ld = m_Move.lastDir[i];
        float score = nc;
        if (dx == ld.x && dz == ld.z) score -= NPC_INER;   // 기존 방향으로 계속 가도록 보정값
        if (score < bestCost)
        {
            bestCost = score;
            best = n.pos;
            found = true;
        }
    }
    return found;
}


void NpcManager::CommitMove(size_t i, const DirectX::XMINT3& curr, const DirectX::XMINT3& best)
{
    if (m_Reserve.TryReserve(MakeCellKey(best), (int)i))
    {
        m_Reserve.Release(MakeCellKey(curr.x, curr.y, curr.z));

        m_Move.targetCell[i] = best;
        m_Move.lastDir[i] = { best.x - curr.x, best.y - curr.y, best.z - curr.z };
        m_Move.targetWorldPos[i] = GetNpcStandPos(best, m_Move.halfHeight[i]);
        m_Move.blockedTime[i] = 0;   // 이동 성공해서 대기 프레임 초기화
    }
    else
    {
        // 이론상 발생하면 안 되지만(루프에서 이미 미점유 확인), 방어적으로 처리
        HoldPosition(i, curr);
    }
}
//-----------------------------------------------------------------


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

bool NpcManager::SampleCostCounted(int x, int y, int z, float& outCost)
{
    ++m_Stats.sampleCostCalls;
    return m_CorridorField.SampleCost(*m_Grid, x, y, z, outCost);
}
bool NpcManager::SampleDirectionCounted(int x, int y, int z, DirectX::XMINT3& outDir)
{
    ++m_Stats.sampleDirCalls;
    return m_CorridorField.SampleDirection(*m_Grid, x, y, z, outDir);
}
int NpcManager::ReserveFindCounted(int64_t key)
{
    ++m_Stats.reserveFindCalls;
    return m_Reserve.Find(key);
}


void NpcManager::ReportMemory(const char* filePath) const
{
    auto vecB = [](const auto& v) {
        return v.capacity() * sizeof(typename std::decay_t<decltype(v)>::value_type);
    };

    size_t moveData = vecB(m_Move.position) + vecB(m_Move.targetWorldPos)
        + vecB(m_Move.currCell) + vecB(m_Move.targetCell)
        + vecB(m_Move.active) + vecB(m_Move.halfHeight)
        + vecB(m_Move.lastDir) + vecB(m_Move.blockedTime);

    size_t instances = vecB(m_NpcInstances);
    size_t moveOrder = vecB(m_MoveOrder);
    size_t startCells = vecB(m_StartCells);

    // unordered_map/set 근사: bucket_count()*16(포인터 배열) + size()*(노드+오버헤드)
    auto mapB = [](size_t bucketCount, size_t count, size_t nodePayload) {
        return bucketCount * 16 + count * (nodePayload + 16);
    };
    size_t reserve = mapB(m_Reserve.BucketCount(), m_Reserve.Size(), 12); // CellReservation 내부 접근자 필요

    char buf[1024];
    int n = sprintf_s(buf,
        "[NpcManager] NPC=%zu\n"
        "  NpcMoveData(SoA)   : %8.1f KB\n"
        "  m_NpcInstances     : %8.1f KB\n"
        "  m_MoveOrder        : %8.1f KB\n"
        "  m_StartCells       : %8.1f KB\n"
        "  CellReservation    : %8.1f KB (bucket=%zu size=%zu)\n"
        "  --- 합계           : %8.1f KB\n",
        m_Move.size(),
        moveData / 1024.0, instances / 1024.0, moveOrder / 1024.0, startCells / 1024.0,
        reserve / 1024.0, m_Reserve.BucketCount(), m_Reserve.Size(),
        (moveData + instances + moveOrder + startCells + reserve) / 1024.0);

    Utility::Print(buf);
    FILE* fp = nullptr; fopen_s(&fp, filePath, "a");
    if (fp) { fputs(buf, fp); fclose(fp); }
}