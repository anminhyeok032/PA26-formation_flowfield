#include "NpcManager.h"
#include "ChunkKey.h"
#include "GridNeighbors.h"
#include <cmath>
#include <queue>

constexpr float NPC_SPEED = 1.0f;
constexpr float NPC_INER = 0.5f;
const int TARGET_COUNT = 1000;

void NpcManager::Init(const VoxelGrid& grid)
{
    m_Grid = &grid;
    const float voxelSize = grid.GetCellSize();
    const float NPC_WIDTH = voxelSize / 2.0f;
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
    m_HasGoal = false;
    const size_t n = m_NpcInstances.size();
    if (n == 0) return false;

    // --- 1. 각 NPC의 시작 셀 수집 (마스크 시드 + 이동 초기화에 공유) ---
    // FindNearestWalkable을 여기서 1회만 돌리고, 결과를 멤버에 저장해 InitGroupMovement가 재사용
    m_StartCells.clear();
    m_StartCells.resize(n, { -1,-1,-1 });

    Math::Vector3 centroid(0.0f, 0.0f, 0.0f);
    int validCount = 0;

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
    auto mask = BuildLayerMask(*m_Grid, path, m_StartCells, margin);

    // --- 5. FlowField 계산 ---
    m_CorridorField.Build(*m_Grid, goalCell, mask);


    m_HasGoal = true;
    InitGroupMovement();   // m_StartCells 재사용
    return true;
}


void NpcManager::Update(float dt)
{
    if (false == m_HasGoal) return;

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
            AdvanceCell(i, dt);   // 도착 -> 셀 전환
        }
        else
        {
            //if (true == WouldCollideTrajectory(i))   continue;  // 캡슐 충돌
            Math::Vector3 d = Math::Normalize(delta);
            m_Move.position[i] = pos + d * (NPC_SPEED * dt);
        }
        anyMoved = true;
    }

    // 누군가 움직였을때만 이동
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
}

// 현재는 이동 예약시 바로 기존 cell 예약을 풀어버림
// 이게 충돌 야기시, cell 예약 해제 시점을 조절할것
void NpcManager::AdvanceCell(size_t i, float dt)
{
    // 목표 셀에 정확히 스냅 (연속 이동 누적 오차 제거 + 층 플립 방지)
    m_Move.position[i] = m_Move.targetWorldPos[i];
    m_Move.currCell[i] = m_Move.targetCell[i];

    const auto& curr = m_Move.currCell[i];

    float currCost;
    if (false == m_CorridorField.SampleCost(*m_Grid, curr.x, curr.y, curr.z, currCost))
    {
        m_Move.active[i] = 0;
        return;
    }
    // 목적지 도달 (cost가 0에 가까움)
    if (currCost < 1e-4f)
    { 
        m_Move.active[i] = 0; 
        return;
    }


    // ---- 1순위: FlowField가 가리키는 방향 ----
    DirectX::XMFLOAT3 fdir;
    DirectX::XMINT3 desired{ 0, 0, 0 };
    bool hasDesired = false;
    if (m_CorridorField.SampleDirection(*m_Grid, curr.x, curr.y, curr.z, fdir))
    {
        if (fdir.x * fdir.x + fdir.y * fdir.y + fdir.z * fdir.z >= 1e-6f)
        {
            desired = { (int)std::round(fdir.x), (int)std::round(fdir.y), (int)std::round(fdir.z) };
            hasDesired = true;
        }
    }

    // 특정 이웃 셀이 지금 이동 가능한 유효 후보인지 검사하는 헬퍼
    // (cost 낮아짐 + 미점유 + 비교차). walkable 여부는 m_NeighborScratch에 있는지로 판단
    auto tryCandidate = [&](const DirectX::XMINT3& cand, DirectX::XMINT3& out) -> bool
    {
        // walkable 이웃 목록에 실제로 존재하는지 확인 (지형/코너컷 통과 보장)
        bool isNeighbor = false;
        for (const auto& n : m_NeighborScratch)
        {
            if (n.pos.x == cand.x && n.pos.y == cand.y && n.pos.z == cand.z) { isNeighbor = true; break; }
        }
        if (!isNeighbor)                                                        return false;

        float nc;
        if (false == m_CorridorField.SampleCost(*m_Grid, cand.x, cand.y, cand.z, nc)) return false; // flowfield 아님
        if (nc >= currCost)                                                    return false;    // 목적지로 안 가까워짐
        if (m_Reserve.Find(MakeCellKey(cand.x, cand.y, cand.z)) >= 0)          return false;    // 점유됨
        if (true == IsMoveCross(i, curr, cand))                               return false;     // 교차

        out = cand;
        return true;
    };

    m_NeighborScratch.clear();
    GetWalkableNeighbors(*m_Grid, curr, m_NeighborScratch);

    DirectX::XMINT3 best{ curr.x, curr.y, curr.z };
    bool found = false;

    // desired 방향의 목표 셀 계산 (y는 desired 그대로 사용)
    if (hasDesired)
    {
        DirectX::XMINT3 desiredCell{ curr.x + desired.x, curr.y + desired.y, curr.z + desired.z };

        // (1) 1순위 방향이 바로 가능하면 그대로
        if (tryCandidate(desiredCell, best))
        {
            found = true;
        }
        // (2) 1순위가 대각선인데 막혔으면 -> 성분 분해 대안
        else if (desired.x != 0 && desired.z != 0)
        {
            // 어느 축이 막혔는지: 두 카디널 성분 어깨셀의 점유 여부로 판단
            // x성분 어깨 = (curr.x+dx, curr.z),  z성분 어깨 = (curr.x, curr.z+dz)
            DirectX::XMINT3 zKeep{ curr.x,             curr.y + desired.y, curr.z + desired.z }; // x 버림 (직진 z)
            DirectX::XMINT3 xKeep{ curr.x + desired.x, curr.y + desired.y, curr.z };             // z 버림 (직진 x)

            // x성분 어깨셀이 점유됐으면(=x방향 충돌 유발) x를 버리고 zKeep 우선
            bool xShoulderBlocked = (m_Reserve.Find(MakeCellKey(curr.x + desired.x, curr.y, curr.z)) >= 0);
            bool zShoulderBlocked = (m_Reserve.Find(MakeCellKey(curr.x, curr.y, curr.z + desired.z)) >= 0);

            // 막힘 유발 축의 반대 성분을 우선 시도, 그 다음 나머지 성분
            if (xShoulderBlocked && !zShoulderBlocked)
            {
                if (tryCandidate(zKeep, best)) found = true;
                else if (tryCandidate(xKeep, best)) found = true;
            }
            else if (zShoulderBlocked && !xShoulderBlocked)
            {
                if (tryCandidate(xKeep, best)) found = true;
                else if (tryCandidate(zKeep, best)) found = true;
            }
            else
            {
                // 판별 애매(둘 다 막힘/둘 다 안 막힘) -> 둘 다 순서대로 시도
                if (tryCandidate(zKeep, best)) found = true;
                else if (tryCandidate(xKeep, best)) found = true;
            }
        }
    }

    // (3) 위에서 못 찾았으면 이웃 중 cost가 낮아지는 셀들을 모아, 예약 가능한 것 중 가장 좋은 걸 고름
    if (!found)
    {
        float bestCost = currCost;
        for (const auto& n : m_NeighborScratch)
        {
            float nc;
            if (false == m_CorridorField.SampleCost(*m_Grid, n.pos.x, n.pos.y, n.pos.z, nc)) continue; // flowfield 아님ㅁ
            if (nc >= currCost)                                                 continue;       // 목적지로 안 가까워짐
            if (m_Reserve.Find(MakeCellKey(n.pos.x, n.pos.y, n.pos.z)) >= 0)    continue;       // 점유됨
            if (true == IsMoveCross(i, curr, n.pos))                            continue;       // 교차

            int dx = n.pos.x - curr.x, dz = n.pos.z - curr.z;
            const auto& ld = m_Move.lastDir[i];
            float score = nc;
            if (dx == ld.x && dz == ld.z) score -= NPC_INER;    // 기존 방향으로 계속 가도록 보정값
            if (score < bestCost)
            {
                bestCost = score;
                best = n.pos;
                found = true;
            }
        }
    }

    if (!found)
    {
        // 갈 곳 없음 -> 제자리 대기 (다음 프레임 재시도)
        m_Move.targetCell[i] = curr;
        m_Move.targetWorldPos[i] = m_Move.position[i];
        return;
    }
    

    // 예약 전환
    if (m_Reserve.TryReserve(MakeCellKey(best), (int)i))
    {
        m_Reserve.Release(MakeCellKey(curr.x, curr.y, curr.z));

        m_Move.targetCell[i] = best;
        m_Move.lastDir[i] = { best.x - curr.x, best.y - curr.y, best.z - curr.z };
        m_Move.targetWorldPos[i] = GetNpcStandPos(best, m_Move.halfHeight[i]);
    }
    else
    {
        // 이론상 발생하면 안 되지만(루프에서 이미 미점유 확인), 방어적으로 처리
        // 예약 실패 시 !found와 동일하게 제자리 대기 -> 다음 프레임 재시도
        m_Move.targetCell[i] = curr;
        m_Move.targetWorldPos[i] = m_Move.position[i];
    }

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

