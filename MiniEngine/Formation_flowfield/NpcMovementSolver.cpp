#include "NpcMovementSolver.h"
#include "ChunkKey.h"
#include "NpcConstants.h"


NpcMovementSolver::NpcMovementSolver(NpcMoveData& move, CellReservation& reserve)
    : m_Move(move), m_Reserve(reserve)
{
}

//---------------------------------------------------------------------
//
//	이동시 규율
//
//---------------------------------------------------------------------
void NpcMovementSolver::AdvanceCell(const VoxelGrid& grid, 
    const std::vector<std::unique_ptr<LeafGroup>>& leaves,
    const CorridorFlowField* nearField,
    size_t i, float dt, bool chasing)
{
    const int lid = m_Move.leafId[i];
    if (lid < 0) { m_Move.active[i] = 0; return; }  // 미소속 방지

    const CorridorFlowField& farField = *leaves[lid]->field;

    SnapToTargetCell(i);
    const DirectX::XMINT3 curr = m_Move.currCell[i];   // 값 복사 - 이하 안 바뀜

    // ----- near / far 필드 중 하나만 갱신 -----
    const CorridorFlowField* pField = nullptr;
    float currCost = 0.0f;

    if (nearField && nearField->SampleCost(grid, curr.x, curr.y, curr.z, currCost))
    {
        pField = nearField;
    }
    // near 밖
    else if (farField.SampleCost(grid, curr.x, curr.y, curr.z, currCost))
    {
        pField = &farField;
    }
    else
    {
        // 두 필드 어디에도 없다
        if (chasing)
        {
            HoldPosition(i, curr);
            return;
        }
        m_Move.active[i] = 0;
        m_Move.stopReason[i] = 1;
        return;
    }
    
    const CorridorFlowField& field = *pField;

    if (currCost < 1e-4f)   // 목적지 도달
    {
        // 추격에는 도착 x 
        if (chasing) { HoldPosition(i, curr); return; }

        m_Move.active[i] = 0;
        m_Move.stopReason[i] = 0;
        return;
    }

    m_NeighborScratch.clear();
    GetWalkableNeighbors(grid, curr, m_NeighborScratch);

    DirectX::XMINT3 best{ curr.x, curr.y, curr.z };

    // 1순위(FlowField 방향) + 대기 판정 + 성분 분해 슬라이딩 담당
    bool waited = false;
    bool found = TryPrimaryDirection(grid, i, field, curr, currCost, best, waited, dt);
    if (waited) return;   // 1순위 방향 대기 중 - 이번 프레임은 여기서 끝

    // 2순위(cost 최소 + lastDir 정렬) 폴백
    bool hasActiveBlocker = false;
    if (!found)
    {
        found = PickFallbackCell(grid, i, field, curr, currCost, best, hasActiveBlocker);
    }

    if (!found)
    {
        if (!hasActiveBlocker && !chasing)
        {
            // 도착 확정: 더 가까워지는 길이 없고, 비켜줄 상대도 없음
            m_Move.active[i] = 0;
            m_Move.blockedTime[i] = 0;
            return;
        }
        HoldPosition(i, curr);   // 곧 비킬 상대에게 막힘 - 다음 프레임 재시도
        return;
    }

    CommitMove(grid, i, curr, best);

}

void NpcMovementSolver::AdvanceWanderCell(const VoxelGrid& grid, size_t i, float dt,
    const DirectX::XMINT3& anchor, int radius)
{
    SnapToTargetCell(i);    
    const DirectX::XMINT3 curr = m_Move.currCell[i];


    // 대기 중이면 아무것도 안함
    // 전부 매 프레임 안 움직이게, 개체별로 대기시간 뿌리기
    if (m_Move.stateTimer[i] > 0.0f)
    {
        m_Move.stateTimer[i] -= dt;
        HoldPosition(i, curr);
        return;
    }


    m_NeighborScratch.clear();
    GetWalkableNeighbors(grid, curr, m_NeighborScratch);
    if (m_NeighborScratch.empty()) 
    {
        HoldPosition(i, curr); 
        return; 
    }

    // anchor에서 멀어졌으면, anchor쪽으로 향하게
    const int dax = anchor.x - curr.x, daz = anchor.z - curr.z;
    const int currDistSq = dax * dax + daz * daz;
    const bool tooFar = currDistSq > (radius * radius);

    int candidates[8];
    int count = 0;
    for (size_t k = 0; k < m_NeighborScratch.size(); ++k)
    {
        const DirectX::XMINT3& p = m_NeighborScratch[k].pos;

        if (tooFar)
        {
            const int ndx = anchor.x - p.x, ndz = anchor.z - p.z;
            if (ndx * ndx + ndz * ndz >= currDistSq) continue;   // 더 멀어지면 제외
        }
        if (m_Reserve.Find(MakeCellKey(p)) >= 0)     continue;   // 점유됨
        if (IsMoveCross(i, curr, p))                 continue;   // 서로 통과 방지

        candidates[count++] = (int)k;
    }
    if (count == 0) { HoldPosition(i, curr); return; }

    const int pick = candidates[NextRand(m_Move.noiseSeed[i]) % (uint32_t)count];
    
    CommitMove(grid, i, curr, m_NeighborScratch[pick].pos);

    // 다음 셀까지 대기시간 재설정
    const float t = (float)(NextRand(m_Move.noiseSeed[i]) & 0xFFFF) / 65535.0f;
    m_Move.stateTimer[i] = WANDER_PAUSE_MIN_SEC + t * (WANDER_PAUSE_MAX_SEC - WANDER_PAUSE_MIN_SEC);

}

bool NpcMovementSolver::AdvanceReturnCell(const VoxelGrid& grid, size_t i,
    const DirectX::XMINT3& anchor)
{
    SnapToTargetCell(i);
    const DirectX::XMINT3 curr = m_Move.currCell[i];

    const int dx = anchor.x - curr.x, dz = anchor.z - curr.z;
    const int currDistSq = dx * dx + dz * dz;


    // 도착 판정은 호출자가 반경으로 
    if (currDistSq == 0) { HoldPosition(i, curr); return true; }   // 도달

    m_NeighborScratch.clear();
    GetWalkableNeighbors(grid, curr, m_NeighborScratch);

    // 필드 없이 anchor에 가장 가까워지는 이웃 하나.
    // 복귀에 필드를 쓰면 목표가 개체마다 달라져 필드가 N개 필요해진다
    int bestDistSq = currDistSq;
    const DirectX::XMINT3* best = nullptr;
    for (const auto& n : m_NeighborScratch)
    {
        if (m_Reserve.Find(MakeCellKey(n.pos)) >= 0) continue;
        if (IsMoveCross(i, curr, n.pos))             continue;

        const int ndx = anchor.x - n.pos.x, ndz = anchor.z - n.pos.z;
        const int d = ndx * ndx + ndz * ndz;
        if (d < bestDistSq) { bestDistSq = d; best = &n.pos; }
    }

    // 막힘 - 호출측이 타임아웃 처리
    if (best == nullptr)
    {
        HoldPosition(i, curr); 
        return false;
    }  

    CommitMove(grid, i, curr, *best);
    return true;
}

bool NpcMovementSolver::AdvanceFleeCell(const VoxelGrid& grid, size_t i, const DirectX::XMINT3& from)
{
    SnapToTargetCell(i);
    const DirectX::XMINT3 curr = m_Move.currCell[i];

    const int dx = from.x - curr.x, dz = from.z - curr.z;
    const int currDistSq = dx * dx + dz * dz;

    m_NeighborScratch.clear();
    GetWalkableNeighbors(grid, curr, m_NeighborScratch);

    // 자극원에서 가장 멀어지는 이웃. 비교 방향만 AdvanceReturnCell과 반대다.
    int bestDistSq = currDistSq;
    const DirectX::XMINT3* best = nullptr;
    for (const auto& n : m_NeighborScratch)
    {
        if (m_Reserve.Find(MakeCellKey(n.pos)) >= 0) continue;
        if (IsMoveCross(i, curr, n.pos))             continue;

        const int ndx = from.x - n.pos.x, ndz = from.z - n.pos.z;
        const int d = ndx * ndx + ndz * ndz;
        if (d > bestDistSq) { bestDistSq = d; best = &n.pos; }
    }

    // 막힘 - 벽이나 다른 개체에 몰린 상태
    // PANIC은 타이머로 빠져나가므로 영구 정지 안된다
    if (best == nullptr)
    {
        HoldPosition(i, curr);
        return false;
    }

    CommitMove(grid, i, curr, *best);
    return true;
}

void NpcMovementSolver::SnapToTargetCell(size_t i)
{
    m_Move.position[i] = m_Move.targetWorldPos[i];
    m_Move.currCell[i] = m_Move.targetCell[i];
}

void NpcMovementSolver::HoldPosition(size_t i, const DirectX::XMINT3& curr)
{
    m_Move.targetCell[i] = curr;
    m_Move.targetWorldPos[i] = m_Move.position[i];
}


bool NpcMovementSolver::TryPrimaryDirection(const VoxelGrid& grid, size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
    DirectX::XMINT3& best, bool& outWaited, float dt)
{
    const float NPC_CARDINAL_WAIT_SECONDS = (grid.GetCellSize() / NPC_SPEED) * NPC_WAIT_RATIO;
    const float NPC_DIAGONAL_WAIT_SECONDS = (grid.GetCellSize() * 1.41421356f / NPC_SPEED) * NPC_WAIT_RATIO;


    outWaited = false;

    DirectX::XMINT3 desired;
    if (false == field.SampleDirection(grid, curr.x, curr.y, curr.z, desired))
        return false;

    // 저장된 y는 사용하지 않음 - Climb 때문에 지형으로 해석
    DirectX::XMINT3 desiredCell;
    // y값 주변 통해서 찾기
    if(false == ResolveNeighborCell(curr.x + desired.x, curr.z + desired.z, desiredCell))
        return false;

    // (1) 1순위 방향이 바로 가능하면 그대로
    if (AcceptCell(grid, i, field, curr, currCost, desiredCell)) 
    {
        best = desiredCell; 
        return true; 
    }

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
            //m_Move.congestionTime[i] += dt;
            HoldPosition(i, curr);
            outWaited = true;
            return false;
        }
    }

    // (3) 1순위가 대각선인데 막혔으면 -> 성분 분해 대안
    if (desired.x == 0 || desired.z == 0)   return false;

    // 어깨셀 분석
    DirectX::XMINT3 zKeep, xKeep;   // x 버림(z직진) // z 버림(x직진)
    // 각 어깨 정확한 위치 받기
    const bool hasZ = ResolveNeighborCell( curr.x, curr.z + desired.z, zKeep);
    const bool hasX = ResolveNeighborCell(curr.x + desired.x, curr.z, xKeep);
    // 어깨 셀 예약 확인
    const bool zBlocked = hasZ && (m_Reserve.Find(MakeCellKey(zKeep)) >= 0);
    const bool xBlocked = hasX && (m_Reserve.Find(MakeCellKey(xKeep)) >= 0);
   
    // 막힌 곳 반대 우선 시도
    const DirectX::XMINT3* order[2] = { &zKeep, &xKeep };
    bool valid[2] = { hasZ, hasX };
    if (zBlocked && !xBlocked)
    {
        order[0] = &xKeep;
        order[1] = &zKeep;
        valid[0] = hasX;
        valid[1] = hasZ;
    }

    // 갈 수 있는 어깨 셀 우선 검사
    for (int k = 0; k < 2; ++k)
    {
        if (!valid[k])   continue;
        if (AcceptCell(grid, i, field, curr, currCost, *order[k]))
        {
            best = *order[k];
            return true;
        }
    }

    return false;
}

bool NpcMovementSolver::PickFallbackCell(const VoxelGrid& grid, size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
    DirectX::XMINT3& best, bool& outHasActiveBlocker)
{
    outHasActiveBlocker = false;    // 점유자 다 죽어있으면 도착 판정용
    float bestCost = currCost;
    bool found = false;

    for (const auto& n : m_NeighborScratch)
    {
        float nc;
        if (false == field.SampleCost(grid, n.pos.x, n.pos.y, n.pos.z, nc)) continue;
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


void NpcMovementSolver::CommitMove(const VoxelGrid& grid, size_t i, const DirectX::XMINT3& curr,
    const DirectX::XMINT3& best)
{
    if (m_Reserve.TryReserve(MakeCellKey(best), (int)i))
    {
        m_Reserve.Release(MakeCellKey(curr.x, curr.y, curr.z));

        m_Move.targetCell[i] = best;
        m_Move.lastDir[i] = { best.x - curr.x, best.y - curr.y, best.z - curr.z };
        m_Move.targetWorldPos[i] = GetNpcStandPos(grid, best, m_Move.halfHeight[i]);
        m_Move.blockedTime[i] = 0;   // 이동 성공해서 대기 프레임 초기화
    }
    else
    {
        // 이론상 발생하면 안 되지만(루프에서 이미 미점유 확인), 방어적으로 처리
        HoldPosition(i, curr);
    }
}

Math::Vector3 NpcMovementSolver::GetNpcStandPos(const VoxelGrid& grid, const DirectX::XMINT3& cell, float halfHeight) const
{
    Math::Vector3 cellCenter = grid.GetWorldPos(cell.x, cell.y, cell.z);
    float cellSize = grid.GetCellSize();

    // Startup()의 배치 공식과 동일: 셀 중심 -> 셀 윗면(절반) -> NPC 반높이만큼 더 위로
    float standY = cellCenter.GetY() + (cellSize * 0.5f) + halfHeight + 0.1f;

    return Math::Vector3(cellCenter.GetX(), standY, cellCenter.GetZ());
}


// y가 2비트로 표현해서 climb을 못 담는다
// 따라서 주변 값들중에 같은거 찾아서 반환
bool NpcMovementSolver::ResolveNeighborCell(int nx, int nz, DirectX::XMINT3& out) const
{
    for (const auto& n : m_NeighborScratch)
    {
        if(n.pos.x == nx && n.pos.z == nz)  
        {
            out = n.pos;
            return true;
        }
    }
    return false;
}
// 이웃 목록에서 이미 해석된 셀 검사
bool NpcMovementSolver::AcceptCell(const VoxelGrid& grid, size_t i, 
    const CorridorFlowField& field, const DirectX::XMINT3& curr, 
    float currCost, const DirectX::XMINT3& cell) const
{
    float nc;
    if (false == field.SampleCost(grid, cell.x, cell.y, cell.z, nc)) return false;  // flowfield 아님
    if (nc >= currCost)                                              return false;  // 목적지 안가까워짐
    if (m_Reserve.Find(MakeCellKey(cell)) >= 0)                      return false;  // 예약됨
    if (true == IsMoveCross(i, curr, cell))                          return false;  // 교차함
    return true;
}

bool NpcMovementSolver::IsMoveCross(size_t i, const DirectX::XMINT3& curr, const DirectX::XMINT3& next) const
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
