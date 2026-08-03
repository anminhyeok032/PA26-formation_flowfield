#include "NpcMovementSolver.h"
#include "ChunkKey.h"

constexpr float NPC_SPEED = 10.0f;
constexpr float NPC_INER = 0.4f;                // 다른 방향 찾아갈때, 기존 방향으로 가게 하는 보정값

constexpr float NPC_WAIT_RATIO = 0.2f;   // 1칸 움직이는데 비율 (x / 1.0f)%
constexpr float VOXEL_SIZE_REF = 0.5f;   // VoxelGrid::GetCellSize()와 반드시 일치해야 함
constexpr float NPC_CARDINAL_WAIT_SECONDS = (VOXEL_SIZE_REF / NPC_SPEED) * NPC_WAIT_RATIO;
constexpr float NPC_DIAGONAL_WAIT_SECONDS = (VOXEL_SIZE_REF * 1.41421356f / NPC_SPEED) * NPC_WAIT_RATIO;


NpcMovementSolver::NpcMovementSolver(NpcMoveData& move, CellReservation& reserve)
    : m_Move(move), m_Reserve(reserve)
{
}

//---------------------------------------------------------------------
//
//	이동시 규율
//
//---------------------------------------------------------------------
void NpcMovementSolver::AdvanceCell(const VoxelGrid& grid, const std::vector<std::unique_ptr<LeafGroup>>& leaves, size_t i, float dt)
{
    const int lid = m_Move.leafId[i];
    if (lid < 0) { m_Move.active[i] = 0; return; }  // 미소속 방지

    const CorridorFlowField& field = leaves[lid]->field;

    SnapToTargetCell(i);
    const DirectX::XMINT3 curr = m_Move.currCell[i];   // 값 복사 - 이하 안 바뀜

    float currCost;
    if (false == field.SampleCost(grid, curr.x, curr.y, curr.z, currCost))
    {
        m_Move.active[i] = 0;
        m_Move.stopReason[i] = 1;   // 지형변경으로 인한 필드 밖
        return;
    }
    if (currCost < 1e-4f)   // 목적지 도달
    {
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
        if (!hasActiveBlocker)
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

// 특정 이웃 셀이 지금 이동 가능한 유효 후보인지 검사
// (cost 낮아짐 + 미점유 + 비교차). walkable 여부는 m_NeighborScratch에 있는지로 판단
bool NpcMovementSolver::TryCandidate(const VoxelGrid& grid, size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost, const DirectX::XMINT3& cand, DirectX::XMINT3& out) const
{
    bool isNeighbor = false;
    for (const auto& n : m_NeighborScratch)
    {
        if (n.pos.x == cand.x && n.pos.y == cand.y && n.pos.z == cand.z) { isNeighbor = true; break; }
    }
    if (!isNeighbor) return false;

    float nc;
    if (false == field.SampleCost(grid, cand.x, cand.y, cand.z, nc)) return false; // flowfield 아님
    if (nc >= currCost)                                                    return false; // 목적지로 안 가까워짐
    if (m_Reserve.Find(MakeCellKey(cand.x, cand.y, cand.z)) >= 0)          return false; // 점유됨
    if (true == IsMoveCross(i, curr, cand))                                return false; // 교차

    out = cand;
    return true;
}

bool NpcMovementSolver::TryPrimaryDirection(const VoxelGrid& grid, size_t i, const CorridorFlowField& field, const DirectX::XMINT3& curr, float currCost,
    DirectX::XMINT3& best, bool& outWaited, float dt)
{
    outWaited = false;

    DirectX::XMINT3 desired;
    if (false == field.SampleDirection(grid, curr.x, curr.y, curr.z, desired))
        return false;

    DirectX::XMINT3 desiredCell{ curr.x + desired.x, curr.y + desired.y, curr.z + desired.z };

    // (1) 1순위 방향이 바로 가능하면 그대로
    if (TryCandidate(grid, i, field, curr, currCost, desiredCell, best)) return true;

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
            m_Move.congestionTime[i] += dt;
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

    if (TryCandidate(grid, i, field, curr, currCost, first, best)) return true;
    if (TryCandidate(grid, i, field, curr, currCost, second, best)) return true;
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


void NpcMovementSolver::CommitMove(const VoxelGrid& grid, size_t i, const DirectX::XMINT3& curr, const DirectX::XMINT3& best)
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
