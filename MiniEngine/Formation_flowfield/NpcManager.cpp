#include "NpcManager.h"
#include "ChunkKey.h"
#include <cmath>

void NpcManager::Init(const VoxelGrid& grid)
{
    m_Grid = &grid;

    const float voxelSize = grid.GetCellSize();
    const float NPC_WIDTH = voxelSize / 2.0f;        // 기존 상수와 동일
    const float NPC_HEIGHT = voxelSize * 3.0f / 2.0f;

    m_NpcInstances.clear();
    m_NpcInstances.reserve(100);

    constexpr int INTER_NPC = 1;
    // 지형 위에 10x10 격자로 100개 배치
    for (int i = 0; i < 100; i++)
    {
        for (int j = 0; j < 100; j++)
        {
            int gx = 100 + i * INTER_NPC;
            int gz = 100 + j * INTER_NPC;

            float surfY = (float)grid.GetSurfaceY(gx, gz) * voxelSize;

            NpcRenderer::InstanceData inst = {};
            inst.scaleXZ = NPC_WIDTH;
            inst.scaleY = NPC_HEIGHT;
            inst.position[0] = gx * voxelSize;
            inst.position[1] = surfY + (voxelSize / 2.0f) + inst.scaleY + 0.1f;
            inst.position[2] = gz * voxelSize;
            inst.colorType = 0;
            m_NpcInstances.push_back(inst);
        }
    }

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

    if (m_SelectedNpcIndex >= 0 && m_SelectedNpcIndex < (int)m_NpcInstances.size())
    {
        m_NpcInstances[m_SelectedNpcIndex].colorType = 0;
    }

    m_SelectedNpcIndex = (hitIndex == m_SelectedNpcIndex) ? -1 : hitIndex;
    if (m_SelectedNpcIndex >= 0)
    {
        m_NpcInstances[m_SelectedNpcIndex].colorType = 2;
    }

    NpcRenderer::UpdateInstances(m_NpcInstances);
    return m_SelectedNpcIndex >= 0;
}

//bool NpcManager::SetGroupDestination(const DirectX::XMINT3& goalCell, std::vector<DirectX::XMINT3>* outPath)
//{
//    m_HasGoal = false;
//    if (m_NpcInstances.empty()) return false;
//
//    // 그룹 대표 시작점: 첫 NPC(또는 무게중심) 위치의 walkable 셀
//    // (FlowField는 목적지 기준이라 시작점은 마스크 범위 결정용)
//    const auto& rep = m_NpcInstances[0];
//    Math::Vector3 repPos(rep.position[0], rep.position[1], rep.position[2]);
//
//    int sx, sy, sz;
//    if (!m_Grid->FindNearestWalkable(repPos, sx, sy, sz)) return false;
//
//    std::vector<DirectX::XMINT3> path;
//    if (!m_Pathfinder.FindPath(*m_Grid, { sx,sy,sz }, goalCell, path)) return false;
//    if (outPath) *outPath = path;   // 경로 시각화를 위한 배출
//
//    int memberCount = (int)m_NpcInstances.size();
//    int margin = ComputeMarginCells(memberCount);
//    auto mask = BuildLayerMask(*m_Grid, path, margin);
//    m_CorridorField.Build(*m_Grid, goalCell, mask);
//
//    m_HasGoal = true;
//    InitGroupMovement();   // 이동 상태 전체 초기화
//    return true;
//}

bool NpcManager::SetGroupDestination(const DirectX::XMINT3& goalCell,
    std::vector<DirectX::XMINT3>* outPath)
{
    m_HasGoal = false;
    const size_t n = m_NpcInstances.size();
    if (n == 0) return false;

    // ── 1. 각 NPC의 시작 셀 수집 (마스크 시드 + 이동 초기화에 공유) ──
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
    

    // ── 2. 무게중심 셀에서 A* (경로 대표성: 그룹 한가운데를 관통) ──
    int cx = (int)std::round(centroid.GetX());
    int cy = (int)std::round(centroid.GetY());
    int cz = (int)std::round(centroid.GetZ());
    int asx, asy, asz;
    Math::Vector3 centroidWorld = m_Grid->GetWorldPos(cx, cy, cz);
    if (!m_Grid->FindNearestWalkable(centroidWorld, asx, asy, asz)) return false;

    std::vector<DirectX::XMINT3> path;
    if (!m_Pathfinder.FindPath(*m_Grid, { asx,asy,asz }, goalCell, path)) return false;
    if (outPath) *outPath = path;

    // ── 3. margin을 "그룹 분포 반경"으로 계산 (시드 간 공백 방지) ──
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

    // ── 4. 마스크: A* 경로 + 모든 NPC 시작 셀을 시드로 ──
    auto mask = BuildLayerMask(*m_Grid, path, m_StartCells, margin);

    // ── 5. FlowField 계산 ──
    m_CorridorField.Build(*m_Grid, goalCell, mask);

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

    for (size_t i = 0; i < n; ++i)
    {
        if (!m_Move.active[i]) continue;

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
}

void NpcManager::InitGroupMovement()
{
    const size_t n = m_NpcInstances.size();
    m_Move.Resize(n);

    for (size_t i = 0; i < n; ++i)
    {
        //const auto& inst = m_NpcInstances[i];
        //m_Move.halfHeight[i] = inst.scaleY;   // 불변값 1회 복사

        //Math::Vector3 npcPos(inst.position[0], inst.position[1], inst.position[2]);

        //int sx, sy, sz;
        //if (!m_Grid->FindNearestWalkable(npcPos, sx, sy, sz))
        //{
        //    m_Move.active[i] = 0;   // 걸을 곳 없으면 이동 제외
        //    continue;
        //}

        //DirectX::XMINT3 startCell{ sx, sy, sz };
        //m_Move.currCell[i] = startCell;
        //m_Move.position[i] = GetNpcStandPos(startCell, inst.scaleY);

        //// 첫 목표 셀 조회
        //DirectX::XMFLOAT3 dir;
        //if (m_CorridorField.SampleDirection(*m_Grid, sx, sy, sz, dir) &&
        //    (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) >= 1e-6f)
        //{
        //    DirectX::XMINT3 next{ sx + (int)std::round(dir.x),
        //                          sy + (int)std::round(dir.y),
        //                          sz + (int)std::round(dir.z) };
        //    m_Move.targetCell[i] = next;
        //    m_Move.targetWorldPos[i] = GetNpcStandPos(next, inst.scaleY);
        //    m_Move.active[i] = 1;
        //}
        //else
        //{
        //    // 이미 목적지 위이거나 방향 없음 → 제자리 정지
        //    m_Move.targetCell[i] = startCell;
        //    m_Move.targetWorldPos[i] = m_Move.position[i];
        //    m_Move.active[i] = 0;
        //}

        m_Move.halfHeight[i] = m_NpcInstances[i].scaleY;

        const auto& startCell = m_StartCells[i];   // FindNearestWalkable 재호출 안 함
        if (startCell.x < 0) { m_Move.active[i] = 0; continue; }

        m_Move.currCell[i] = startCell;
        m_Move.position[i] = GetNpcStandPos(startCell, m_Move.halfHeight[i]);

        DirectX::XMFLOAT3 dir;
        if (m_CorridorField.SampleDirection(*m_Grid, startCell.x, startCell.y, startCell.z, dir) &&
            (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) >= 1e-6f)
        {
            DirectX::XMINT3 next{ startCell.x + (int)std::round(dir.x),
                                  startCell.y + (int)std::round(dir.y),
                                  startCell.z + (int)std::round(dir.z) };
            m_Move.targetCell[i] = next;
            m_Move.targetWorldPos[i] = GetNpcStandPos(next, m_Move.halfHeight[i]);
            m_Move.active[i] = 1;
        }
        else
        {
            m_Move.targetCell[i] = startCell;
            m_Move.targetWorldPos[i] = m_Move.position[i];
            m_Move.active[i] = 0;
        }
    }
}

void NpcManager::AdvanceCell(size_t i)
{
    // 목표 셀에 정확히 스냅 (연속 이동 누적 오차 제거 + 층 플립 방지)
    m_Move.position[i] = m_Move.targetWorldPos[i];
    m_Move.currCell[i] = m_Move.targetCell[i];

    const auto& cur = m_Move.currCell[i];
    DirectX::XMFLOAT3 dir;
    if (m_CorridorField.SampleDirection(*m_Grid, cur.x, cur.y, cur.z, dir))
    {
        if (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z >= 1e-6f)
        {
            DirectX::XMINT3 next{
                cur.x + (int)std::round(dir.x),
                cur.y + (int)std::round(dir.y),
                cur.z + (int)std::round(dir.z) };
            m_Move.targetCell[i] = next;
            m_Move.targetWorldPos[i] = GetNpcStandPos(next, m_Move.halfHeight[i]);
            return;
        }
    }
    m_Move.active[i] = 0;   // 목적지 도착 or 방향 없음 -> 정지
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
