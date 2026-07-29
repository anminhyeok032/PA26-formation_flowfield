#include "LeafSplitController.h"
#include "ChunkKey.h"
#include "GridNeighbors.h"
#include "PathCorridor.h"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>

constexpr float NPC_SPEED = 1.5f;
constexpr float VOXEL_SIZE_REF = 0.5f;   // VoxelGrid::GetCellSize()와 반드시 일치해야 함

// 이 칸 수만큼 이동할 시간 동안 못 움직이면 분리 검토.
// waitLimit(0.3칸)의 20배 — 성분 분해 폴백까지 반복 실패한 상태를 의미한다
constexpr float NPC_SPLIT_WAIT_CELLS = 0.1f;
constexpr float NPC_SPLIT_WAIT_SECONDS = (VOXEL_SIZE_REF / NPC_SPEED) * NPC_SPLIT_WAIT_CELLS;


LeafSplitController::LeafSplitController(std::vector<std::unique_ptr<LeafGroup>>& leaves,
    NpcMoveData& move, NpcGroup& group, std::vector<DirectX::XMINT3>& startCells)
    : m_Leaves(leaves), m_Move(move), m_Group(group), m_StartCells(startCells)
{
}

bool LeafSplitController::BuildLeafCorridor(const VoxelGrid& grid, LeafGroup& leaf, const DirectX::XMINT3& goalCell, std::vector<DirectX::XMINT3>* outPath)
{
    if (leaf.members.empty()) return false;

    std::vector<DirectX::XMINT3> path;
    DirectX::XMINT3 centroidCell;
    if (!FindLeafPath(grid, leaf, goalCell, path, centroidCell)) return false;
    if (outPath) *outPath = path;

    BuildLeafField(grid, leaf, goalCell, centroidCell, path);
    return true;
}


bool LeafSplitController::FindLeafPath(const VoxelGrid& grid, LeafGroup& leaf, const DirectX::XMINT3& goalCell,
    std::vector<DirectX::XMINT3>& outPath, DirectX::XMINT3& outCentroidCell)
{
    if (leaf.members.empty()) return false;

    // 1 - leaf 멤버 시작 셀 + 무게중심
    Math::Vector3 centroid(0.0f, 0.0f, 0.0f);
    int validCount = 0;

    for (int idx : leaf.members)
    {
        const auto& c = m_StartCells[idx];
        if (c.x < 0) continue;
        centroid += Math::Vector3((float)c.x, (float)c.y, (float)c.z);
        validCount++;
    }
    if (validCount == 0) return false;
    centroid = Math::Vector3(centroid) / (float)validCount;

    // 2 - 무게중심 셀에서 A*
    int cx = (int)std::round(centroid.GetX());
    int cy = (int)std::round(centroid.GetY());
    int cz = (int)std::round(centroid.GetZ());

    int asx, asy, asz;
    Math::Vector3 centroidWorld = grid.GetWorldPos(cx, cy, cz);
    if (!grid.FindNearestWalkable(centroidWorld, asx, asy, asz)) return false;

    std::vector<DirectX::XMINT3> path;
    if (!m_Pathfinder.FindPath(grid, { asx, asy, asz }, goalCell, path, leaf.excluded))
        return false;

    outPath = path;
    outCentroidCell = { cx, cy, cz };
    return true;
}

void LeafSplitController::BuildLeafField(const VoxelGrid& grid, LeafGroup& leaf, const DirectX::XMINT3& goalCell,
    const DirectX::XMINT3& centroidCell, const std::vector<DirectX::XMINT3>& path)
{
    const int cx = centroidCell.x, cz = centroidCell.z;

    // 3 - margin: leaf 인원 + leaf 분포 반경
    int maxDistFromCentroid = 0;
    for (int idx : leaf.members)
    {
        const auto& c = m_StartCells[idx];
        if (c.x < 0) continue;
        int dx = c.x - cx, dz = c.z - cz;
        int d = (int)std::round(std::sqrt((float)(dx * dx + dz * dz)));
        maxDistFromCentroid = std::max(maxDistFromCentroid, d);
    }

    int formationMargin = ComputeMarginCells((int)leaf.members.size());
    int margin = std::max(formationMargin, maxDistFromCentroid) + 2;

    // 4 - 마스크: A* 경로 + leaf 멤버 시작 셀만 시드로
    std::vector<DirectX::XMINT3> leafSeeds;
    leafSeeds.reserve(leaf.members.size());
    for (int idx : leaf.members)
    {
        if (m_StartCells[idx].x >= 0) leafSeeds.push_back(m_StartCells[idx]);
    }

    auto mask = BuildLayerMask(grid, path, leafSeeds, margin);

    // 5 - FlowField 계산
    leaf.field.Build(grid, goalCell, mask);
}

void LeafSplitController::CollectBottleneckCells(const VoxelGrid& grid, const std::vector<int>& stuckMem, std::unordered_set<int64_t>& outCells) const
{
    std::unordered_set<int64_t> standing;
    for (int idx : stuckMem)
    {
        const auto& c = m_Move.currCell[idx];
        standing.insert(MakeCellKey(c));
    }

    // 1 - 각자 가려던 cell을 시드로
    std::queue<DirectX::XMINT3> q;
    std::unordered_map<int64_t, int> dist;

    for (int idx : stuckMem)
    {
        const int lid = m_Move.leafId[idx];
        if (lid < 0) continue;

        const auto& curr = m_Move.currCell[idx];
        DirectX::XMINT3 d;
        if (false == m_Leaves[lid]->field.SampleDirection(grid, curr.x, curr.y, curr.z, d))
            continue;

        DirectX::XMINT3 want{ curr.x + d.x, curr.y + d.y, curr.z + d.z };

        // desired 셀이 진짜 지형 병목인지, 다른 NPC 때문인지 구분
        //bool isTerrainBlocked = !grid.IsWalkable(want.x, want.y, want.z);
        //// walkable인데 못 갔다면 NPC 혼잡 - 이건 벽으로 취급하면 안 됨
        //if (!isTerrainBlocked) continue;   // 병목 seed에서 제외

        int64_t key = MakeCellKey(want);
        if (standing.count(key) > 0) continue;

        if (dist.emplace(key, 0).second)
        {
            outCells.insert(key);
            q.push(want);
        }

    }

    // 2 - BFS로 병목 주변 까지 확장해서 막음 (안그럼 똑같은길로 감)
    std::vector<NeighborInfo> neighbors;
    while (!q.empty())
    {
        DirectX::XMINT3 curr = q.front();
        q.pop();

        int currDist = dist[MakeCellKey(curr)];
        if (currDist >= BOTTLENECK_SPREAD)   continue;  // 정해진 만큼만 병목 벽 확장하기

        GetWalkableNeighbors(grid, curr, neighbors);
        for (const auto& n : neighbors)
        {
            int64_t nKey = MakeCellKey(n.pos);
            if (standing.count(nKey) > 0)    continue;  // npc(자신)가 서있는 길은 안막기

            if (dist.emplace(nKey, currDist + 1).second)
            {
                outCells.insert(nKey);
                q.push(n.pos);
            }
        }
    }
}

bool LeafSplitController::TrySplitLeaf(const VoxelGrid& grid, int leafIdx, const std::vector<int>& stuckMem)
{
    LeafGroup& parent = *m_Leaves[leafIdx];

    // --- 분할 조건 : 연쇄 분리 / 깊이 / 메모리 상한 ---
    if ((int)parent.members.size() <= MIN_SPLIT_SIZE)       return false;
    if (parent.depth >= MAX_SPLIT_DEPTH)                    return false;
    if ((int)m_Leaves.size() >= MAX_LEAVES)                 return false;
    if (stuckMem.empty())                                   return false;
    if (stuckMem.size() >= parent.members.size())           return false;

    // 병목 식별
    std::unordered_set<int64_t> bottleneck;
    CollectBottleneckCells(grid, stuckMem, bottleneck);
    if (bottleneck.empty())  return false;

    // 임시 leaf 구성
    auto tmpLeafPtr = std::make_unique<LeafGroup>();
    LeafGroup& child = *tmpLeafPtr;
    child.leafId = (int)m_Leaves.size();
    child.parentId = parent.parentId;
    child.depth = parent.depth + 1;
    child.members = stuckMem;
    child.excluded = parent.excluded;       // 상위 제한
    child.excluded.insert(bottleneck.begin(), bottleneck.end());

    // a* 만 먼저 판정
    std::vector<DirectX::XMINT3> tmpPath;
    DirectX::XMINT3 centroidCell;
    if (false == FindLeafPath(grid, child, m_Group.goal, tmpPath, centroidCell)) return false;

    // 1 - 같은 길인지 검사
    if (!parent.path.empty())
    {
        std::unordered_set<int64_t> oldSet;
        for (const auto& p : parent.path)
        {
            oldSet.insert(MakeCellKey(p));
        }

        int overlap = 0;
        for (const auto& p : tmpPath)
        {
            if (oldSet.count(MakeCellKey(p)) > 0)    overlap++;
        }

        float ratio = (float)overlap / (float)tmpPath.size();
        if (ratio >= PATH_OVERLAP_LIMIT) return false;  // 같은 통로로 가면 false
    }

    // 2 - 너무 멀리 우회시, 실패
    if (!parent.path.empty() && (float)tmpPath.size() > (float)parent.path.size() * NEW_PATH_LIMIT)
        return false;

    // 전부 통과시, 새 길로 : 필드 구축 + 멤버 이관
    child.path = tmpPath;
    BuildLeafField(grid, child, m_Group.goal, centroidCell, child.path);
    child.active = true;

    const int newIdx = child.leafId;
    m_Leaves.push_back(std::move(tmpLeafPtr));   // 이관 전에 반드시 먼저 등록

    for (int idx : stuckMem)
    {
        ReassignMember(idx, leafIdx, newIdx);
        m_Move.congestionTime[idx] = 0.0f;
    }
    m_Group.leafIds.push_back(newIdx);      // 부모한테 새 leaf 인식
    return true;
}


// leafId와 members를 항상 함께 갱신 — 둘 중 하나만 바뀌는 사고 방지
void LeafSplitController::ReassignMember(int npcIdx, int fromLeaf, int toLeaf)
{
    auto& from = m_Leaves[fromLeaf]->members;
    from.erase(std::remove(from.begin(), from.end(), npcIdx), from.end());

    m_Leaves[toLeaf]->members.push_back(npcIdx);
    m_Move.leafId[npcIdx] = toLeaf;
}

void LeafSplitController::CheckSplitTriggers(const VoxelGrid& grid)
{
    // 이동 루프 끝나고 호출 - 루프중 m_Leaves 재할당 금지
    // leaf 개수가 늘어날수 있으니 현재 개수 고정
    const int leafCount = (int)m_Leaves.size();

    for (int i = 0; i < leafCount; ++i)
    {
        LeafGroup& leaf = *m_Leaves[i];
        if (false == leaf.active)    continue;

        std::vector<int> stuck;
        for (int idx : leaf.members)
        {
            // a - 필드가 끊겨서 멈춘 npc
            if (m_Move.active[idx] == 0 && m_Move.stopReason[idx] == 1)
            {
                stuck.push_back(idx);
                continue;
            }
            // b - 혼잡해서 병목 대기한 npc
            if (m_Move.active[idx] != 0 && m_Move.congestionTime[idx] >= NPC_SPLIT_WAIT_CELLS)
            {
                stuck.push_back(idx);
            }
        }
        if (stuck.empty())   continue;

        // CheckSplitTriggers에서 TrySplitLeaf 호출 전
        // goal까지의 cost 내림차순 정렬 - 먼 쪽이 앞으로
        std::sort(stuck.begin(), stuck.end(), [&](int a, int b) {
            float ca = 0, cb = 0;
            const auto& pa = m_Move.currCell[a];
            const auto& pb = m_Move.currCell[b];
            leaf.field.SampleCost(grid, pa.x, pa.y, pa.z, ca);
            leaf.field.SampleCost(grid, pb.x, pb.y, pb.z, cb);
            return ca > cb;   // 먼 쪽 우선
            });

        // 뒤쪽 절반만 분리 대상 (상한도 함께 걸어 excluded 폭증 방지)
        const int splitCount = std::min((int)stuck.size() / 2, 100);
        if (splitCount < MIN_SPLIT_SIZE) continue;   // 너무 적으면 분리 무의미
        stuck.resize(splitCount);

        // 공간 응집 검사: 서로 다른 병목에서 막힌 NPC가 한 그룹이 되면 안 됨
        //if (false == AreSpatiallyClustered(stuck)) continue;

        if (true == TrySplitLeaf(grid, i, stuck))
        {
            // 성공 - 분리된 npc들 다시 활성화
            for (int idx : stuck)
            {
                m_Move.active[idx] = 1;
                m_Move.stopReason[idx] = 0;
                m_Move.congestionTime[idx] = 0.0f;
            }
        }
        else
        {
            // 실패 - 재시도 방지위해 대기 시간 리셋
            // TODO : 다른 우회로가 안생긴다고 가정할시, 리셋이 아닌 아예 재시도를 금지시킬것
            for (int idx : stuck)
            {
                if (m_Move.active[idx] != 0)
                {
                    m_Move.congestionTime[idx] = 0.0f;
                }
            }
        }
    }
}


bool LeafSplitController::AreSpatiallyClustered(const std::vector<int>& members) const
{
    if (members.size() <= 1) return true;

    // 무게중심에서 가장 먼 멤버까지의 거리가 임계 이내인지
    constexpr int CLUSTER_RADIUS = 8;   // 셀 단위

    float sx = 0, sy = 0, sz = 0;
    for (int idx : members)
    {
        const auto& c = m_Move.currCell[idx];
        sx += c.x; sy += c.y; sz += c.z;
    }
    const float n = (float)members.size();
    const int cx = (int)std::round(sx / n);
    const int cy = (int)std::round(sy / n);
    const int cz = (int)std::round(sz / n);

    for (int idx : members)
    {
        const auto& c = m_Move.currCell[idx];
        int dx = c.x - cx, dy = c.y - cy, dz = c.z - cz;
        if (dx * dx + dy * dy + dz * dz > CLUSTER_RADIUS * CLUSTER_RADIUS)
            return false;
    }
    return true;
}
