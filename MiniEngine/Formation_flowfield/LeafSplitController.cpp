#include "LeafSplitController.h"
#include "ChunkKey.h"
#include "GridNeighbors.h"
#include "PathCorridor.h"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include "NpcConstants.h"


namespace
{
    // 청크 추가할지, 전체 재구성할지 판별 함수
    bool CanUseMask(const FieldBuildRequest& req, const FieldMaskCache& cache)
    {
        if (req.mode != FieldBuildMode::ChaseIncremental)    return false;
        if (cache.IsEmpty())                                 return false;  // 처음 갱신
        if (cache.leafId != req.leafId)                      return false;  // 다른 그룹
        if (cache.generation != req.generation)              return false;  // 지형이 바뀜
        if (cache.hops <= 0)                                 return false;  // hop 초기화 실패

        // mask 추가의 상한까지만
        const size_t limit = (size_t)(cache.baseSize * CHASE_MASK_GROWTH_LIMIT);
        if (cache.mask.size() > limit)                      return false;   // 상한 초과시 다시 생성

        return true;
    }

    // 목적지 주변 버블 노드 집합에 합치기
    void AddGoalBubble(const VoxelGrid& grid, const ChunkGraph& chunkGraph,
        const DirectX::XMINT3& goalCell, std::unordered_set<uint32_t>& nodes)
    {
        const int goalNode = chunkGraph.NodeIdOf(grid, goalCell);
        if (goalNode < 0) return;

        auto bubble = chunkGraph.ExpandNodes({ (uint32_t)goalNode }, CHASE_GOAL_HOPS);
        nodes.insert(bubble.begin(), bubble.end());
    }
}




LeafSplitController::LeafSplitController(std::vector<std::unique_ptr<LeafGroup>>& leaves,
    NpcMoveData& move, NpcGroup& group, std::vector<DirectX::XMINT3>& startCells)
    : m_Leaves(leaves), m_Move(move), m_Group(group), m_StartCells(startCells) 
{
}

//------------------------
//
// 기초 길찾기 로직
//
//------------------------
bool LeafSplitController::BuildLeafCorridor(const VoxelGrid& grid, const ChunkGraph& chunkGraph,
    LeafGroup& leaf, const DirectX::XMINT3& goalCell,
    std::vector<uint32_t>* outNodePath)
{
    std::vector<uint32_t> nodePath;
    DirectX::XMINT3 centroidCell;
    if (!FindLeafPath(grid, chunkGraph, leaf, goalCell, nodePath, centroidCell)) return false;

    leaf.path = nodePath;
    if (outNodePath) *outNodePath = nodePath;

    BuildLeafField(grid, chunkGraph, leaf, goalCell, centroidCell, nodePath);
    return true;
}


bool LeafSplitController::FindLeafPath(const VoxelGrid& grid, const ChunkGraph& chunkGraph, LeafGroup& leaf,
    const DirectX::XMINT3& goalCell,
    std::vector<uint32_t>& outNodePath, DirectX::XMINT3& outCentroidCell)
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

    outCentroidCell = { cx, cy, cz };   // margin 계산용

    // 경로 출발점은 무게중심에 가장 가까운 실제 멤버 셀 사용
    DirectX::XMINT3 startCell{ -1, -1, -1 };
    int bestDistSq = INT_MAX;
    for (int idx : leaf.members)
    {
        const auto& c = m_StartCells[idx];
        if (c.x < 0) continue;

        const int dx = c.x - cx, dz = c.z - cz;
        const int d = dx * dx + dz * dz;
        if (d < bestDistSq) { bestDistSq = d; startCell = c; }
    }
    if (startCell.x < 0) return false;

    if (!chunkGraph.FindNodePath(grid, startCell, goalCell, outNodePath))
        return false;

    return true;
}

void LeafSplitController::BuildLeafField(const VoxelGrid& grid, const ChunkGraph& chunkGraph, LeafGroup& leaf,
    const DirectX::XMINT3& goalCell, const DirectX::XMINT3& centroidCell,
    const std::vector<uint32_t>& nodePath)
{
    // 3 - margin: leaf 인원 + leaf 분포 반경
    const int margin = ComputeMarginCells((int)leaf.members.size()) + 3;

    // 4 - 마스크: A* 경로 노드 + leaf 멤버가 서 있는 노드를 시드로
    std::vector<uint32_t> seeds = nodePath;
    for (int idx : leaf.members)
    {
        const auto& c = m_StartCells[idx];
        if (c.x < 0) continue;

        // 셀 -> 노드. 청크키로 넣으면 그 청크의 모든 성분이 시드가 되어
        // 멤버가 서 있지도 않은 층까지 확장의 출발점이 된다
        const int node = chunkGraph.NodeIdOf(grid, c);
        if (node >= 0) seeds.push_back((uint32_t)node);
    }

    auto nodes = chunkGraph.ExpandNodes(seeds, ChunkGraph::MarginChunksFor(margin));
    AddGoalBubble(grid, chunkGraph, goalCell, nodes);
    auto mask = chunkGraph.MaskCellsFromNodes(grid, nodes);

    // 5 - FlowField 계산
    auto built = std::make_shared<CorridorFlowField>();
    built->Build(grid, goalCell, mask);

    leaf.field = move(built);   // shared_ptr<CorridorFlowField> -> shared_ptr<const>
}

FieldBuildResult LeafSplitController::RunBuild(const VoxelGrid& grid, const ChunkGraph& chunkGraph, 
    const FieldBuildRequest& req, FieldMaskCache& cache, 
    const std::atomic<bool>* cancelFlag)
{
    FieldBuildResult res;
    res.leafId = req.leafId;
    res.generation = req.generation;

    const bool CanExpand = CanUseMask(req, cache);

    if (true == CanExpand)
    {
        // --- 기존 마스크에다 확장 마스크만 추가해서 다익스트라 ---
        const int goalNode = chunkGraph.NodeIdOf(grid, req.goalCell);
        if (goalNode < 0)    return res;        // 못가는 곳

        // margin 기존대로 확장
        std::vector<uint32_t> seeds{ (uint32_t)goalNode };
        auto nodes = chunkGraph.ExpandNodes(seeds, cache.hops);

        // 캐시 없는 노드만 골라서 셀로 전개
        std::unordered_set<uint32_t> fresh;
        for (uint32_t n : nodes)
        {
            if (cache.nodes.insert(n).second)    fresh.insert(n);
        }
        if (!fresh.empty())
        {
            auto addCells = chunkGraph.MaskCellsFromNodes(grid, fresh);
            cache.mask.insert(addCells.begin(), addCells.end());
        }
    }
    // --- 전체 재구성 - 최초 목적지 / 지형 변경 / 마스크 폭주 ---
    else
    {
        if (req.startCell.x < 0) return res;    // 유효멤버 x

        // 1- 성분 노드 A*
        if (!chunkGraph.FindNodePath(grid, req.startCell, req.goalCell, res.nodePath))
            return res;

        // 2 - margin : 폭 or 실제 분포 반경
        const int formationMargin = ComputeMarginCells(req.memberCount);
        const int marginCells = formationMargin + 3;

        // 3 - Seed : 경로 노드 + 멤버 서 있는 노드
        std::vector<uint32_t> seeds = res.nodePath;
        for (const auto& c : req.memberCells)
        {
            const int node = chunkGraph.NodeIdOf(grid, c);
            if (node >= 0) seeds.push_back((uint32_t)node);
        }

        auto nodes = chunkGraph.ExpandNodes(seeds, ChunkGraph::MarginChunksFor(marginCells));
        AddGoalBubble(grid, chunkGraph, req.goalCell, nodes);
        auto mask = chunkGraph.MaskCellsFromNodes(grid, nodes);

        // 캐시 재수립 - baseSize가 이후 증분의 폭주 판정 기준이 된다
        cache.nodes = std::move(nodes);
        cache.mask = std::move(mask);
        cache.generation = req.generation;
        cache.hops = CHASE_GOAL_HOPS;   // 확장도 같은 규칙으로 
        cache.baseSize = cache.mask.size();
        cache.leafId = req.leafId;
    }


    // 4 - Dijkstra + 방향 - (너무 느리면 여기 최적화 할것)
    auto built = std::make_shared<CorridorFlowField>();
    built->Build(grid, req.goalCell, cache.mask, cancelFlag);

    // 취소 시, field 버퍼 스왑 금지
    if (cancelFlag && cancelFlag->load(std::memory_order_relaxed))   return res;

    // Build는 goal 컬럼을 못 찾으면 조용히 빈 필드를 남긴다
    // 즉, 실패시, 옛날 필드 유지 / 증가된 마스크가 goal을 못담음 - cache 지워주기
    if (!built->IsVisited(grid, req.goalCell.x, req.goalCell.y, req.goalCell.z))
    {
        if (true == CanExpand)   cache.Clear();
        return res;
    }

    res.field = std::move(built);
    res.success = true;
    return res;
}

FieldBuildRequest LeafSplitController::MakeRequest(const LeafGroup& leaf, const DirectX::XMINT3& goal,
    uint64_t generation,
    const std::vector<DirectX::XMINT3>& cellSource,
    const std::vector<uint8_t>& stateSource) const
{
    FieldBuildRequest req;
    req.leafId = leaf.leafId;
    req.goalCell = goal;
    req.generation = generation;

    // 무게 중심
    Math::Vector3 centroid(0.0f, 0.0f, 0.0f);
    int validCount = 0;
    req.memberCells.reserve(leaf.members.size());

    for (int idx : leaf.members)
    {
        if (idx < 0 || idx >= (int)cellSource.size()) continue;   // 멤버 목록이 어긋난 경우 방어
        const auto& c = cellSource[idx];
        if (c.x < 0) continue;

        // Chase / Alerted만 넣기 - idle 넣으면 맵 전체
        const uint8_t st = stateSource[idx];
        if (st != NPC_STATE_CHASE && st != NPC_STATE_ALERTED)    continue;

        req.memberCells.push_back(c);   // 값 복사
        centroid += Math::Vector3((float)c.x, (float)c.y, (float)c.z);
        ++validCount;
    }

    if (validCount == 0) return req;

    req.memberCount = validCount;
    centroid = Math::Vector3(centroid) / (float)validCount;

    const int cx = (int)std::round(centroid.GetX());
    const int cy = (int)std::round(centroid.GetY());
    const int cz = (int)std::round(centroid.GetZ());

    // 출발점 선정 + 분포 반경을 한 루프에서
    // 원래는 FindLeafPath와 BuildLeafField가 각각 members를 돌았다 - 같은 데이터라 합친다
    int bestDistSq = INT_MAX;
    for (const auto& c : req.memberCells)
    {
        const int dx = c.x - cx, dz = c.z - cz;
        const int dsq = dx * dx + dz * dz;

        if (dsq < bestDistSq) 
        { 
            bestDistSq = dsq; 
            req.startCell = c; 
        }

        const int d = (int)std::round(std::sqrt((float)dsq));
        req.maxDistFromCentroid = std::max(req.maxDistFromCentroid, d);
    }
    return req;

}
