#pragma once
#include "NpcTypes.h"
#include "AStarPathfinder.h"
#include <DirectXMath.h>
#include <unordered_set>
#include <vector>
#include <memory>

// NpcManager 비대화에 따라 분리됨 - leaf(경로/flowfield 소유 단위) 구축과
// 병목에서의 자동 분리(TrySplitLeaf) 로직을 전담
class LeafSplitController
{
public:
    LeafSplitController(std::vector<std::unique_ptr<LeafGroup>>& leaves,
        NpcMoveData& move,
        NpcGroup& group,
        std::vector<DirectX::XMINT3>& startCells);

    bool BuildLeafCorridor(const VoxelGrid& grid, LeafGroup& leaf, const DirectX::XMINT3& goalCell,
        std::vector<DirectX::XMINT3>* outPath = nullptr);

    void CheckSplitTriggers(const VoxelGrid& grid);   // 이동 루프 종료 후 호출

private:
    // TODO : NpcManager 비대화에 따른 파일 분리 및 리팩토링 할것
    // --- 분리 제한 요소 ---
    static constexpr int    MIN_SPLIT_SIZE = 4;         // 분리 최소 인원
    static constexpr int    MAX_SPLIT_DEPTH = 2;        // 최대 분리 횟수
    static constexpr int    MAX_LEAVES      = 8;        // 메모리 상한(leaf당 flowfield 하나)
    static constexpr int    BOTTLENECK_SPREAD = 10;      // 병목시 단면 확보용 bfs 반경
    static constexpr float  PATH_OVERLAP_LIMIT = 0.5f; // (0.0~1.0) path 겹침 ratio -> 이 이상 겹치면 분할x
    static constexpr float  NEW_PATH_LIMIT = 10.0f;      // 우회경로가 n배 보다 길면 분할 포기
    static constexpr float  SPLIT_BLOCK_TIME = 2.0f;    // 이 시간 이상 막히면 분리 검토

    // a* 실행
    bool FindLeafPath(const VoxelGrid& grid, LeafGroup& leaf, const DirectX::XMINT3& goalCell,
        std::vector<DirectX::XMINT3>& outPath, DirectX::XMINT3& outCentroidCell);

    // margin / mask / field 구축
    void BuildLeafField(const VoxelGrid& grid, LeafGroup& leaf, const DirectX::XMINT3& goalCell,
        const DirectX::XMINT3& centroidCell, const std::vector<DirectX::XMINT3>& path);

    // 막힌 Npc들이 가려던 방향을 BFS로 넓혀서 병목 단면 수집용
    void CollectBottleneckCells(const VoxelGrid& grid, const std::vector<int>& stuckMem,
        std::unordered_set<int64_t>& outCells) const;

    // 막힌 멤버들을 새 leaf로 분리 시도 - 실패하면 false
    bool TrySplitLeaf(const VoxelGrid& grid, int leafIdx, const std::vector<int>& stuckMem);

    // leafId와 members를 항상 함께 갱신 — 둘 중 하나만 바뀌는 사고 방지
    void ReassignMember(int npcIdx, int fromLeaf, int toLeaf);

    bool AreSpatiallyClustered(const std::vector<int>& members) const;

    AStarPathfinder   m_Pathfinder;

    std::vector<std::unique_ptr<LeafGroup>>& m_Leaves;
    NpcMoveData& m_Move;
    NpcGroup& m_Group;
    std::vector<DirectX::XMINT3>& m_StartCells;
};
