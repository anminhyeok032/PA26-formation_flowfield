#pragma once
#include "NpcTypes.h"
#include "ChunkGraph.h"
#include "FieldBuildJob.h"
#include <DirectXMath.h>
#include <unordered_set>
#include <vector>
#include <memory>
#include <atomic>

// NpcManager 비대화에 따라 분리됨 - leaf(경로/flowfield 소유 단위) 구축과
// 병목에서의 자동 분리(TrySplitLeaf) 로직을 전담
class LeafSplitController
{
public:
    LeafSplitController(std::vector<std::unique_ptr<LeafGroup>>& leaves,
        NpcMoveData& move,
        NpcGroup& group,
        std::vector<DirectX::XMINT3>& startCells);

    bool BuildLeafCorridor(const VoxelGrid& grid, const ChunkGraph& chunkGraph,
        LeafGroup& leaf, const DirectX::XMINT3& goalCell,
        std::vector<uint32_t>* outNodePath = nullptr);

    //void CheckSplitTriggers(const VoxelGrid& grid);   // 이동 루프 종료 후 호출

    // 워커 스레드 호출 계산용
    // cancelFlag가 세워지면 return
    static FieldBuildResult RunBuild(const VoxelGrid& grid, const ChunkGraph& chunkGraph,
        const FieldBuildRequest& req, FieldMaskCache& cache,
        const std::atomic<bool>* cancelFlag);

    // 메인 스레드에서 요청을 구성. m_StartCells / leaf.members 스냅샷
    FieldBuildRequest MakeRequest(const LeafGroup& leaf, const DirectX::XMINT3& goal,
        uint64_t generation,
        const std::vector<DirectX::XMINT3>& cellSource) const;



private:
    // a* 실행
    bool FindLeafPath(const VoxelGrid& grid, const ChunkGraph& chunkGraph, LeafGroup& leaf,
        const DirectX::XMINT3& goalCell,
        std::vector<uint32_t>& outNodePath, DirectX::XMINT3& outCentroidCell);

    // margin / mask / field 구축
    void BuildLeafField(const VoxelGrid& grid, const ChunkGraph& chunkGraph, LeafGroup& leaf,
        const DirectX::XMINT3& goalCell, const DirectX::XMINT3& centroidCell,
        const std::vector<uint32_t>& nodePath);



    /*
    * // --- 분리 제한 요소 ---
    static constexpr int    MIN_SPLIT_SIZE = 4;         // 분리 최소 인원
    static constexpr int    MAX_SPLIT_DEPTH = 2;        // 최대 분리 횟수
    static constexpr int    MAX_LEAVES      = 8;        // 메모리 상한(leaf당 flowfield 하나)
    static constexpr int    BOTTLENECK_SPREAD = 10;      // 병목시 단면 확보용 bfs 반경
    static constexpr float  PATH_OVERLAP_LIMIT = 0.5f; // (0.0~1.0) path 겹침 ratio -> 이 이상 겹치면 분할x
    static constexpr float  NEW_PATH_LIMIT = 10.0f;      // 우회경로가 n배 보다 길면 분할 포기
    static constexpr float  SPLIT_BLOCK_TIME = 2.0f;    // 이 시간 이상 막히면 분리 검토

    // 막힌 Npc들이 가려던 방향을 BFS로 넓혀서 병목 단면 수집용
    void CollectBottleneckCells(const VoxelGrid& grid, const std::vector<int>& stuckMem,
        std::unordered_set<int64_t>& outCells) const;

    // 막힌 멤버들을 새 leaf로 분리 시도 - 실패하면 false
    bool TrySplitLeaf(const VoxelGrid& grid, int leafIdx, const std::vector<int>& stuckMem);

    // leafId와 members를 항상 함께 갱신 — 둘 중 하나만 바뀌는 사고 방지
    void ReassignMember(int npcIdx, int fromLeaf, int toLeaf);

    bool AreSpatiallyClustered(const std::vector<int>& members) const;

    */
    
    std::vector<std::unique_ptr<LeafGroup>>& m_Leaves;
    NpcMoveData& m_Move;
    NpcGroup& m_Group;
    std::vector<DirectX::XMINT3>& m_StartCells;
};
