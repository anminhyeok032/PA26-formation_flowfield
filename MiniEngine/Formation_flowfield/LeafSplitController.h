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
        const std::vector<DirectX::XMINT3>& cellSource,
        const std::vector<uint8_t>& stateSource) const;



private:
    // a* 실행
    bool FindLeafPath(const VoxelGrid& grid, const ChunkGraph& chunkGraph, LeafGroup& leaf,
        const DirectX::XMINT3& goalCell,
        std::vector<uint32_t>& outNodePath, DirectX::XMINT3& outCentroidCell);

    // margin / mask / field 구축
    void BuildLeafField(const VoxelGrid& grid, const ChunkGraph& chunkGraph, LeafGroup& leaf,
        const DirectX::XMINT3& goalCell, const DirectX::XMINT3& centroidCell,
        const std::vector<uint32_t>& nodePath);

    
    std::vector<std::unique_ptr<LeafGroup>>& m_Leaves;
    NpcMoveData& m_Move;
    NpcGroup& m_Group;
    std::vector<DirectX::XMINT3>& m_StartCells;
};
