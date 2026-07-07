#pragma once
#include "VoxelGrid.h"
#include <DirectXMath.h>
#include <vector>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <memory>

class CorridorFlowField
{
public:
    // goal에서 거꾸로 BFS를 돌리되, mask에 포함된 청크 밖으로는 확장하지 않음.
    void Build(const VoxelGrid& grid, const DirectX::XMINT3& goal, const std::unordered_set<int64_t>& mask);

    // (x,y,z) 셀의 이동 방향을 조회. 이 좌표가 마스크 밖(계산 안 됨)이면 false.
    bool SampleDirection(int x, int y, int z, DirectX::XMFLOAT3& outDir) const;

private:
    struct FlowFieldRecord
    {
        float             cost = FLT_MAX;
        DirectX::XMFLOAT3 direction{ 0.0f, 0.0f, 0.0f };
        bool              visited = false;
    };

    struct FlowFieldChunk
    {
        static constexpr int SIZE = 16;
        static constexpr int VOLUME = SIZE * SIZE * SIZE;

        // BFS(1단계)가 압도적으로 자주 읽고 쓰는 필드들을 별도 배열로 분리(SoA).
        // direction은 2단계(ComputeDirections)에서만 쓰이므로 따로 떼어내서,
        // BFS 도중 캐시라인에 불필요한 데이터가 섞여 들어오지 않게 함.
        std::array<float, VOLUME> cost;
        std::array<bool, VOLUME> visited;

        // 2단계에서만 쓰이는 "차가운(cold)" 데이터 - 별도 배열
        std::array<DirectX::XMFLOAT3, VOLUME> direction;

        FlowFieldChunk()
        {
            cost.fill(FLT_MAX);
            visited.fill(false);
            direction.fill(DirectX::XMFLOAT3(0, 0, 0));
        }
    };

    // AStarPathfinder와 동일하게 unique_ptr+unordered_map 방식
    std::unordered_map<int64_t, std::unique_ptr<FlowFieldChunk>> m_Chunks;

    FlowFieldChunk* FindOrCreateChunk(int x, int y, int z, int& outLocalIdx);
    const FlowFieldChunk* FindChunk(int x, int y, int z, int& outLocalIdx) const;

    void ComputeDirections(const VoxelGrid& grid);
};
