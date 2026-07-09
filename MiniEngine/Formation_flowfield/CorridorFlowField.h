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
    // Dijkstra를 이용한 flowfield 만들기
    void Build(const VoxelGrid& grid, const DirectX::XMINT3& goal, const std::unordered_set<int64_t>& mask);

    // (x,y,z) 셀의 이동 방향을 조회. 이 좌표가 마스크 밖(계산 안 됨)이면 false.
    // const는 위치 조회로 새로운 청크 할당 방지
    bool SampleDirection(int x, int y, int z, DirectX::XMFLOAT3& outDir) const;

private:
    struct FlowFieldRecord
    {
        float             cost = FLT_MAX;
        DirectX::XMFLOAT3 direction{ 0.0f, 0.0f, 0.0f };
        bool              visited = false;
    };


    // cell당 17Byte
    // 
    struct FlowFieldChunk
    {
        static constexpr int SIZE = 8;
        static constexpr int VOLUME = SIZE * SIZE * SIZE;

        // BFS에서 자주 읽고 쓰는 필드들 별도 배열로 분리 (SoA)
        std::array<float, VOLUME> cost;
        std::array<bool, VOLUME> visited;

        // cold 데이터 - flowfield 값
        std::array<DirectX::XMFLOAT3, VOLUME> direction;

        FlowFieldChunk()
        {
            cost.fill(FLT_MAX);
            visited.fill(false);
            direction.fill(DirectX::XMFLOAT3(0, 0, 0));
        }
    };

    // 그전에 찾은 청크를 캐싱해서 매번 해싱 오버헤드 방지
    struct ChunkCache
    {
        // -1 - 캐시안된 상태
        int64_t key = -1;
        FlowFieldChunk* chunk = nullptr;
    };

    // unique_ptr+unordered_map
    std::unordered_map<int64_t, std::unique_ptr<FlowFieldChunk>> m_Chunks;

    FlowFieldChunk* FindOrCreateChunk(int x, int y, int z, int& outLocalIdx, ChunkCache& cache);
    const FlowFieldChunk* FindChunk(int x, int y, int z, int& outLocalIdx, ChunkCache& cache) const;



    // 주변셀의 Cost를 비교해 낮아지는 칸을 경로로 지정
    void ComputeDirections(const VoxelGrid& grid);
};
