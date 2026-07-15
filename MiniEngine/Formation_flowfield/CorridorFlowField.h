#pragma once
#include "VoxelGrid.h"
#include "PathCorridor.h"
#include <DirectXMath.h>
#include <vector>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <memory>

class CorridorFlowField
{

private:

    // cell당 17Byte
    // 
    struct FlowFieldChunk
    {
        static constexpr int SIZE = 8;
        static constexpr int AREA = SIZE * SIZE; // 256
        static constexpr int MAX_SLOTS = SurfaceChunk::SurfaceColumn::INLINE_CAPACITY;

        // 각 xz + y_slot좌표의 SoA구조 - cost, visited, dir값
        struct ColumnData
        {
            std::array<float, MAX_SLOTS>                cost;
            std::array<bool, MAX_SLOTS>                 visited;
            // cold 데이터 - flowfield 값
            std::array<DirectX::XMFLOAT3, MAX_SLOTS>    direction;

            ColumnData()
            {
                cost.fill(FLT_MAX);
                visited.fill(false);
                direction.fill(DirectX::XMFLOAT3(0, 0, 0));
            }
        };

        std::array<ColumnData, AREA> columns;

        ColumnData& At(int lx, int lz)              { return columns[lx + SIZE * lz]; }
        const ColumnData& At(int lx, int lz) const  { return columns[lx + SIZE * lz]; }
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

    FlowFieldChunk::ColumnData* FindOrCreateColumn(const VoxelGrid& grid, int x, int y, int z,
        int& outSlotIdx, ChunkCache& cache);
    const FlowFieldChunk::ColumnData* FindColumn(const VoxelGrid& grid, int x, int y, int z,
        int& outSlotIdx, ChunkCache& cache) const;



    // 주변셀의 Cost를 비교해 낮아지는 칸을 경로로 지정
    void ComputeDirections(const VoxelGrid& grid);

public:
    // Dijkstra를 이용한 flowfield 만들기
    void Build(const VoxelGrid& grid, const DirectX::XMINT3& goal, const std::unordered_set<int64_t>& mask);

    // (x,y,z) 셀의 이동 방향을 조회. 이 좌표가 마스크 밖(계산 안 됨)이면 false.
    // const는 위치 조회로 새로운 청크 할당 방지
    bool SampleDirection(const VoxelGrid& grid, int x, int y, int z, DirectX::XMFLOAT3& outDir) const;

    // 해당 (xyz)가 실제로 방문되었는지 검사
    bool IsVisited(const VoxelGrid& grid, int x, int y, int z) const;

    // 청크 순회용 getter
    const std::unordered_map<int64_t, std::unique_ptr<FlowFieldChunk>>& GetChunks() const { return m_Chunks; }
    const int GetChunkSize() { return FlowFieldChunk::SIZE; }
};
