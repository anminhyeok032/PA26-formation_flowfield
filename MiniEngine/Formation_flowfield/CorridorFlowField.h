#pragma once
#include "VoxelGrid.h"
#include "PathCorridor.h"
#include "ChunkKey.h"
#include <DirectXMath.h>
#include <vector>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <atomic>

class CorridorFlowField
{

private:

    // cell당 17Byte
    // 
    struct FlowFieldChunk
    {
        static constexpr int SIZE = CHUNK_SIZE;
        static constexpr int AREA = SIZE * SIZE; // 256
        static constexpr int MAX_SLOTS = SurfaceChunk::SurfaceColumn::INLINE_CAPACITY;

        // 방향 없음(목적지 자신 or 더 낮은 이웃 부재) ColumnData 생성자가 참조하므로 여기 둠
        static constexpr uint8_t kNoDir = 0xFFu;

        // 각 xz + y_slot좌표의 SoA구조 - cost, visited, dir값
        struct ColumnData
        {
            std::array<float, MAX_SLOTS>                cost;
            // cold 데이터 - flowfield 값
            //     진행 방향을 (dx,dy,dz) 각 -1/0/1로 2비트씩 packing한 1바이트 - 2비트 남았음
            //     실수 단위벡터가 필요한 소비처(화살표 시각화)는 조회 시점에 정규화
            std::array<uint8_t, MAX_SLOTS>              direction;

            ColumnData()
            {
                cost.fill(FLT_MAX);
                direction.fill(kNoDir);
            }
            
            // 다익스트라가 해당 슬롯 이미 확정했는지 확인용
            bool IsReached(int slot) const { return cost[slot] != FLT_MAX; }
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
    void Build(const VoxelGrid& grid, const DirectX::XMINT3& goal, 
        const std::unordered_set<int64_t>& mask,
        const std::atomic<bool>* cancelFlag = nullptr);

    // (x,y,z) 셀의 이동 방향을 조회. 이 좌표가 마스크 밖(계산 안 됨)이면 false.
    // const는 위치 조회로 새로운 청크 할당 방지
    bool SampleDirection(const VoxelGrid& grid, int x, int y, int z, DirectX::XMINT3& outDir) const;
    // (x,y,z) 셀의 flowfield 비용을 조회. 이 좌표가 마스크 밖(계산 안 됨)이면 false.
    bool SampleCost(const VoxelGrid& grid, int x, int y, int z, float& outCost) const;

    // 해당 (xyz)가 실제로 방문되었는지 검사
    bool IsVisited(const VoxelGrid& grid, int x, int y, int z) const;

    // 청크 순회용 getter
    const std::unordered_map<int64_t, std::unique_ptr<FlowFieldChunk>>& GetChunks() const { return m_Chunks; }
   
};
