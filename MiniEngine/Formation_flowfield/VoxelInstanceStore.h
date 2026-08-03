#pragma once

#include "VoxelGrid.h"
#include "VoxelRenderer.h"
#include <vector>
#include <unordered_map>
#include <cstdint>

// 복셀 렌더 인스턴스와 그 색인들의 단일 소유자
// m_Instances / m_Coords / m_CoordToIndex / m_ChunkToIndices - 반드시 함께 갱신해야함
// 외부는 좌표/인덱스 조회 API로만 접근
class VoxelInstanceStore
{
public:
    void Initialize(const VoxelGrid* grid) { m_Grid = grid; }

    // 초기 전체 구축 (Startup 전용)
    void Build();

    // 지형 편집 델타를 국소만 반영 (swap-and-pop + append + 색인 갱신 + GPU 업로드)
    void ApplyDelta(const VoxelGrid::TerrainEditDelta& delta);

    // ---- 조회 ----
    int  Count() const { return (int)m_Instances.size(); }
    bool IsValidIndex(int index) const { return index >= 0 && index < (int)m_Instances.size(); }

    // 좌표 -> 인덱스 (없으면 -1)
    int FindIndex(int x, int y, int z) const;
    int FindIndex(DirectX::XMINT3 v) const;

    // 인덱스 -> 좌표 (IsValidIndex 확인 후 호출할 것)
    const VoxelGrid::CellCoord& CoordAt(int index) const { return m_Coords[index]; }
    int CoordCount() const { return (int)m_Coords.size(); }

    int ChunkIndicesCount() const { return (int)m_ChunkToIndices.bucket_count(); }

    // 청크 내 인덱스 목록 (없으면 nullptr)
    const std::vector<int>* IndicesInChunk(int64_t chunkKey) const;

    // ---- 색상 ----
    // 셀 타입 기반 기본색. 디버그 레이어(확정/경로/점유)는 상위 계층 소관.
    uint32_t GetBaseColorType(int index) const;
    void     SetColor(int index, uint32_t colorType) { m_Instances[index].colorType = colorType; }

    // ---- GPU ----
    // 변경 인덱스만 부분 업로드 (흩어져 있으면 전체 업로드로 폴백)
    void Flush(std::vector<int>& changedIndices);

private:
    //==== 인스턴스 배열 일부 갱신용 헬퍼(swap-and-pop / append) ====
    void RemoveInstanceAt(int idx, std::vector<int>& dirty);
    void AppendInstance(const VoxelGrid::TerrainEditDelta::AddedCell& cell, std::vector<int>& dirty);
    void EraseFromChunkIndex(int64_t chunkKey, int idx);
    void ReindexInChunkIndex(int64_t chunkKey, int oldIdx, int newIdx);
    void RebuildIndices();

    const VoxelGrid* m_Grid = nullptr;

    std::vector<VoxelRenderer::InstanceData> m_Instances;   // 렌더용 복셀 데이터 - (xyz),s,c
    std::vector<VoxelGrid::CellCoord>        m_Coords;      // 인덱스->xyz좌표 (각 인스턴스의 m_Instances와 1:1)

     // (x,y,z) 좌표 -> m_Instances 배열의 인덱스로 즉시 찾아가기 위한 색인
    std::unordered_map<int64_t, int>              m_CoordToIndex;
    // 청크 key / 청크 내부 복셀 인스턴스 인덱스 목록
    std::unordered_map<int64_t, std::vector<int>> m_ChunkToIndices;
};
