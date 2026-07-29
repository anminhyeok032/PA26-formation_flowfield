#pragma once
#include "VoxelGrid.h"
#include "CorridorFlowField.h"   // CHUNK_SIZE 단일 진실
#include <DirectXMath.h>
#include <vector>
#include <unordered_map>
#include <cstdint>

// 청크 경계를 가로지르는 연속 walkable 구간 = 포탈.
// 청크 단위는 CorridorFlowField와 반드시 동일해야 함
// (포탈 경로를 마스크로 바꿀 때 같은 격자를 가정하므로)



class PortalGraph
{
public:
    static constexpr int CHUNK_SIZE = CorridorFlowField::CHUNK_SIZE;                // 8
    static constexpr int MAX_SLOTS = SurfaceChunk::SurfaceColumn::INLINE_CAPACITY;  // 4

    struct Portal
    {
        int64_t         chunkA, chunkB;   // 인접한 두 청크 (A = -X/-Z 쪽)
        DirectX::XMINT3 cellA, cellB;     // 대표 통과 셀(구간 중앙), 각 청크 쪽
        uint8_t         width;            // 구간 길이(셀 수) 추후 혼잡 용량 산정에 사용
    };

    // 맵 전체 스캔 -> 모든 내부 경계에서 포탈 추출
    void Build(const VoxelGrid& grid);

    const std::vector<Portal>& GetPortals() const { return m_Portals; }

    // 해당 청크에 접한 포탈 인덱스 목록. 없으면 nullptr.
    const std::vector<uint32_t>* FindChunkPortals(int64_t chunkKey) const;

    void ExtractCorner(const VoxelGrid& grid, int cx, int cz, int dirX, int dirZ);

    // 청크당 최대 N개 가정을 검증하기 위한 실측치
    uint32_t GetMaxPortalsPerChunk() const { return m_MaxPortalsPerChunk; }

    static int64_t ChunkKeyOf(int x, int z) { return CorridorFlowField::ChunkKeyOf(x, z); }

    struct Neighbor { uint32_t portal; float cost; };

    // 포탈 추출(1단계) 이후 호출 - 같은 청크에 속한 포탈끼리 실제 이동 거리로 연결
    void ConnectIntra(const VoxelGrid& grid);

    const std::vector<Neighbor>& GetNeighbors(uint32_t portalIndex) const
    {
        return m_Adjacency[portalIndex];
    }

    // 씨앗 청크들에서 포탈을 따라 marginChunks만큼 확장.
    // 포탈이 없는 방향(벽)으로는 퍼지지 않으므로 기존 셀 BFS의 안전성이 유지됨.
    std::unordered_set<int64_t> ExpandChunks(const std::vector<int64_t>& seedChunks,
        int marginChunks) const;

private:
    enum class Axis { X, Z, CornerNE, CornerSE };

    // (cx,cz) 청크의 +X 또는 +Z 경계 한 면에서 포탈을 추출
    void ExtractOnBoundary(const VoxelGrid& grid, int cx, int cz, Axis axis);
    void RegisterPortal(const Portal& portal);

    std::vector<Portal>                                 m_Portals;
    std::unordered_map<int64_t, std::vector<uint32_t>>  m_ChunkPortals;
    uint32_t                                            m_MaxPortalsPerChunk = 0;

    // 포탈은 두 청크에 걸쳐 있음 - 지금 계산 중인 청크 쪽 셀을 고름
    static DirectX::XMINT3 CellInChunk(const Portal& portal, int64_t chunkKey)
    {
        return (portal.chunkA == chunkKey) ? portal.cellA : portal.cellB;
    }

    void ConnectFromPortal(const VoxelGrid& grid, int64_t chunkKey, uint32_t fromPortal,
        const std::vector<uint32_t>& chunkPortals);

    std::vector<std::vector<Neighbor>> m_Adjacency;   // 포탈 인덱스 -> 이웃 포탈들
};
