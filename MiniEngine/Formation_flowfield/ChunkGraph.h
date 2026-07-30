#pragma once
#include "VoxelGrid.h"
#include "CorridorFlowField.h"   // CHUNK_SIZE 단일 진실
#include "ChunkKey.h"
#include <DirectXMath.h>
#include <vector>
#include <unordered_set>
#include <cstdint>

// 청크(8x8 xz) 단위 인접 그래프.
// 청크마다 1바이트 - 8방향 이웃 청크로 건너갈 수 있는가만 비트로 보관
// 정확한 통로 위치/거리는 담지 않는다. 그건 마스크가 만들어진 뒤
// CorridorFlowField가 셀 단위로 다시 계산하므로 여기서 중복 계산할 필요가 없다
class ChunkGraph
{
public:
    static constexpr int CHUNK_SIZE = CorridorFlowField::CHUNK_SIZE;
    static constexpr int DIR_COUNT = 8;

    static int64_t ChunkKeyOf(int x, int z)
    {
        return MakeChunkKey(x / CHUNK_SIZE, 0, z / CHUNK_SIZE);
    }

    // 지형 확정 직후 1회
    void Build(const VoxelGrid& grid);

    // 지형 편집 후. 편집 셀의 청크 + 8이웃만 재계산
    // (대각 연결이 코너 청크에 걸리므로 4이웃으로는 부족하다)
    void RefreshAround(const VoxelGrid& grid, const std::vector<DirectX::XMINT3>& editedCells);

    // 청크 단위 A*. 성공 시 시작~목적 청크키를 순서대로 채운다.
    bool FindChunkPath(int startX, int startZ, int goalX, int goalZ,
        std::vector<int64_t>& outChunkPath) const;

    // 씨앗 청크에서 연결된 방향으로만 marginChunks만큼 확장
    // 벽 쪽은 비트가 안 서 있으므로 자동 제외
    std::unordered_set<int64_t> ExpandChunks(const std::vector<int64_t>& seeds,
        int marginChunks) const;

    // 청크 집합 -> CorridorFlowField::Build가 받는 셀 마스크
    static std::unordered_set<int64_t> MaskCellsFromChunks(
        const VoxelGrid& grid, const std::unordered_set<int64_t>& chunks);

    // 셀 margin -> 청크 margin. margin이 청크 폭 이하면 경로 청크만으로 충분하다
    // (경로 청크 자체가 이미 8칸 폭을 확보하므로)
    static int MarginChunksFor(int marginCells)
    {
        return (marginCells <= CHUNK_SIZE) ? 0 : (marginCells + CHUNK_SIZE - 1) / CHUNK_SIZE;
    }

    // 디버그 시각화용. 청크 경로를 청크당 대표 셀 하나로 변환
    // 실제 이동에는 쓰이지 않으므로 대표 셀 선정은 근사로 충분
    static void ChunkPathToCells(const VoxelGrid& grid, const CorridorFlowField& field,
        const std::vector<int64_t>& chunkPath,
        std::vector<DirectX::XMINT3>& outCells);

private:
    // dir 방향으로 건널 수 있는 셀이 하나라도 있는지 (있으면 즉시 탈출)
    bool    HasCrossing(const VoxelGrid& grid, int cx, int cz, int dir) const;
    uint8_t ComputeLinks(const VoxelGrid& grid, int cx, int cz) const;

    std::vector<uint8_t> m_Links;   // 인덱스: cx + m_CountX * cz
    int m_CountX = 0;
    int m_CountZ = 0;
};
