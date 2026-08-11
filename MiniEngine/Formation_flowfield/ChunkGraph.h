#pragma once
#include "VoxelGrid.h"
#include "CorridorFlowField.h" 
#include "ChunkKey.h"
#include <DirectXMath.h>
#include <vector>
#include <unordered_set>
#include <cstdint>
#include <unordered_map>

// 청크(8x8 xz) 단위 라우팅 그래프.
// 노드는 청크가 아닌 청크 내부의 연결 성분
// 청크 하나가 벽이나 절벽으로 갈라져 있으면 성분이 둘 이상 생기고,
// 그 사이엔 간선이 없으므로 A*가 관통 경로를 만들지 못한다
//
// 거리는 담지 않는다(균등 비용 1 / sqrt2). 실제 경로는 마스크가 만들어진 뒤
// CorridorFlowField가 셀 단위로 다시 계산하므로 여기서 정밀도를 낼 이유가 없다
class ChunkGraph
{
public:
    static constexpr int MAX_SLOTS = SurfaceChunk::SurfaceColumn::INLINE_CAPACITY;
    static constexpr int DIR_COUNT = 8;
    static constexpr int LABELS_PER_CHUNK = CHUNK_SIZE * CHUNK_SIZE * MAX_SLOTS;   // 256
    static constexpr int SLOTS_PER_CHUNK = 4;

    // m_NodeOffset 배열 자체가 불필요
    static uint32_t NodeOffsetOf(int chunkIdx) { return (uint32_t)chunkIdx * SLOTS_PER_CHUNK; }



    void Build(const VoxelGrid& grid);

    // 지형 편집 후, 성분 개수가 그대로면 국소 갱신  바뀌면 전체 재빌드
    void RefreshAround(const VoxelGrid& grid, const std::vector<DirectX::XMINT3>& editedCells);

    // 셀 -> 노드 id  해당 청크를 그 자리에서 라벨링(저장하지 않음)
    // 갈 수 없는 셀이면 -1
    int NodeIdOf(const VoxelGrid& grid, const DirectX::XMINT3& cell) const;

    // 성분 노드 단위 A*  청크키가 아니라 노드 id를 반환 -
    // 청크키로 바꾸는 순간 어느 층으로 갈지가 사라져 마스크가 위아래를 모두 삼킨다
    bool FindNodePath(const VoxelGrid& grid,
        const DirectX::XMINT3& start, const DirectX::XMINT3& goal,
        std::vector<uint32_t>& outNodePath) const;

    // 마진만큼 주변 노드로 확장
    std::unordered_set<uint32_t> ExpandNodes(const std::vector<uint32_t>& seeds,
        int marginChunks) const;

    // 노드가 가리키는 성분에 속한 셀만 마스크에 넣는다
    // grid는 ExpandNodes 때와 같은 상태여야 한다 - 라벨 번호가 재현되지 않으면 성분이 어긋난다
    std::unordered_set<int64_t> MaskCellsFromNodes(
        const VoxelGrid& grid, const std::unordered_set<uint32_t>& nodes) const;

    static int MarginChunksFor(int marginCells)
    {
        return (marginCells <= CHUNK_SIZE) ? 0 : (marginCells + CHUNK_SIZE - 1) / CHUNK_SIZE;
    }

    // 실측용 - 성분이 2개 이상인 청크 수
    int GetSplitChunkCount() const;

    // 실측용 - 관측된 최대 성분 수.SLOTS_PER_CHUNK를 낮출 수 있는지 판단하는 근거.
    // 이 값이 SLOTS_PER_CHUNK를 넘으면 넘친 성분은 노드를 받지 못한 상태다
    int GetMaxCompPerChunk() const { return m_MaxCompPerChunk; }

    // 실측용 - 국소 갱신이 전체 재빌드로 폴백한 횟수
    int GetFullRebuildCount() const { return m_FullRebuildCount; }

    // 디버그 시각화용 - 노드 경로를 노드당 대표 셀 하나로 변환
    // 실제 이동에는 쓰이지 않으므로 대표 셀 선정은 근사로 충분
    void NodePathToCells(const VoxelGrid& grid, const CorridorFlowField& field,
        const std::vector<uint32_t>& nodePath,
        std::vector<DirectX::XMINT3>& outCells) const;

private:
    static int LocalIndex(int lx, int lz, int slot)
    {
        return (lx + CHUNK_SIZE * lz) * MAX_SLOTS + slot;
    }

    // 청크 하나를 flood fill로 라벨링. 성분 개수 반환
    int  LabelChunk(const VoxelGrid& grid, int cx, int cz, int16_t* outLabels) const;


    // 고정 stride라 역참조 배열 없이 나눗셈으로 되돌린다
    static int ChunkOfNode(uint32_t node) { return (int)(node / SLOTS_PER_CHUNK); }

    std::vector<uint16_t> m_CompCount;    // 청크당 성분 개수


    // 각 청크 면마다의 인접 리스트 (단방향)
    // m_NodeOffset[idx] + srcComp -> 출발노드의 id
    // m_NodeOffset[nChunkIdx]- 주변청크 인덱스 + dstComp-> 도착노드 id
    std::vector<std::vector<uint32_t>> m_Adjacency; 

    int m_FullRebuildCount = 0;
    int m_MaxCompPerChunk = 0;

    // 라벨 버퍼 접근. slotOf가 nullptr이면 청크 인덱스를 그대로 슬롯으로 사용(전체 빌드용)
    // 아니면 필요한 청크만 담은 조밀 버퍼에서 찾는다(증분 갱신용)
    static const int16_t* LabelsOf(int chunkIdx, const std::vector<int16_t>& labels,
        const std::unordered_map<int, int>* slotOf);

    // 이 청크에서 나가는 간선만 다시 만든다. allLabels는 최소 이 청크+8이웃을 포함해야 함
    void BuildEdgesForChunk(const VoxelGrid& grid, int cx, int cz,
        const std::vector<int16_t>& labels,
        const std::unordered_map<int, int>* slotOf);


    int m_CountX = 0;
    int m_CountZ = 0;
};
