#pragma once
#include "VectorMath.h"
#include "VoxelRenderer.h"
#include <vector>
#include <array>

class HeightMap;

using VoxelSourceFn = std::function<bool(int x, int y, int z)>;

// 해당 노드가 walkable인지 판별할때, 이웃간의 cache locality 향상
// 이웃을 봐도 최대 거리가 256바이트이므로 캐시라인 4번이면 처음부터 끝까지 접근 가능
class VoxelChunk
{
public:
    static constexpr int CHUNK_SIZE = 16;
    static constexpr int CHUNK_VOLUME = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE;   // = 4096

    enum class CellType : uint8_t
    {
        Empty = 0,      // 복셀이 없는 공간
        Walkable = 1,   // 이동 가능 표면
        Blocked = 2,    // 이동 불가 (내부 or 경사)
    };

    CellType Get(int lx, int ly, int lz) const { return m_Cells[LocalIndex(lx, ly, lz)]; }
    void Set(int lx, int ly, int lz, CellType type) { m_Cells[LocalIndex(lx, ly, lz)] = type; }

private:
    static int LocalIndex(int lx, int ly, int lz) { return lx + 16 * (ly + 16 * lz); }
    std::array<CellType, CHUNK_VOLUME> m_Cells;                 // 청크 하나 = 정확히 4096바이트(4KB)
};

// 복셀 메모리는 16^3 청크(VoxelChunk)로 구성 — 3D A*/FlowField 인접 접근 캐시 효율용.
// TODO : 청크 내부 인덱싱 Morton order 전환은 BFS/A* 프로파일링 후 결정
class VoxelGrid
{
public:
    // 기존
    void Initialize(int sizeX, int sizeY, int sizeZ, float cellSize);
    void Shutdown();

    using CellType = VoxelChunk::CellType;

    // TODO - 지형 동적 변경
    void SetCell(int x, int y, int z, CellType type);
    // 렌더용 인스턴스 목록 생성
    void BuildInstanceList(std::vector<VoxelRenderer::InstanceData>& outInstances) const;

    CellType      GetCell(int x, int y, int z) const;
    bool          IsWalkable(int x, int y, int z) const;
    Math::Vector3 GetWorldPos(int x, int y, int z) const;
    int   GetSizeX()    const { return m_SizeX; }
    int   GetSizeZ()    const { return m_SizeZ; }
    float GetCellSize() const { return m_CellSize; }

    //-----
    // HeightMap 기반 복셀 생성
    void BuildFromHeightMap(const HeightMap& hm);
    // 패스 1 — 높이값만 읽어서 복셀 배치
    void BuildCells(const HeightMap& hm);

    // xz당 표면 1개 가정을 없앤 build
    void BuildFromVolumeSource(const VoxelSourceFn& isSolid, int sizeX, int sizeY, int sizeZ, float cellSize);

    // 패스 2 — 배치 완료 후 표면 복셀 walkable 재판정
    void ValidateWalkable();
    // 표면 복셀인지 확인 (위쪽이 비어있는 복셀)
    bool IsSurface(int x, int y, int z) const;
    // x,z 위치의 표면 y값 반환
    int  GetSurfaceY(int x, int z) const;

    // 동굴이 있으면 2개 이상, 없으면 GetSurfaceY와 동일한 값 1개만 담긴 리스트가 됨.
    std::vector<int> GetSurfaceYList(int x, int z) const;

private:
    // 표면 복셀 하나만 저장 (위치 + 타입)
    struct VoxelCell
    {
        int      x, y, z;
        CellType type;
    };

    std::vector<VoxelCell>  m_Cells;    // 기존 구조와 병행
    //std::vector<CellType>   m_Grid;   // 3D 그리드 (A* 등 용도)
    std::vector<VoxelChunk> m_Chunks;

    // 청크 개수 멤버
    int m_ChunkCountX = 0;
    int m_ChunkCountY = 0;
    int m_ChunkCountZ = 0;

    int   m_SizeX = 0;
    int   m_SizeY = 0;
    int   m_SizeZ = 0;
    float m_CellSize = 1.0f;

    // 청크 배열 리사이즈 + 초기화 (m_SizeX/Y/Z, m_ChunkCount* 설정 후 호출)
    void AllocateChunks();

    void ToChunkCoord(int x, int y, int z, int& chunkIndex, int& lx, int& ly, int& lz) const;
};
