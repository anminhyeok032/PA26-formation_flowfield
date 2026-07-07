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
    static int LocalIndex(int lx, int ly, int lz) { return lx + CHUNK_SIZE * (ly + CHUNK_SIZE * lz); }
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
 
    // 격자 좌표 하나 (우클릭 피킹 결과와 렌더 인스턴스를 매칭하기 위한 용도)
    struct CellCoord { int x, y, z; };

    // 렌더용 인스턴스 목록 생성.
    // outCoords가 주어지면 outInstances[i]에 대응하는 격자 좌표를 같은 순서로 채움
    // (기존 호출부는 nullptr 그대로 두면 기존과 동일하게 동작).
    void BuildInstanceList(std::vector<VoxelRenderer::InstanceData>& outInstances,
        std::vector<CellCoord>* outCoords = nullptr) const;

    // 레이(origin, dir)를 따라 격자를 순회하며 처음 만나는 non-Empty 셀을 찾음.
    // 우클릭 피킹(목적지 복셀 지정)에 사용. 찾으면 true, maxDistance 안에 없으면 false.
    bool RaycastVoxel(const Math::Vector3& origin, const Math::Vector3& dir,
        float maxDistance, int& outX, int& outY, int& outZ) const;

    CellType        GetCell(int x, int y, int z) const;
    bool            IsWalkable(int x, int y, int z) const;
    Math::Vector3   GetWorldPos(int x, int y, int z) const;
    int             GetSizeX()    const { return m_SizeX; }
    int             GetSizeZ()    const { return m_SizeZ; }
    float           GetCellSize() const { return m_CellSize; }

    //-----
    // HeightMap 기반 복셀 생성
    void BuildFromHeightMap(const HeightMap& hm);
    // 패스 1 — 높이값만 읽어서 복셀 배치
    void BuildCells(const HeightMap& hm);

    // xz당 표면 1개 가정을 없앤 build
    void BuildFromVolumeSource(const VoxelSourceFn& isSolid, int sizeX, int sizeY, int sizeZ, float cellSize);

    // ===== 변경 (좌표 4개 기반) =====
        // 시작점(startX,startZ) ~ 끝점(endX,endZ) 사이에 직선 아치형 터널을 생성.
        // 지표면(ground)은 그대로 유지한 채, 그 위에 다리처럼 아치를 얹고
        // 아치 아래 빈 공간이 터널이 됨 (NPC는 아래로 통과, 아치 위로도 통과 가능).
        //
        // openAtStart/openAtEnd == true인 쪽은 그 좌표에서 아치 높이가 0으로 강제되어
        // 반드시 개방된 입/출구가 생김. false면 그쪽 끝은 개방을 강제하지 않고
        // (막다른 동굴처럼) 자연 지형에 그대로 파묻힘.
    void BuildFromHeightMapWithTunnel(const HeightMap& ground,
        float startX, float startZ,
        float endX, float endZ,
        float tunnelHeightWorld = 2.0f,  // 터널 내부 통행 가능 높이(헤드룸)
        float tunnelRadiusWorld = 3.0f,  // 터널 폭(중심선 기준 반경)
        float archThicknessWorld = 1.0f,  // 아치(천장) 두께
        bool  openAtStart = true,
        bool  openAtEnd = true);


    // 패스 2 — 배치 완료 후 표면 복셀 walkable 재판정
    void ValidateWalkable();
    // 표면 복셀인지 확인 (위쪽이 비어있는 복셀)
    bool IsSurface(int x, int y, int z) const;
    // x,z 위치의 표면 y값 반환
    int  GetSurfaceY(int x, int z) const;

    // 동굴이 있으면 2개 이상, 없으면 GetSurfaceY와 동일한 값 1개만 담긴 리스트가 됨.
    std::vector<int> GetSurfaceYList(int x, int z) const;


    // worldPos에서 가장 가까운 Walkable 셀을 찾음 (A* 시작점 스냅 등에 사용).
    // 중심에서부터 반지름을 넓혀가며 검사하므로, 맵 전체를 순회하지 않고
    // 대부분의 경우 몇 칸 안에서 빠르게 찾아짐. maxSearchRadius 안에 못 찾으면 false.
    bool FindNearestWalkable(const Math::Vector3& worldPos,
        int& outX, int& outY, int& outZ,
        int maxSearchRadius = 10) const;

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
