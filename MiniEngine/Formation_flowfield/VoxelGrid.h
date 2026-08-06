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



// 표면 캐시 전용 2d 청크 
// VoxelChunk와 동일한 CHUNK_SIZE(16)를 재사용해서, 같은 청크 좌표 체계를 공유함
// (A*가 국소적으로 이웃을 맴도는 사용 패턴과 일치시키기 위함)
// --특정 좌표를 찍어서 해당 좌표가 몇개의 y층을 가지고 있는지를 알기위한 청크 --
// -- 따라서 순회해서 청크 접근시, 모든 층에 접근하므로 순회사용 금지 --
class SurfaceChunk
{
public :
    static constexpr int CHUNK_SIZE = VoxelChunk::CHUNK_SIZE;
    static constexpr int CHUNK_AREA = CHUNK_SIZE * CHUNK_SIZE; // 256 (xz 표면만 사용함)

    struct SurfaceColumn
    {
        static constexpr int INLINE_CAPACITY = 4;
        std::array<int16_t, INLINE_CAPACITY> surfaces{};    // y가 32'767 넘으면 자료형 교체
        uint8_t count = 0;
    };

    // 해당 위치(x, z)의 surfacecolmn 반환
    SurfaceColumn& At(int lx, int lz) { return m_Columns[lx + CHUNK_SIZE * lz]; }
    const SurfaceColumn& At(int lx, int lz) const { return m_Columns[lx + CHUNK_SIZE * lz]; }

private:
    std::array<SurfaceColumn, CHUNK_AREA> m_Columns;

};





// 복셀 메모리는 16^3 청크(VoxelChunk)로 구성 — 3D A*/FlowField 인접 접근 캐시 효율용.
class VoxelGrid
{
public:
    // 기존
    void Initialize(int sizeX, int sizeY, int sizeZ, float cellSize);
    void Shutdown();

    using CellType = VoxelChunk::CellType;

    void SetCell(int x, int y, int z, CellType type);

    // 격자 좌표 하나 (우클릭 피킹 결과와 렌더 인스턴스를 매칭하기 위한 용도)
    // 좌표는 21비트 키(MakeCellKey)에 담기는 크기 (실맵 좌표 << 32767).
    struct CellCoord { int16_t x, y, z; };

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
    CellType        GetCell(const DirectX::XMINT3 c) const;
    bool            IsWalkable(int x, int y, int z) const;
    Math::Vector3   GetWorldPos(int x, int y, int z) const;
    int             GetSizeX()    const { return m_SizeX; }
    int             GetSizeZ()    const { return m_SizeZ; }
    float           GetCellSize() const { return m_CellSize; }



    // xz당 표면 1개 가정을 없앤 build
    void BuildFromVolumeSource(const VoxelSourceFn& isSolid, int sizeX, int sizeY, int sizeZ, float cellSize);


    // ---- 지형 생성 ----
    // 지형이 동굴 바닥과 같고 높이만 높힌 터널 제작 함수
    // 시작점(startX,startZ) ~ 끝점(endX,endZ) 사이에 직선 아치형 터널을 생성.
    // 지표면(ground)은 그대로 유지한 채, 그 위에 다리처럼 아치를 얹고
    // 아치 아래 빈 공간이 터널이 됨 (NPC는 아래로 통과, 아치 위로도 통과 가능).
    void BuildFromHeightMapWithTunnel(const HeightMap& ground,
        const DirectX::XMFLOAT3& start,
        const DirectX::XMFLOAT3& end,
        float radius = 3.0f,            // 터널 폭(중심선 기준 반경)
        float shellThickness = 1.5f,    // 터널 벽/천장의 두께
        bool  openAtStart = true,
        bool  openAtEnd = true);

    // 이미 만들어진 지형 위에 좌우 절벽을 추가해서 병목(허리 잘록한 협곡)을 만듦
    // start~end 축을 따라 통로 반폭이 outerHalfWidth(양끝) -> minHalfWidth(중앙)로 좁아졌다 다시 넓어짐
    // 축 구간 안에서는 통로 밖 전체가 맵 경계까지 벽이므로, 축을 가로지르려면 반드시
    // 통로를 통과해야 함(우회 불가 = 진짜 병목) 축 구간 밖은 전혀 건드리지 않아 접근/이탈은 자유
    void AddNarrowingCliffs(const DirectX::XMFLOAT3& start,
        const DirectX::XMFLOAT3& end,
        float outerHalfWidth = 12.0f,   // 양끝(넓은 구간) 통로 반폭
        float minHalfWidth = 3.0f,      // 중앙(가장 좁은 지점) 통로 반폭
        float cliffHeight = 4.0f);      // 절벽이 지면 위로 솟는 높이



    // OverwriteCells 돌려주는 렌더 목록 변경분
    struct TerrainEditDelta
    {
        struct AddedCell { int16_t x, y, z; CellType type; };
        std::vector<DirectX::XMINT3>    removed;    // 렌더 목록에서 빠진 셀
        std::vector<AddedCell>          added;      // 새로 추가된 셀
    };
    // 지정된 셀 type 덮어쓰기 / 해당 컬럼 렌더 목록 및 표면 캐시 재구축
    void OverwriteCells(const std::vector<DirectX::XMINT3>& cells, CellType type, TerrainEditDelta& outDelta);

    void ReportMemory(const char* filePath = "Report/mem_VoxelGrid.txt") const;

    // 해당 셀이 공기(Empty)에 노출됐는지: 바닥/맵 경계이거나 6방향 이웃 중 하나라도 Empty
    // BuildFromVolumeSource와 AddNarrowingCliffs가 동일 로직을 공유하기 위한 헬퍼
    bool IsCellExposed(int x, int y, int z) const;

    // 컬럼의 최하단 표면 y. 다층 컬럼(터널 등)에서도 원래 지면을 잡기 위함
    // 표면이 없으면 -1.
    int  GetGroundY(int x, int z) const;






    // 패스 2 — 배치 완료 후 표면 복셀 walkable 재판정
    void ValidateWalkable();
    // 표면 복셀인지 확인 (위쪽이 비어있는 복셀)
    bool IsSurface(int x, int y, int z) const;

    // 맵 밖인지 반환
    bool IsInBounds(int x, int y, int z) const
    {
        return x >= 0 && x < m_SizeX && y >= 0 && y < m_SizeY&& z >= 0 && z < m_SizeZ;
    }


    // Y List를 받아 순회용 구조체
    struct SurfaceSpan
    {
        // y들어있는 list
        const int16_t* data;
        int count;
        // range 기반 탐색용 - std::span 패턴
        const int16_t* begin() const { return data; }
        const int16_t* end() const { return data + count; }
    };
    SurfaceSpan GetSurfaceYList(int x, int z) const;


    // worldPos에서 가장 가까운 Walkable 셀을 찾음 (A* 시작점 스냅 등에 사용).
    // 중심에서부터 반지름을 넓혀가며 검사하므로, 맵 전체를 순회하지 않고
    // 대부분의 경우 몇 칸 안에서 빠르게 찾아짐. maxSearchRadius 안에 못 찾으면 false.
    bool FindNearestWalkable(const Math::Vector3& worldPos,
        int& outX, int& outY, int& outZ,
        int maxSearchRadius = 10) const;

private:

    // 렌더용 셀 저장소 - 전체 복셀 좌표만 보관 
    std::vector<CellCoord>  m_Cells;    

    // walkable 판별용 청크 구조
    std::vector<VoxelChunk> m_Chunks; 

    // xz좌표당 몇개의 표면(y)를 갖고있는지 아는 청크
    std::vector<SurfaceChunk> m_SurfaceChunks;
    int m_SurfaceChunkCountX = 0;
    int m_SurfaceChunkCountZ = 0;

    // 청크 개수 멤버
    int m_ChunkCountX = 0;
    int m_ChunkCountY = 0;
    int m_ChunkCountZ = 0;

    // 맵 전체 크기(cell 기준 단위)
    int   m_SizeX = 0;
    int   m_SizeY = 0;
    int   m_SizeZ = 0;
    float m_CellSize = 0.5f;

    // m_Chunks 청크 배열 리사이즈 + 초기화 (m_SizeX/Y/Z, m_ChunkCount* 설정 후 호출)
    void AllocateChunks();

    // --- SurfaceChunk 캐싱을 위한 함수 ---
    // xz좌표당 표면 cell 전용 청크 크기 할당 (AllocateChunks 시점에 같이 호출)
    void AllocateSurfaceCache();                
    // 컬럼 하나(x,z)만 다시 스캔해서 캐시 갱신.
    // 지형 확정 직후 전체 컬럼에 대해 1회씩 호출되고,
    void RefreshSurfaceColumn(int x, int z); 
    // 지형이 확정된 직후(모든 SetCell 완료 후) 호출 — 표면 캐시 할당 + 전체 컬럼 1회 스캔.
    void BuildSurfaceCache();
    // (x,y,z) 헤드룸 walkable 조건
    bool CheckWalkableCondition(int x, int y, int z) const;

    void ToChunkCoord(int x, int y, int z, int& chunkIndex, int& lx, int& ly, int& lz) const;
};
