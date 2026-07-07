#pragma once
#include "VoxelGrid.h"
#include "ChunkKey.h"
#include <vector>
#include <array>
#include <unordered_map>
#include <memory>

// DirectX::XMINT3는 POD라 operator==/해시가 없으므로 자유 함수/특수화로 보강.
inline bool operator==(const DirectX::XMINT3& a, const DirectX::XMINT3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}
inline bool operator!=(const DirectX::XMINT3& a, const DirectX::XMINT3& b) { return !(a == b); }

namespace std
{
    template<> struct hash<DirectX::XMINT3>
    {
        size_t operator()(const DirectX::XMINT3& v) const noexcept
        {
            // 좌표 세 개를 간단히 섞어서 해시값 생성 
            // (경로/방문 기록에서 직접 쓰진 않지만 향후 unordered_set/map에 좌표를 키로 쓸 일이 생기면 바로 재사용 가능하도록 미리 준비)
            size_t h = std::hash<int>()(v.x);
            h ^= std::hash<int>()(v.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= std::hash<int>()(v.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };
}

class AStarPathfinder
{
public:
    // start -> goal 경로를 탐색. 성공 시 outPath에 start부터 goal까지 순서대로 채움.
    // maxIterations는 탐색이 비정상적으로 오래 걸리는 경우(예: 도달 불가) 안전하게 끊기 위한 상한.
    bool FindPath(const VoxelGrid& grid, 
                    const DirectX::XMINT3& start, 
                    const DirectX::XMINT3& goal,
                    std::vector<DirectX::XMINT3>& outPath, 
                    int maxIterations = 100000);

private:
    // ---- A* 방문 기록 저장용 청크 (VoxelChunk와 같은 크기 철학, 필요한 만큼만 지연 할당) ----
    struct PathNodeRecord
    {
        float           gScore = FLT_MAX;
        DirectX::XMINT3 cameFrom{ -1, -1, -1 };
        bool            closed = false;
    };

    struct PathNodeChunk
    {
        static constexpr int SIZE = VoxelChunk::CHUNK_SIZE;
        std::array<PathNodeRecord, VoxelChunk::CHUNK_VOLUME> records;
    };


    std::unordered_map<int64_t, std::unique_ptr<PathNodeChunk>> m_Chunks;


    PathNodeRecord& GetRecord(int x, int y, int z);


    static float Heuristic(const DirectX::XMINT3& a, const DirectX::XMINT3& b)
    {
        // y는 지형에 종속된 파생값이라 수평 거리만 사용 (기존 FlowField 설계와 동일 원칙)
        return (float)(std::abs(a.x - b.x) + std::abs(a.z - b.z));
    }
};
