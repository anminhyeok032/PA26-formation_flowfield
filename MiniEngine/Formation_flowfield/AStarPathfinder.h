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
    // cameFrom을 아예 저장하지 않음. 이동 비용이 균일(1칸=1)하므로,
    // 탐색이 끝난 뒤 내 이웃 중 gScore가 정확히 나보다 1 작은 셀을 찾으면 부모
    struct PathNodeChunk
    {
        static constexpr int SIZE = 16;
        static constexpr int VOLUME = SIZE * SIZE * SIZE;

        std::array<float, VOLUME> gScore;
        std::array<bool, VOLUME> closed;

        PathNodeChunk()
        {
            gScore.fill(FLT_MAX);
            closed.fill(false);
        }
    };

    std::unordered_map<int64_t, std::unique_ptr<PathNodeChunk>> m_Chunks;


    PathNodeChunk* FindOrCreateChunk(int x, int y, int z, int& outLocalIdx);
    const PathNodeChunk* FindChunk(int x, int y, int z, int& outLocalIdx) const;

    // 탐색 종료 후, gScore만으로 목적지->시작점 경로를 거꾸로 재구성.
    // 방문한 노드 전체가 아니라 실제 경로 길이만큼만 탐색
    bool ReconstructPath(const VoxelGrid& grid, const DirectX::XMINT3& start,
        const DirectX::XMINT3& goal, std::vector<DirectX::XMINT3>& outPath) const;



    static float Heuristic(const DirectX::XMINT3& a, const DirectX::XMINT3& b)
    {
        // y는 지형에 종속된 파생값이라 수평 거리만 사용 (기존 FlowField 설계와 동일 원칙)
        return (float)(std::abs(a.x - b.x) + std::abs(a.z - b.z));
    }
};
