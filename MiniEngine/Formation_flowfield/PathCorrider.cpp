#include "PathCorridor.h"
#include "VoxelGrid.h"
#include "ChunkKey.h"
#include <algorithm>

int ComputeMarginChunks(int memberCount, int chunkSize, int bufferCells)
{
    const int MIN_MARGIN = 0;

    if (memberCount <= 1)   return MIN_MARGIN;

    float side = std::sqrt((float)memberCount);
    float radiusCells = side * 0.5f + bufferCells;

    int margin = (int)std::ceil(radiusCells / (float)chunkSize);
    return std::max(margin, MIN_MARGIN);
}

std::unordered_set<int64_t> BuildChunkMask(const std::vector<DirectX::XMINT3>& path, int marginChunks, int chunkSize)
{
    std::unordered_set<int64_t> mask;

    for (const auto& node : path)
    {
        int cx = node.x / chunkSize;
        int cz = node.z / chunkSize;

        // 수평(XZ) 방향으로만 margin을 확장. Y는 지형 표면이 몇 층 없어서
        // 확장할 필요가 크지 않고(그룹이 위아래로 퍼질 일은 없음), 옆으로만 퍼지므로
        // XZ만 넓혀서 그룹 대형이 지나갈 폭을 확보함.
        for (int dx = -marginChunks; dx <= marginChunks; dx++)
        {
            for (int dz = -marginChunks; dz <= marginChunks; dz++)
            {
                // 음수 청크 좌표는 실제로 존재하지 않는(맵 밖) 청크이므로 0으로 클램프.
                // (맵 가장자리 근처 경로일 때 margin이 맵 밖으로 나가는 걸 방지)
                int mcx = std::max(0, cx + dx);
                int mcz = std::max(0, cz + dz);

                mask.insert(MakeChunkKey(mcx, 0, mcz));
            }
        }
    }

    return mask;
}
