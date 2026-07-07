#include "PathCorridor.h"
#include "VoxelGrid.h"
#include "ChunkKey.h"
#include <algorithm>

int ComputeMarginChunks(int memberCount, int chunkSize, int bufferCells)
{
    if (memberCount <= 0) memberCount = 1;

    // 정사각형 대형 가정: 인원수의 제곱근을 올림 -> 한 변의 길이(셀 단위)
    int formationWidthCells = (int)std::ceil(std::sqrt((float)memberCount));

    // 중심에서 대형 가장자리까지의 거리 (올림 나눗셈으로 절반)
    int halfWidthCells = (formationWidthCells + 1) / 2;

    // 여유(buffer)를 더해서, 대형이 마스크 경계에 딱 붙지 않도록 함
    int totalRadiusCells = halfWidthCells + bufferCells;

    // 그 반경이 청크 몇 개에 걸치는지 (올림 나눗셈)
    int marginChunks = (totalRadiusCells + chunkSize - 1) / chunkSize;

    return std::max(1, marginChunks); // 최소 1청크는 항상 보장
}

std::unordered_set<int64_t> BuildChunkMask(const std::vector<DirectX::XMINT3>& path, int marginChunks)
{
    std::unordered_set<int64_t> mask;

    for (const auto& node : path)
    {
        int cx = node.x / VoxelChunk::CHUNK_SIZE;
        int cy = node.y / VoxelChunk::CHUNK_SIZE;
        int cz = node.z / VoxelChunk::CHUNK_SIZE;

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

                mask.insert(MakeChunkKey(mcx, cy, mcz));
            }
        }
    }

    return mask;
}
