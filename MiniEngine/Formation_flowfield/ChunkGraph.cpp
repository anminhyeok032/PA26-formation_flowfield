#include "ChunkGraph.h"
#include "GridNeighbors.h"
#include <queue>
#include <algorithm>
#include <cfloat>
#include <cmath>

namespace
{
    // GetWalkableNeighbors의 dx[8]/dz[8]과 순서까지 동일해야 한다.
    // 어긋나면 두 시스템이 서로 다른 그래프를 가정하게 된다.
    const int kDirX[8] = { 1, -1, 0,  0,  1,  1, -1, -1 };
    const int kDirZ[8] = { 0,  0, 1, -1,  1, -1,  1, -1 };

    constexpr float kDiagCost = 1.41421356f;
}

bool ChunkGraph::HasCrossing(const VoxelGrid& grid, int cx, int cz, int dir) const
{
    // 해당 방향에 대한 경계(8칸)을 기준으로 하나라도 건널 수 있는지 검사
    const int  dx = kDirX[dir];
    const int  dz = kDirZ[dir];
    const bool diag = (dir >= 4);

    // 카디널은 경계 8칸을 훑고, 대각선은 코너 셀 하나만 본다
    const int steps = diag ? 1 : CHUNK_SIZE;
    const int startX = (dx != 0) ? cx * CHUNK_SIZE + (dx > 0 ? CHUNK_SIZE - 1 : 0)
        : cx * CHUNK_SIZE;
    const int startZ = (dz != 0) ? cz * CHUNK_SIZE + (dz > 0 ? CHUNK_SIZE - 1 : 0)
        : cz * CHUNK_SIZE;
    const int stepX = (!diag && dx == 0) ? 1 : 0;
    const int stepZ = (!diag && dz == 0) ? 1 : 0;

    for (int i = 0; i < steps; ++i)
    {
        const int ax = startX + stepX * i;
        const int az = startZ + stepZ * i;

        // 컬럼의 모든 y층을 확인 - 다층 경계(다리 위/아래)에서 한쪽만 통해도 연결이다
        for (int16_t ay : grid.GetSurfaceYList(ax, az))
        {
            if (!grid.IsWalkable(ax, ay, az)) continue;

            // 대각선은 코너 파고들기 방지 게이트 통과 필요 (GetWalkableNeighbors와 동일)
            if (diag)
            {
                const int gateX = FindConnectableSurfaceY(grid, ax + dx, az, ay);
                if (gateX < 0 || !grid.IsWalkable(ax + dx, gateX, az)) continue;

                const int gateZ = FindConnectableSurfaceY(grid, ax, az + dz, ay);
                if (gateZ < 0 || !grid.IsWalkable(ax, gateZ, az + dz)) continue;
            }

            const int by = FindConnectableSurfaceY(grid, ax + dx, az + dz, ay);
            if (by >= 0 && grid.IsWalkable(ax + dx, by, az + dz))
                return true;   // 하나만 있으면 연결 확정
        }
    }
    return false;
}

uint8_t ChunkGraph::ComputeLinks(const VoxelGrid& grid, int cx, int cz) const
{
    uint8_t links = 0;
    for (int dir = 0; dir < DIR_COUNT; ++dir)
    {
        const int nx = cx + kDirX[dir];
        const int nz = cz + kDirZ[dir];
        if (nx < 0 || nx >= m_CountX || nz < 0 || nz >= m_CountZ) continue;

        if (HasCrossing(grid, cx, cz, dir)) links |= (uint8_t)(1u << dir);
    }
    return links;
}

void ChunkGraph::Build(const VoxelGrid& grid)
{
    m_CountX = (grid.GetSizeX() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    m_CountZ = (grid.GetSizeZ() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    m_Links.assign((size_t)m_CountX * m_CountZ, 0);

    for (int cz = 0; cz < m_CountZ; ++cz)
    {
        for (int cx = 0; cx < m_CountX; ++cx)
        {
            m_Links[cx + m_CountX * cz] = ComputeLinks(grid, cx, cz);
        }
    }
}

void ChunkGraph::RefreshAround(const VoxelGrid& grid,
    const std::vector<DirectX::XMINT3>& editedCells)
{
    // 같은 청크를 여러 번 재계산하지 않도록 모아둔다
    std::unordered_set<int> dirty;

    for (const auto& cell : editedCells)
    {
        const int ccx = cell.x / CHUNK_SIZE;
        const int ccz = cell.z / CHUNK_SIZE;

        for (int dz = -1; dz <= 1; ++dz)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                const int nx = ccx + dx;
                const int nz = ccz + dz;
                if (nx < 0 || nx >= m_CountX || nz < 0 || nz >= m_CountZ) continue;
                dirty.insert(nx + m_CountX * nz);
            }
        }
    }

    for (int idx : dirty)
    {
        m_Links[idx] = ComputeLinks(grid, idx % m_CountX, idx / m_CountX);
    }
}

bool ChunkGraph::FindChunkPath(int startX, int startZ, int goalX, int goalZ,
    std::vector<int64_t>& outChunkPath) const
{
    outChunkPath.clear();
    if (m_Links.empty()) return false;

    const int goalCx = goalX / CHUNK_SIZE;
    const int goalCz = goalZ / CHUNK_SIZE;
    const int startIdx = (startX / CHUNK_SIZE) + m_CountX * (startZ / CHUNK_SIZE);
    const int goalIdx = goalCx + m_CountX * goalCz;

    // 옥타일 거리 - 대각 비용(루트 2)과 맞춰야 admissible
    auto heuristic = [&](int idx)
    {
        const int dx = std::abs((idx % m_CountX) - goalCx);
        const int dz = std::abs((idx / m_CountX) - goalCz);
        return (float)(dx + dz) - (2.0f - kDiagCost) * (float)std::min(dx, dz);
    };

    std::vector<float> gScore(m_Links.size(), FLT_MAX);
    std::vector<int>   cameFrom(m_Links.size(), -1);

    struct OpenEntry
    {
        int   idx;
        float g;
        float f;
        bool operator>(const OpenEntry& o) const { return f > o.f; }
    };
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open;

    gScore[startIdx] = 0.0f;
    open.push({ startIdx, 0.0f, heuristic(startIdx) });

    while (!open.empty())
    {
        const OpenEntry curr = open.top();
        open.pop();

        // CorridorFlowField::Build과 동일한 lazy deletion
        if (curr.g > gScore[curr.idx]) continue;

        if (curr.idx == goalIdx)
        {
            for (int at = goalIdx; at != -1; at = cameFrom[at])
            {
                outChunkPath.push_back(MakeChunkKey(at % m_CountX, 0, at / m_CountX));
            }
            std::reverse(outChunkPath.begin(), outChunkPath.end());
            return true;
        }

        const int     cx = curr.idx % m_CountX;
        const int     cz = curr.idx / m_CountX;
        const uint8_t links = m_Links[curr.idx];

        for (int dir = 0; dir < DIR_COUNT; ++dir)
        {
            if ((links & (1u << dir)) == 0) continue;   // 연결 없는 방향

            const int   nIdx = (cx + kDirX[dir]) + m_CountX * (cz + kDirZ[dir]);
            const float next = curr.g + ((dir < 4) ? 1.0f : kDiagCost);

            if (next < gScore[nIdx])
            {
                gScore[nIdx] = next;
                cameFrom[nIdx] = curr.idx;
                open.push({ nIdx, next, next + heuristic(nIdx) });
            }
        }
    }
    return false;
}

std::unordered_set<int64_t> ChunkGraph::ExpandChunks(const std::vector<int64_t>& seeds, int marginChunks) const
{
    std::unordered_set<int64_t> result(seeds.begin(), seeds.end());
    if (marginChunks <= 0) return result;

    std::vector<int> depth(m_Links.size(), -1);
    std::queue<int>  pending;

    for (int64_t key : seeds)
    {
        int cx, cy, cz;
        DecodeChunkKey(key, cx, cy, cz);
        const int idx = cx + m_CountX * cz;
        if (depth[idx] < 0) { depth[idx] = 0; pending.push(idx); }
    }

    while (!pending.empty())
    {
        const int curr = pending.front();
        pending.pop();
        if (depth[curr] >= marginChunks) continue;

        const int     cx = curr % m_CountX;
        const int     cz = curr / m_CountX;
        const uint8_t links = m_Links[curr];

        for (int dir = 0; dir < DIR_COUNT; ++dir)
        {
            if ((links & (1u << dir)) == 0) continue;

            const int nx = cx + kDirX[dir];
            const int nz = cz + kDirZ[dir];
            const int nIdx = nx + m_CountX * nz;
            if (depth[nIdx] >= 0) continue;

            depth[nIdx] = depth[curr] + 1;
            result.insert(MakeChunkKey(nx, 0, nz));
            pending.push(nIdx);
        }
    }
    return result;
}

std::unordered_set<int64_t> ChunkGraph::MaskCellsFromChunks(
    const VoxelGrid& grid, const std::unordered_set<int64_t>& chunks)
{
    std::unordered_set<int64_t> mask;
    mask.reserve(chunks.size() * CHUNK_SIZE * CHUNK_SIZE);

    for (int64_t key : chunks)
    {
        int cx, cy, cz;
        DecodeChunkKey(key, cx, cy, cz);

        for (int lz = 0; lz < CHUNK_SIZE; ++lz)
        {
            for (int lx = 0; lx < CHUNK_SIZE; ++lx)
            {
                const int x = cx * CHUNK_SIZE + lx;
                const int z = cz * CHUNK_SIZE + lz;

                // 모든 y층을 넣는다 - 다층 지형에서 어느 층으로 갈지는 Dijkstra가 정한다
                for (int16_t y : grid.GetSurfaceYList(x, z))
                {
                    if (grid.IsWalkable(x, y, z)) mask.insert(MakeCellKey(x, y, z));
                }
            }
        }
    }
    return mask;
}



void ChunkGraph::ChunkPathToCells(const VoxelGrid& grid, const CorridorFlowField& field,
    const std::vector<int64_t>& chunkPath, std::vector<DirectX::XMINT3>& outCells)
{
    outCells.clear();
    outCells.reserve(chunkPath.size());

    for (int64_t key : chunkPath)
    {
        int cx, cy, cz;
        DecodeChunkKey(key, cx, cy, cz);

        // 중앙에 가장 가까운 필드가 실제로 도달한 셀을 고르기
        const int centerX = cx * CHUNK_SIZE + CHUNK_SIZE / 2;
        const int centerZ = cz * CHUNK_SIZE + CHUNK_SIZE / 2;

        DirectX::XMINT3 best{ -1, -1, -1 };
        int bestDistSq = INT_MAX;

        for (int lz = 0; lz < CHUNK_SIZE; ++lz)
        {
            for (int lx = 0; lx < CHUNK_SIZE; ++lx)
            {
                const int x = cx * CHUNK_SIZE + lx;
                const int z = cz * CHUNK_SIZE + lz;

                const int dx = x - centerX, dz = z - centerZ;
                const int distSq = dx * dx + dz * dz;
                if (distSq >= bestDistSq) continue;   // 이미 더 가까운 후보가 있음

                // 이 컬럼에서 필드가 도달한 층 중 비용이 가장 낮은 것 = 실제 이동에 쓰이는 층
                float bestCost = FLT_MAX;
                int   bestY = -1;
                for (int16_t y : grid.GetSurfaceYList(x, z))
                {
                    if (!grid.IsWalkable(x, y, z)) continue;

                    float c = 0.0f;
                    if (!field.SampleCost(grid, x, y, z, c)) continue;   // 미도달 층은 제외
                    if (c < bestCost) { bestCost = c; bestY = y; }
                }
                if (bestY < 0) continue;

                bestDistSq = distSq;
                best = { x, bestY, z };
            }
        }

        if (best.x >= 0) outCells.push_back(best);

    }
}
