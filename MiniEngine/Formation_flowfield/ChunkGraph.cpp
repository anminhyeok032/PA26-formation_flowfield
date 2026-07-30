#include "ChunkGraph.h"
#include "GridNeighbors.h"
#include <queue>
#include <algorithm>
#include <cfloat>
#include <cmath>

namespace
{
    // GetWalkableNeighbors의 dx[8]/dz[8]과 순서까지 동일해야 한다.
    const int kDirX[8] = { 1, -1, 0,  0,  1,  1, -1, -1 };
    const int kDirZ[8] = { 0,  0, 1, -1,  1, -1,  1, -1 };

    constexpr float kDiagCost = 1.41421356f;
}

// ---------------------------------------------------------------- 
// 
//  라벨링
//
// ---------------------------------------------------------------- 
int ChunkGraph::LabelChunk(const VoxelGrid& grid, int cx, int cz, int16_t* outLabels) const
{
    // 라벨 없음으로 초기화
    std::fill(outLabels, outLabels + LABELS_PER_CHUNK, (int16_t)-1);

    int compCount = 0;
    std::vector<DirectX::XMINT3> stack;
    std::vector<NeighborInfo>    neighbors;

    // 스캔 순서가 결정론적이어야 질의 때 재라벨링해도 같은 번호가 나온다
    for (int lz = 0; lz < CHUNK_SIZE; ++lz)
    {
        for (int lx = 0; lx < CHUNK_SIZE; ++lx)
        {
            const int x = cx * CHUNK_SIZE + lx;
            const int z = cz * CHUNK_SIZE + lz;

            VoxelGrid::SurfaceSpan surfaces = grid.GetSurfaceYList(x, z);
            // 모든 y 슬롯까지 검사해서 해당 청크에서 갈수 있는지 검사
            for (int slot = 0; slot < surfaces.count; ++slot)
            {
                const int y = surfaces.data[slot];
                if (!grid.IsWalkable(x, y, z))              continue;
                if (outLabels[LocalIndex(lx, lz, slot)] >= 0) continue;   // 이미 다른 성분 기록되있음

                // 새성분 dfs
                outLabels[LocalIndex(lx, lz, slot)] = (int16_t)compCount;
                stack.clear();
                stack.push_back({ x, y, z });

                // dfs로 각 셀마다 성분 기록
                while (!stack.empty())
                {
                    const DirectX::XMINT3 curr = stack.back();
                    stack.pop_back();

                    // 이웃 판정은 반드시 이 함수만 사용 - 이동 그래프와 어긋나면 안 된다
                    GetWalkableNeighbors(grid, curr, neighbors);

                    for (const auto& n : neighbors)
                    {
                        // 청크 밖은 성분에 넣지 않기
                        if (n.pos.x / CHUNK_SIZE != cx) continue;
                        if (n.pos.z / CHUNK_SIZE != cz) continue;

                        const int nslot = FindSurfaceSlot(grid, n.pos.x, n.pos.y, n.pos.z);
                        if (nslot < 0) continue;

                        const int idx = LocalIndex(n.pos.x % CHUNK_SIZE,
                            n.pos.z % CHUNK_SIZE, nslot);
                        if (outLabels[idx] >= 0) continue;

                        outLabels[idx] = (int16_t)compCount;
                        stack.push_back(n.pos);
                    }
                }
                ++compCount;
            }
        }
    }
    return compCount;
}

// ---------------------------------------------------------------- 간선

void ChunkGraph::BuildEdgesForChunk(const VoxelGrid& grid, int cx, int cz,
    const std::vector<int16_t>& allLabels)
{
    const int chunkIdx = cx + m_CountX * cz;

    for (uint16_t c = 0; c < m_CompCount[chunkIdx]; ++c)
        m_Adjacency[m_NodeOffset[chunkIdx] + c].clear();

    for (int dir = 0; dir < DIR_COUNT; ++dir)
    {
        const int ncx = cx + kDirX[dir];
        const int ncz = cz + kDirZ[dir];
        if (ncx < 0 || ncx >= m_CountX || ncz < 0 || ncz >= m_CountZ) continue;

        const int nChunkIdx = ncx + m_CountX * ncz;

        const int  dx = kDirX[dir];
        const int  dz = kDirZ[dir];
        const bool diag = (dir >= 4);

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

            VoxelGrid::SurfaceSpan surfaces = grid.GetSurfaceYList(ax, az);

            // 조기 탈출하지 않는다 - 같은 경계선의 다른 성분 간선을 잃기 때문
            for (int slot = 0; slot < surfaces.count; ++slot)
            {
                const int ay = surfaces.data[slot];
                if (!grid.IsWalkable(ax, ay, az)) continue;

                // 대각선은 코너 파고들기 방지 게이트 통과 필요 (GetWalkableNeighbors와 동일)
                if (diag)
                {
                    const int gateX = FindConnectableSurfaceY(grid, ax + dx, az, ay);
                    if (gateX < 0 || !grid.IsWalkable(ax + dx, gateX, az)) continue;

                    const int gateZ = FindConnectableSurfaceY(grid, ax, az + dz, ay);
                    if (gateZ < 0 || !grid.IsWalkable(ax, gateZ, az + dz)) continue;
                }

                const int bx = ax + dx, bz = az + dz;
                const int by = FindConnectableSurfaceY(grid, bx, bz, ay);
                if (by < 0 || !grid.IsWalkable(bx, by, bz)) continue;

                const int bslot = FindSurfaceSlot(grid, bx, by, bz);
                if (bslot < 0) continue;

                const int16_t srcComp = allLabels[(size_t)chunkIdx * LABELS_PER_CHUNK
                    + LocalIndex(ax % CHUNK_SIZE, az % CHUNK_SIZE, slot)];
                const int16_t dstComp = allLabels[(size_t)nChunkIdx * LABELS_PER_CHUNK
                    + LocalIndex(bx % CHUNK_SIZE, bz % CHUNK_SIZE, bslot)];
                if (srcComp < 0 || dstComp < 0) continue;

                m_Adjacency[m_NodeOffset[chunkIdx] + srcComp]
                    .push_back(m_NodeOffset[nChunkIdx] + dstComp);
            }
        }
    }

    // 같은 성분쌍이 여러 크로싱에서 나오므로 중복 제거
    for (uint16_t c = 0; c < m_CompCount[chunkIdx]; ++c)
    {
        auto& list = m_Adjacency[m_NodeOffset[chunkIdx] + c];
        std::sort(list.begin(), list.end());
        list.erase(std::unique(list.begin(), list.end()), list.end());
    }
}

// ---------------------------------------------------------------- 빌드

void ChunkGraph::Build(const VoxelGrid& grid)
{
    m_CountX = (grid.GetSizeX() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    m_CountZ = (grid.GetSizeZ() + CHUNK_SIZE - 1) / CHUNK_SIZE;

    const int chunkCount = m_CountX * m_CountZ;
    m_CompCount.assign(chunkCount, 0);
    m_NodeOffset.assign(chunkCount, 0);

    // 라벨은 간선을 만드는 동안만 필요하므로 지역 버퍼에 두고 함수 끝에서 버린다
    std::vector<int16_t> labels((size_t)chunkCount * LABELS_PER_CHUNK, -1);

    for (int cz = 0; cz < m_CountZ; ++cz)
    {
        for (int cx = 0; cx < m_CountX; ++cx)
        {
            const int idx = cx + m_CountX * cz;
            m_CompCount[idx] = (uint16_t)LabelChunk(grid, cx, cz,
                &labels[(size_t)idx * LABELS_PER_CHUNK]);
        }
    }

    // 성분 개수 누적합으로 노드 id 배정 - 고정 상한이 없어 초과로 깨질 여지가 없다
    uint32_t total = 0;
    for (int i = 0; i < chunkCount; ++i)
    {
        m_NodeOffset[i] = total;
        total += m_CompCount[i];
    }

    m_NodeChunk.assign(total, 0);
    for (int i = 0; i < chunkCount; ++i)
    {
        for (uint16_t c = 0; c < m_CompCount[i]; ++c)
        {
            m_NodeChunk[m_NodeOffset[i] + c] = (uint32_t)i;
        }
    }

    m_Adjacency.assign(total, {});

    for (int cz = 0; cz < m_CountZ; ++cz)
    {
        for (int cx = 0; cx < m_CountX; ++cx)
        {
            BuildEdgesForChunk(grid, cx, cz, labels);
        }
    }
}

void ChunkGraph::RefreshAround(const VoxelGrid& grid,
    const std::vector<DirectX::XMINT3>& editedCells)
{
    if (m_CompCount.empty()) { Build(grid); return; }

    // 라벨이 바뀌는 건 편집 셀이 든 청크뿐이다(라벨링은 청크 내부만 보므로).
    // 크로싱 판정은 이웃 셀을 보므로 간선은 그 8이웃까지 다시 만들어야 한다.
    std::unordered_set<int> edited;
    for (const auto& cell : editedCells)
    {
        const int cx = cell.x / CHUNK_SIZE, cz = cell.z / CHUNK_SIZE;
        if (cx < 0 || cx >= m_CountX || cz < 0 || cz >= m_CountZ) continue;
        edited.insert(cx + m_CountX * cz);
    }
    if (edited.empty()) return;

    // 성분 개수가 바뀌면 이후 모든 노드 id가 밀리므로 전체 재빌드
    std::vector<int16_t> probe(LABELS_PER_CHUNK);
    for (int idx : edited)
    {
        const int n = LabelChunk(grid, idx % m_CountX, idx / m_CountX, probe.data());
        if ((uint16_t)n != m_CompCount[idx]) { Build(grid); return; }
    }

    // 간선을 다시 만들 청크 = 편집 청크 + 8이웃
    std::unordered_set<int> dirtyEdges;
    for (int idx : edited)
    {
        const int cx = idx % m_CountX, cz = idx / m_CountX;
        for (int dz = -1; dz <= 1; ++dz)
            for (int dx = -1; dx <= 1; ++dx)
            {
                const int nx = cx + dx, nz = cz + dz;
                if (nx < 0 || nx >= m_CountX || nz < 0 || nz >= m_CountZ) continue;
                dirtyEdges.insert(nx + m_CountX * nz);
            }
    }

    // 그 간선들을 만들려면 대상 청크와 그 이웃의 라벨이 필요 -> 한 겹 더 넓게 라벨링
    std::vector<int16_t> labels((size_t)m_CountX * m_CountZ * LABELS_PER_CHUNK, -1);
    std::unordered_set<int> labeled;
    for (int idx : dirtyEdges)
    {
        const int cx = idx % m_CountX, cz = idx / m_CountX;
        for (int dz = -1; dz <= 1; ++dz)
            for (int dx = -1; dx <= 1; ++dx)
            {
                const int nx = cx + dx, nz = cz + dz;
                if (nx < 0 || nx >= m_CountX || nz < 0 || nz >= m_CountZ) continue;
                const int ni = nx + m_CountX * nz;
                if (!labeled.insert(ni).second) continue;
                LabelChunk(grid, nx, nz, &labels[(size_t)ni * LABELS_PER_CHUNK]);
            }
    }

    for (int idx : dirtyEdges)
        BuildEdgesForChunk(grid, idx % m_CountX, idx / m_CountX, labels);
}

// ---------------------------------------------------------------- 질의

int ChunkGraph::NodeIdOf(const VoxelGrid& grid, const DirectX::XMINT3& cell) const
{
    const int cx = cell.x / CHUNK_SIZE, cz = cell.z / CHUNK_SIZE;
    if (cx < 0 || cx >= m_CountX || cz < 0 || cz >= m_CountZ) return -1;

    const int slot = FindSurfaceSlot(grid, cell.x, cell.y, cell.z);
    if (slot < 0) return -1;

    // 라벨을 저장해두지 않는다 - 지형 편집 후 stale될 위험을 없애기 위해.
    // 청크 하나(<=256 셀슬롯)만 다시 라벨링하므로 비용은 무시 가능하다.
    int16_t labels[LABELS_PER_CHUNK];
    LabelChunk(grid, cx, cz, labels);

    const int16_t comp = labels[LocalIndex(cell.x % CHUNK_SIZE, cell.z % CHUNK_SIZE, slot)];
    if (comp < 0 || (uint16_t)comp >= m_CompCount[cx + m_CountX * cz]) return -1;

    return (int)m_NodeOffset[cx + m_CountX * cz] + comp;
}

bool ChunkGraph::FindChunkPath(const VoxelGrid& grid,
    const DirectX::XMINT3& start, const DirectX::XMINT3& goal,
    std::vector<int64_t>& outChunkPath) const
{
    outChunkPath.clear();
    if (m_Adjacency.empty()) return false;

    const int startNode = NodeIdOf(grid, start);
    const int goalNode = NodeIdOf(grid, goal);
    if (startNode < 0 || goalNode < 0) return false;

    const int goalChunk = ChunkOfNode((uint32_t)goalNode);
    const int goalCx = goalChunk % m_CountX, goalCz = goalChunk / m_CountX;

    // 옥타일 거리 - 간선 하나가 청크 한 칸 이동이므로 admissible
    auto heuristic = [&](uint32_t node)
    {
        const int c = ChunkOfNode(node);
        const int dx = std::abs((c % m_CountX) - goalCx);
        const int dz = std::abs((c / m_CountX) - goalCz);
        return (float)(dx + dz) - (2.0f - kDiagCost) * (float)std::min(dx, dz);
    };

    std::vector<float>    gScore(m_Adjacency.size(), FLT_MAX);
    std::vector<uint32_t> cameFrom(m_Adjacency.size(), UINT32_MAX);

    struct OpenEntry
    {
        uint32_t node;
        float    g;
        float    f;
        bool operator>(const OpenEntry& o) const { return f > o.f; }
    };
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open;

    gScore[startNode] = 0.0f;
    open.push({ (uint32_t)startNode, 0.0f, heuristic((uint32_t)startNode) });

    while (!open.empty())
    {
        const OpenEntry curr = open.top();
        open.pop();
        if (curr.g > gScore[curr.node]) continue;   // lazy deletion

        if (curr.node == (uint32_t)goalNode)
        {
            for (uint32_t at = (uint32_t)goalNode; at != UINT32_MAX; at = cameFrom[at])
            {
                const int c = ChunkOfNode(at);
                outChunkPath.push_back(MakeChunkKey(c % m_CountX, 0, c / m_CountX));
            }
            std::reverse(outChunkPath.begin(), outChunkPath.end());
            return true;
        }

        const int currChunk = ChunkOfNode(curr.node);
        const int ccx = currChunk % m_CountX, ccz = currChunk / m_CountX;

        for (uint32_t next : m_Adjacency[curr.node])
        {
            // 비용은 저장하지 않고 청크 좌표차로 유도 - 대각이면 sqrt2
            const int nc = ChunkOfNode(next);
            const bool isDiag = (nc % m_CountX != ccx) && (nc / m_CountX != ccz);
            const float cost = curr.g + (isDiag ? kDiagCost : 1.0f);

            if (cost < gScore[next])
            {
                gScore[next] = cost;
                cameFrom[next] = curr.node;
                open.push({ next, cost, cost + heuristic(next) });
            }
        }
    }
    return false;
}

std::unordered_set<int64_t> ChunkGraph::ExpandChunks(const std::vector<int64_t>& seeds,
    int marginChunks) const
{
    std::unordered_set<int64_t> result(seeds.begin(), seeds.end());
    if (marginChunks <= 0) return result;

    // 성분 노드를 따라 퍼진다 - 갈라진 청크의 반대쪽으로는 새지 않는다
    std::vector<int> depth(m_Adjacency.size(), -1);
    std::queue<uint32_t> pending;

    for (int64_t key : seeds)
    {
        int cx, cy, cz;
        DecodeChunkKey(key, cx, cy, cz);
        if (cx < 0 || cx >= m_CountX || cz < 0 || cz >= m_CountZ) continue;

        const int idx = cx + m_CountX * cz;
        for (uint16_t c = 0; c < m_CompCount[idx]; ++c)
        {
            const uint32_t node = m_NodeOffset[idx] + c;
            if (depth[node] < 0) { depth[node] = 0; pending.push(node); }
        }
    }

    while (!pending.empty())
    {
        const uint32_t curr = pending.front();
        pending.pop();
        if (depth[curr] >= marginChunks) continue;

        for (uint32_t next : m_Adjacency[curr])
        {
            if (depth[next] >= 0) continue;
            depth[next] = depth[curr] + 1;

            const int c = ChunkOfNode(next);
            result.insert(MakeChunkKey(c % m_CountX, 0, c / m_CountX));
            pending.push(next);
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

                // 모든 y층을 넣는다 - 어느 층으로 갈지는 Dijkstra가 정한다
                VoxelGrid::SurfaceSpan surfaces = grid.GetSurfaceYList(x, z);
                for (int slot = 0; slot < surfaces.count; ++slot)
                {
                    const int y = surfaces.data[slot];
                    if (grid.IsWalkable(x, y, z)) mask.insert(MakeCellKey(x, y, z));
                }
            }
        }
    }
    return mask;
}

int ChunkGraph::GetSplitChunkCount() const
{
    int n = 0;
    for (uint16_t c : m_CompCount) if (c > 1) ++n;
    return n;
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
