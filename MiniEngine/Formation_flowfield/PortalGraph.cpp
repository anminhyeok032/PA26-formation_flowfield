#include "PortalGraph.h"
#include "GridNeighbors.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <cfloat>

namespace
{
    // 경계선 위에서 "건너갈 수 있다"고 확인된 지점 하나
    struct Crossing
    {
        int     step;    // 경계선을 따라간 위치 0..CHUNK_SIZE-1
        int16_t yA, yB;  // 양쪽 표면 y
    };

    // 최대 32개짜리 전용 union-find.
    // 인접 판정이 양방향 병합(양쪽 단이 합쳐지는 경우)을 만들 수 있어
    // 단순 그리디 대신 컴포넌트로 묶는 게 확실함.
    class TinyUnionFind
    {
    public:
        void Reset(int count) { for (int i = 0; i < count; ++i) m_Parent[i] = i; }
        int  Find(int i)
        {
            while (m_Parent[i] != i) { m_Parent[i] = m_Parent[m_Parent[i]]; i = m_Parent[i]; }
            return i;
        }
        void Unite(int a, int b)
        {
            a = Find(a); b = Find(b);
            if (a != b) m_Parent[b] = a;
        }
    private:
        int m_Parent[PortalGraph::CHUNK_SIZE * PortalGraph::MAX_SLOTS];
    };
}

void PortalGraph::Build(const VoxelGrid& grid)
{
    m_Portals.clear();
    m_ChunkPortals.clear();
    m_MaxPortalsPerChunk = 0;

    const int chunkCountX = (grid.GetSizeX() + CHUNK_SIZE - 1) / CHUNK_SIZE;
    const int chunkCountZ = (grid.GetSizeZ() + CHUNK_SIZE - 1) / CHUNK_SIZE;

    // 내부 경계를 정확히 한 번씩만 훑기 위해 +X / +Z 두 방향만 검사.
    // (-X/-Z는 이웃 청크가 자기 +방향으로 이미 처리함)
    for (int cz = 0; cz < chunkCountZ; ++cz)
    {
        for (int cx = 0; cx < chunkCountX; ++cx)
        {
            if (cx + 1 < chunkCountX) ExtractOnBoundary(grid, cx, cz, Axis::X);
            if (cz + 1 < chunkCountZ) ExtractOnBoundary(grid, cx, cz, Axis::Z);

            // 코너 - edge와 별개 함수. 구간이 아니라 셀 하나뿐이라 union-find 불필요
            if (cx + 1 < chunkCountX && cz + 1 < chunkCountZ)
                ExtractCorner(grid, cx, cz, +1, +1);   // NE
            if (cx + 1 < chunkCountX && cz - 1 >= 0)
                ExtractCorner(grid, cx, cz, +1, -1);   // SE
        }
    }
}

void PortalGraph::ExtractOnBoundary(const VoxelGrid& grid, int cx, int cz, Axis axis)
{
    const int baseX = cx * CHUNK_SIZE;
    const int baseZ = cz * CHUNK_SIZE;

    // A쪽 경계선: +X면이면 x 고정하고 z를 따라가고, +Z면이면 그 반대
    const int startX = (axis == Axis::X) ? baseX + CHUNK_SIZE - 1 : baseX;
    const int startZ = (axis == Axis::X) ? baseZ : baseZ + CHUNK_SIZE - 1;
    const int stepDX = (axis == Axis::X) ? 0 : 1;
    const int stepDZ = (axis == Axis::X) ? 1 : 0;
    const int crossDX = (axis == Axis::X) ? 1 : 0;   // A -> B 로 건너가는 방향
    const int crossDZ = (axis == Axis::X) ? 0 : 1;

    Crossing crossings[CHUNK_SIZE * MAX_SLOTS];
    int      crossingCount = 0;

    for (int step = 0; step < CHUNK_SIZE; ++step)
    {
        const int xA = startX + stepDX * step;
        const int zA = startZ + stepDZ * step;
        const int xB = xA + crossDX;
        const int zB = zA + crossDZ;

        if (!grid.IsInBounds(xA, 0, zA) || !grid.IsInBounds(xB, 0, zB)) continue;

        // 컬럼의 표면 층을 전부 훑음 - 같은 xz라도 층이 다르면 별개 통로
        VoxelGrid::SurfaceSpan surfaces = grid.GetSurfaceYList(xA, zA);
        for (int16_t yA : surfaces)
        {
            if (!grid.IsWalkable(xA, yA, zA)) continue;

            const int yB = FindConnectableSurfaceY(grid, xB, zB, yA);
            if (yB < 0 || !grid.IsWalkable(xB, yB, zB)) continue;

            crossings[crossingCount++] = { step, yA, (int16_t)yB };
        }
    }

    if (crossingCount == 0) return;

    // 경계선을 따라 인접(step 차 1)하면서 높이차 1 이내면 같은 포탈.
    // 같은 step의 두 표면은 y가 반드시 2 이상 차이나므로(표면 위는 Empty)
    // 같은 컬럼끼리 합쳐지는 경우는 없음.
    TinyUnionFind unionFind;
    unionFind.Reset(crossingCount);
    for (int i = 0; i < crossingCount; ++i)
    {
        for (int j = i + 1; j < crossingCount; ++j)
        {
            if (crossings[j].step - crossings[i].step != 1) continue;
            if (std::abs(crossings[i].yA - crossings[j].yA) > 1) continue;
            unionFind.Unite(i, j);
        }
    }

    // 컴포넌트 하나 = 포탈 하나
    for (int root = 0; root < crossingCount; ++root)
    {
        if (unionFind.Find(root) != root) continue;

        int members[CHUNK_SIZE * MAX_SLOTS];
        int memberCount = 0;
        for (int i = 0; i < crossingCount; ++i)
        {
            if (unionFind.Find(i) == root) members[memberCount++] = i;
        }

        // 대표 셀은 구간 중앙 - 끝단을 쓰면 상위 경로 길이가 실제보다 왜곡됨
        const Crossing& mid = crossings[members[memberCount / 2]];
        const int xA = startX + stepDX * mid.step;
        const int zA = startZ + stepDZ * mid.step;

        Portal portal;
        portal.chunkA = ChunkKeyOf(xA, zA);
        portal.chunkB = ChunkKeyOf(xA + crossDX, zA + crossDZ);
        portal.cellA = { xA, mid.yA, zA };
        portal.cellB = { xA + crossDX, mid.yB, zA + crossDZ };
        portal.width = (uint8_t)memberCount;
        RegisterPortal(portal);
    }
}

void PortalGraph::RegisterPortal(const Portal& portal)
{
    const uint32_t index = (uint32_t)m_Portals.size();
    m_Portals.push_back(portal);

    // 포탈은 두 청크 모두에 등록 - 상위 A*가 어느 쪽에서 들어와도 찾을 수 있어야 함
    for (int64_t key : { portal.chunkA, portal.chunkB })
    {
        std::vector<uint32_t>& list = m_ChunkPortals[key];
        list.push_back(index);
        m_MaxPortalsPerChunk = std::max(m_MaxPortalsPerChunk, (uint32_t)list.size());
    }
}

const std::vector<uint32_t>* PortalGraph::FindChunkPortals(int64_t chunkKey) const
{
    auto it = m_ChunkPortals.find(chunkKey);
    return (it == m_ChunkPortals.end()) ? nullptr : &it->second;
}


// GetWalkableNeighbors의 대각선 게이트 조건을 그대로 복제
void PortalGraph::ExtractCorner(const VoxelGrid& grid, int cx, int cz, int dirX, int dirZ)
{
    const int ax = cx * CHUNK_SIZE + (dirX > 0 ? CHUNK_SIZE - 1 : 0);
    const int az = cz * CHUNK_SIZE + (dirZ > 0 ? CHUNK_SIZE - 1 : 0);
    const int dx_ = ax + dirX;
    const int dz_ = az + dirZ;
    const int bx = ax + dirX, bz = az;   // 카디널 게이트 B (X방향)
    const int cx_ = ax, cz_ = az + dirZ;   // 카디널 게이트 C (Z방향)

    if (!grid.IsInBounds(dx_, 0, dz_)) return;

    VoxelGrid::SurfaceSpan surfaces = grid.GetSurfaceYList(ax, az);
    for (int16_t ay : surfaces)
    {
        if (!grid.IsWalkable(ax, ay, az)) continue;

        // GetWalkableNeighbors와 동일한 코너컷팅 게이트: B, C가 A.y 기준 연결+walkable이어야 함
        int by = FindConnectableSurfaceY(grid, bx, bz, ay);
        if (by < 0 || !grid.IsWalkable(bx, by, bz)) continue;
        int cy = FindConnectableSurfaceY(grid, cx_, cz_, ay);
        if (cy < 0 || !grid.IsWalkable(cx_, cy, cz_)) continue;

        // 대각선 목적지 D 자체의 연결 - A.y 기준 (B, C와 무관한 독자 판정)
        int dy = FindConnectableSurfaceY(grid, dx_, dz_, ay);
        if (dy < 0 || !grid.IsWalkable(dx_, dy, dz_)) continue;

        Portal portal;
        portal.chunkA = ChunkKeyOf(ax, az);
        portal.chunkB = ChunkKeyOf(dx_, dz_);
        portal.cellA = { ax, ay, az };
        portal.cellB = { dx_, dy, dz_ };
        portal.width = 1;   // 코너는 항상 폭 1
        RegisterPortal(portal);
    }
}

void PortalGraph::ConnectIntra(const VoxelGrid& grid)
{
    m_Adjacency.assign(m_Portals.size(), {});

    for (const auto& entry : m_ChunkPortals)
    {
        const std::vector<uint32_t>& portals = entry.second;
        if (portals.size() < 2) continue;   // 이을 상대가 없음

        for (uint32_t from : portals)
            ConnectFromPortal(grid, entry.first, from, portals);
    }
}

void PortalGraph::ConnectFromPortal(const VoxelGrid& grid, int64_t chunkKey,
    uint32_t fromPortal,
    const std::vector<uint32_t>& chunkPortals)
{
    // 청크 하나만 담는 고정 배열(8*8*4 = 1KB). 해시맵도 힙 할당도 없음.
    float cost[CHUNK_SIZE * CHUNK_SIZE][MAX_SLOTS];
    for (auto& column : cost)
        for (float& c : column) c = FLT_MAX;

    auto localIndex = [](int x, int z)
        { return (x % CHUNK_SIZE) + CHUNK_SIZE * (z % CHUNK_SIZE); };

    struct OpenEntry
    {
        DirectX::XMINT3 pos;
        float cost;
        bool operator>(const OpenEntry& o) const { return cost > o.cost; }
    };
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open;

    const DirectX::XMINT3 start = CellInChunk(m_Portals[fromPortal], chunkKey);
    const int startSlot = FindSurfaceSlot(grid, start.x, start.y, start.z);
    if (startSlot < 0) return;

    cost[localIndex(start.x, start.z)][startSlot] = 0.0f;
    open.push({ start, 0.0f });

    std::vector<NeighborInfo> neighbors;
    while (!open.empty())
    {
        const OpenEntry curr = open.top();
        open.pop();

        const int currSlot = FindSurfaceSlot(grid, curr.pos.x, curr.pos.y, curr.pos.z);
        if (currSlot < 0) continue;
        // CorridorFlowField::Build과 동일한 lazy deletion
        if (curr.cost > cost[localIndex(curr.pos.x, curr.pos.z)][currSlot]) continue;

        // 이웃 판정은 반드시 이 함수 하나만 사용 - 이동 그래프와 어긋나면 안 됨
        GetWalkableNeighbors(grid, curr.pos, neighbors);

        for (const auto& n : neighbors)
        {
            // 스코프 제한: 청크 밖으로 안 나감. 마스크 해시셋 대신 정수 비교 하나
            if (ChunkKeyOf(n.pos.x, n.pos.z) != chunkKey) continue;

            const int nSlot = FindSurfaceSlot(grid, n.pos.x, n.pos.y, n.pos.z);
            if (nSlot < 0) continue;

            const int   nIdx = localIndex(n.pos.x, n.pos.z);
            const float next = curr.cost + n.cost;
            if (next < cost[nIdx][nSlot])
            {
                cost[nIdx][nSlot] = next;
                open.push({ n.pos, next });
            }
        }
    }

    // 도달한 포탈만 간선으로. 도달 못 했으면 청크 내부가 벽으로 갈라진 것 -
    // 직선거리 근사가 못 잡아내던 바로 그 경우.
    for (uint32_t to : chunkPortals)
    {
        if (to == fromPortal) continue;

        const DirectX::XMINT3 cell = CellInChunk(m_Portals[to], chunkKey);
        const int slot = FindSurfaceSlot(grid, cell.x, cell.y, cell.z);
        if (slot < 0) continue;

        const float distance = cost[localIndex(cell.x, cell.z)][slot];
        if (distance == FLT_MAX) continue;

        m_Adjacency[fromPortal].push_back({ to, distance });
    }
}

std::unordered_set<int64_t> PortalGraph::ExpandChunks(
    const std::vector<int64_t>& seedChunks, int marginChunks) const
{
    std::unordered_set<int64_t>      result;
    std::unordered_map<int64_t, int> dist;
    std::queue<int64_t>              pending;

    for (int64_t key : seedChunks)
    {
        if (dist.emplace(key, 0).second) { result.insert(key); pending.push(key); }
    }

    while (!pending.empty())
    {
        const int64_t curr = pending.front();
        pending.pop();

        const int depth = dist[curr];
        if (depth >= marginChunks) continue;

        const std::vector<uint32_t>* portals = FindChunkPortals(curr);
        if (!portals) continue;

        for (uint32_t portalIndex : *portals)
        {
            const Portal& portal = m_Portals[portalIndex];
            // 포탈 반대편이 곧 이웃 청크. 포탈이 없는 방향(벽)은 자동 제외됨.
            const int64_t other = (portal.chunkA == curr) ? portal.chunkB : portal.chunkA;

            if (dist.emplace(other, depth + 1).second)
            {
                result.insert(other);
                pending.push(other);
            }
        }
    }
    return result;
}
