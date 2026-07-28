#include "VoxelGrid.h"
#include "HeightMap.h"
#include "ChunkKey.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>

void VoxelGrid::Initialize(int sizeX, int sizeY, int sizeZ, float cellSize)
{
    m_SizeX = sizeX;
    m_SizeY = sizeY;
    m_SizeZ = sizeZ;
    m_CellSize = cellSize;

    AllocateChunks();  // 청크 배열 할당 (전부 Empty 상태)

    // 전체를 Walkable로 채움
    for (int z = 0; z < m_SizeZ; z++)
    {
        for (int y = 0; y < m_SizeY; y++)
        {
            for (int x = 0; x < m_SizeX; x++)
            {
                SetCell(x, y, z, CellType::Walkable);
            }
        }
    }

    m_Cells.clear();

}

void VoxelGrid::Shutdown()
{
    m_Cells.clear();
    m_Chunks.clear();
    m_ChunkCountX = m_ChunkCountY = m_ChunkCountZ = 0;  // 청크 카운트도 리셋
}


void VoxelGrid::BuildFromVolumeSource(const VoxelSourceFn& isSolid, int sizeX, int sizeY, int sizeZ, float cellSize)
{
    // 동굴/다층 지형 등 임의의 3D 소스를 위한 일반화된 빌드 경로.
    // BuildCells(HeightMap)와 달리 "표면은 컬럼당 하나"라는 가정이 없음
    // isSolid(x,y,z)가 y축으로 여러 번 열리고 닫혀도(동굴) 그대로 반영됨.

    m_Cells.clear();
    m_SizeX = sizeX;
    m_SizeY = sizeY;
    m_SizeZ = sizeZ;
    m_CellSize = cellSize;

    AllocateChunks();

    // 1단계: 소스 콜백으로 3D 그리드 채우기 (청크만 채움, m_Cells는 아직 안 넣음)
    for (int z = 0; z < m_SizeZ; z++)
    {
        for (int y = 0; y < m_SizeY; y++)
        {
            for (int x = 0; x < m_SizeX; x++)
            {
                SetCell(x, y, z, isSolid(x, y, z) ? CellType::Blocked : CellType::Empty);
            }
        }
    }

    // 2단계: 렌더링할 복셀만 m_Cells에 추가
    //        조건: 표면(IsSurface) OR 바닥(y=0) OR 6면 중 어느 한 면이라도 이웃이 Empty(노출)
    //        기존 BuildCells는 "컬럼의 최상단 surfY"만 표면으로 취급했지만,
    //        여기서는 셀 단위 IsSurface + 6방향 이웃 노출 검사로 일반화함
    //        (동굴 천장/바닥처럼 컬럼 중간에 있는 표면도 놓치지 않기 위함)
    const int dx[] = { 1, -1, 0, 0, 0, 0 };
    const int dy[] = { 0, 0, 1, -1, 0, 0 };
    const int dz[] = { 0, 0, 0, 0, 1, -1 };

    for (int z = 0; z < m_SizeZ; z++)
    {
        for (int y = 0; y < m_SizeY; y++)
        {
            for (int x = 0; x < m_SizeX; x++)
            {
                if (GetCell(x, y, z) == CellType::Empty) continue; // 빈 공간은 렌더 대상 아님
                if (!IsCellExposed(x, y, z)) continue;             // 완전히 파묻힌 내부 복셀은 렌더 스킵
                //bool isBottom = (y == 0);
                //bool isWallX = (x == 0 || x == m_SizeX - 1);
                //bool isWallZ = (z == 0 || z == m_SizeZ - 1);

                //// 6방향 중 하나라도 Empty(또는 맵 밖)면 공기에 노출된 면이 있는 것
                //bool isExposed = isBottom || isWallX || isWallZ;
                //if (!isExposed)
                //{
                //    for (int d = 0; d < 6; d++)
                //    {
                //        int nx = x + dx[d];
                //        int ny = y + dy[d];
                //        int nz = z + dz[d];
                //        if (nx < 0 || nx >= m_SizeX || ny < 0 || ny >= m_SizeY || nz < 0 || nz >= m_SizeZ)
                //            continue;
                //        if (GetCell(nx, ny, nz) == CellType::Empty)
                //        {
                //            isExposed = true;
                //            break;
                //        }
                //    }
                //}

                //if (!isExposed) continue; // 완전히 파묻힌 내부 복셀은 렌더 스킵

                m_Cells.push_back({ (int16_t)x, (int16_t)y, (int16_t)z });
            }
        }
    }
}

void VoxelGrid::BuildFromHeightMapWithTunnel(const HeightMap& ground,
    const DirectX::XMFLOAT3& start,
    const DirectX::XMFLOAT3& end,
    float radius,               // 터널 폭(중심선 기준 반경)
    float shellThickness,       // 터널 벽/천장의 두께
    bool  openAtStart,
    bool  openAtEnd)
{
    float cellSize = ground.GetVoxelSize();
    float outerRadius = (radius + shellThickness);

    float axisY = ground.SampleHeight(start.x, start.z) - radius/3.0f;

    DirectX::XMFLOAT3 startPoint = { start.x, axisY, start.z };
    DirectX::XMFLOAT3 endPoint = { end.x, axisY, end.z };

    XMVECTOR S = XMLoadFloat3(&startPoint);
    XMVECTOR E = XMLoadFloat3(&endPoint);
    DirectX::XMVECTOR axisDir = DirectX::XMVectorSubtract(E, S);

    // 축 길이의 제곱을 미리 한 번만 계산
    float axisLenSq = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(axisDir));

    // 맵 크기 계산 (기존 BuildCells와 동일한 방식)
    int sizeX = (int)(ground.GetWorldWidth() / cellSize);
    int sizeZ = (int)(ground.GetWorldDepth() / cellSize);

    // 축이 원래 지형 최고높이보다 위로 솟을 수 있으니, 그만큼 sizeY에 여유를 둠
    float maxPossibleHeight = std::max(ground.GetMaxHeight(), axisY + radius + shellThickness);
    int sizeY = (int)(maxPossibleHeight / cellSize) + 2;

    auto isSolid = [&](int x, int y, int z) -> bool
    {
        DirectX::XMFLOAT3 worldF = { x * cellSize, y * cellSize, z * cellSize };
        DirectX::XMVECTOR P = DirectX::XMLoadFloat3(&worldF);

        float wx = worldF.x;
        float wz = worldF.z;
        float GroundY = ground.SampleHeight(wx, wz);
        bool baseSolid = (worldF.y <= GroundY);

        // 원래 지형(바닥)이었던 부분은 원 안쪽이라도 그대로 보존 -> 밟고 다닐 바닥 유지
        if (true == baseSolid)
            return true;

        // cell 위치를 중심축에 투영 후, 비율 구하기
        DirectX::XMVECTOR toP = DirectX::XMVectorSubtract(P, S);
        float rawT = DirectX::XMVectorGetX(DirectX::XMVector3Dot(toP, axisDir)) / axisLenSq;
        float clampedT = std::max(0.0f, std::min(rawT, 1.0f));

        // S + axisDir × clampedT
        DirectX::XMVECTOR closeestPoint = DirectX::XMVectorAdd(S, DirectX::XMVectorScale(axisDir, clampedT));
        // 검사하려는 셀과 실제 거리
        float dist = DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMVectorSubtract(P, closeestPoint)));

        // 비율이 양끝이면 검사로 끝에를 연다
        bool inStart = (rawT < 0.0f);
        bool inEnd = (rawT > 1.0f);

        bool openShell = (inStart && openAtStart) || (inEnd && openAtEnd);

        // 바깥 반지름 안쪽이면 지형과 상관없이 새로운 지형 생성
        bool forcedSolid = (dist <= outerRadius) && (!openShell);
        bool solidHere = baseSolid || forcedSolid;

        if (dist <= radius)
            return false;
        return solidHere;
    };
    BuildFromVolumeSource(isSolid, sizeX, sizeY, sizeZ, cellSize);
    ValidateWalkable();
    BuildSurfaceCache();
}



void VoxelGrid::ValidateWalkable()
{
    for (auto& cell : m_Cells)
    {
        // 표면 복셀만 판정 (위쪽 y+1이 비어있어야 표면)
        if (false == IsSurface(cell.x, cell.y, cell.z))
        {
            SetCell(cell.x, cell.y, cell.z, CellType::Blocked);
            continue;
        }




        //// 조건 1 — 이웃 4방향 표면 복셀과 높이차 검사
        //// 패스 1이 끝난 뒤라 GetSurfaceY로 정확한 이웃 높이를 알 수 있음
        //const int dx[] = { 1, -1, 0,  0 };
        //const int dz[] = { 0,  0, 1, -1 };

        //for (int d = 0; d < 4; d++)
        //{
        //    int nx = cell.x + dx[d];
        //    int nz = cell.z + dz[d];

        //    // 맵 경계 밖은 Blocked
        //    if (nx < 0 || nx >= m_SizeX || nz < 0 || nz >= m_SizeZ)
        //    {
        //        walkable = false;
        //        break;
        //    }

        //    int neighborSurfaceY = GetSurfaceY(nx, nz);
        //    int heightDiff = std::abs(cell.y - neighborSurfaceY);

        //    // 이웃과 높이차가 복셀 2개 이상이면 경사가 너무 가파름
        //    if (heightDiff >= 2)
        //    {
        //        walkable = false;
        //        break;
        //    }
        //}



        // 조건 2 — 머리 위 공간(헤드룸) 확인
        // 터널/아치 구조에서는 컬럼마다 천장 위치가 다르므로, 표면(cell.y) 바로 위부터
        // HEADROOM_CELLS칸이 전부 비어있어야 실제로 지나갈 수 있는 통로로 인정.
        bool walkable = CheckWalkableCondition(cell.x, cell.y, cell.z);

        // 조건 3 — TODO : 장애물 마킹 (동적 변경 시)
        
\
        // 청크에도 반영 (FlowField 계산 시 빠른 접근용)
        SetCell(cell.x, cell.y, cell.z, walkable ? CellType::Walkable : CellType::Blocked);
    }

}

bool VoxelGrid::IsSurface(int x, int y, int z) const
{
    // 위쪽이 Empty이면 표면
    int above = y + 1;
    if (above >= m_SizeY) return true;
    return GetCell(x, above, z) == CellType::Empty;
}

int VoxelGrid::GetSurfaceY(int x, int z) const
{
    // 위에서 아래로 내려오면서 처음으로 채워진 복셀 찾기
    for (int y = m_SizeY - 1; y >= 0; y--)
    {
        // Empty만 스킵 — Walkable이든 Blocked든 "채워진 것"으로 취급
        if (GetCell(x, y, z) == CellType::Empty)  continue;
        return y;
    }
    return 0;
}


VoxelGrid::SurfaceSpan VoxelGrid::GetSurfaceYList(int x, int z) const
{
    if (x < 0 || x >= m_SizeX || z < 0 || z >= m_SizeZ) return { nullptr, 0 };
    // 캐시 미구축 상태
    if (m_SurfaceChunks.empty())    return { nullptr, 0 };

    int cx = x / SurfaceChunk::CHUNK_SIZE;
    int cz = z / SurfaceChunk::CHUNK_SIZE;
    int lx = x % SurfaceChunk::CHUNK_SIZE;
    int lz = z % SurfaceChunk::CHUNK_SIZE;
    int chunkIdx = cx + m_SurfaceChunkCountX * cz;

    const auto& col = m_SurfaceChunks[chunkIdx].At(lx, lz);
    return { col.surfaces.data(), (int)col.count };
}

bool VoxelGrid::FindNearestWalkable(const Math::Vector3& worldPos, int& outX, int& outY, int& outZ, int maxSearchRadius) const
{
    // worldPos에 가장 가까운 셀 좌표 (중심 정렬 좌표계이므로 반올림 사용)
    int centerX = (int)std::round(worldPos.GetX() / m_CellSize);
    int centerZ = (int)std::round(worldPos.GetZ() / m_CellSize);

    // 반지름 0(중심 칸 자신)부터 시작해서 바깥으로 넓혀가며 검사
    for (int radius = 0; radius <= maxSearchRadius; radius++)
    {
        bool found = false;
        int bestX = 0, bestY = 0, bestZ = 0;
        float bestDistSq = FLT_MAX;

        // 이번 반지름에 해당하는 "링"만 순회 (정사각형 테두리)
        for (int dx = -radius; dx <= radius; dx++)
        {
            for (int dz = -radius; dz <= radius; dz++)
            {
                // 링의 테두리만: 이전 반지름에서 이미 검사한 내부는 건너뜀
                if (std::max(std::abs(dx), std::abs(dz)) != radius) continue;

                int x = centerX + dx;
                int z = centerZ + dz;
                if (x < 0 || x >= m_SizeX || z < 0 || z >= m_SizeZ) continue;

                // 이 컬럼의 모든 표면 후보 중, worldPos.y와 가장 가까운 Walkable 층을 찾음
                VoxelGrid::SurfaceSpan surfaces = GetSurfaceYList(x, z);
                for(auto sy : surfaces)
                {
                    if (false == IsWalkable(x, sy, z)) continue;

                    Math::Vector3 candidatePos = GetWorldPos(x, sy, z);
                    float distSq = Math::LengthSquare(candidatePos - worldPos);

                    if (distSq < bestDistSq)
                    {
                        bestDistSq = distSq;
                        bestX = x; bestY = sy; bestZ = z;
                        found = true;
                    }
                }
            }
        }

        if (true == found)
        {
            outX = bestX; outY = bestY; outZ = bestZ;
            return true; // 이 반지름에서 찾았으면 더 바깥은 볼 필요 없음(더 가까울 수 없음)
        }
    }

    return false; // maxSearchRadius 안에 Walkable 셀이 아예 없음
}


void VoxelGrid::AllocateChunks()
{
    // 올림 나눗셈으로 각 축에 필요한 청크 개수 계산
    // 예: m_SizeX=20 → (20+15)/16 = 2개 청크 (0~15, 16~31 중 16~19만 사용)
    m_ChunkCountX = (m_SizeX + VoxelChunk::CHUNK_SIZE - 1) / VoxelChunk::CHUNK_SIZE;
    m_ChunkCountY = (m_SizeY + VoxelChunk::CHUNK_SIZE - 1) / VoxelChunk::CHUNK_SIZE;
    m_ChunkCountZ = (m_SizeZ + VoxelChunk::CHUNK_SIZE - 1) / VoxelChunk::CHUNK_SIZE;

    size_t totalChunks = (size_t)m_ChunkCountX * m_ChunkCountY * m_ChunkCountZ;

    // VoxelChunk 기본 생성자가 이미 Empty로 채우므로 별도 fill 불필요
    m_Chunks.clear();
    m_Chunks.resize(totalChunks);
}

void VoxelGrid::AllocateSurfaceCache()
{
    m_SurfaceChunkCountX = (m_SizeX + SurfaceChunk::CHUNK_SIZE - 1) / SurfaceChunk::CHUNK_SIZE;
    m_SurfaceChunkCountZ = (m_SizeZ + SurfaceChunk::CHUNK_SIZE - 1) / SurfaceChunk::CHUNK_SIZE;

    size_t total = (size_t)m_SurfaceChunkCountX * m_SurfaceChunkCountZ;
    m_SurfaceChunks.clear();
    m_SurfaceChunks.resize(total);   // 기본 생성자가 count=0으로 초기화
}

void VoxelGrid::RefreshSurfaceColumn(int x, int z)
{
    int cx = x / SurfaceChunk::CHUNK_SIZE;
    int cz = z / SurfaceChunk::CHUNK_SIZE;
    int lx = x % SurfaceChunk::CHUNK_SIZE;
    int lz = z % SurfaceChunk::CHUNK_SIZE;
    int chunkIdx = cx + m_SurfaceChunkCountX * cz;

    auto& col = m_SurfaceChunks[chunkIdx].At(lx, lz);
    col.count = 0;

    bool prevFilled = false;
    for (int y = 0; y < m_SizeY; y++)
    {
        bool currFilled = (GetCell(x, y, z) != CellType::Empty);
        if (prevFilled && !currFilled)
        {
            // INLINE_CAPACITY(4) 초과하는 극단적 케이스는 뒤 표면을 버림 —
            // 동굴이 4겹 이상 겹치는 경우는 사실상 없다고 가정
            // 아파트 같은 구조를 만약 만들게 되면 해당 로직 수정 필요.
            if (col.count < SurfaceChunk::SurfaceColumn::INLINE_CAPACITY)
            {
                col.surfaces[col.count++] = y - 1;
            }
        }
        prevFilled = currFilled;
    }

    if (prevFilled && col.count < SurfaceChunk::SurfaceColumn::INLINE_CAPACITY)
    {
        col.surfaces[col.count++] = m_SizeY - 1;
    }
}

void VoxelGrid::BuildSurfaceCache()
{
    AllocateSurfaceCache();  // 청크 배열 할당

    for (int z = 0; z < m_SizeZ; z++)
    {
        for (int x = 0; x < m_SizeX; x++)
        {
            RefreshSurfaceColumn(x, z);   // 컬럼당 1회 전체 스캔
        }
    }
}

void VoxelGrid::ToChunkCoord(int x, int y, int z, int& chunkIndex, int& lx, int& ly, int& lz) const
{
    int cx = x / VoxelChunk::CHUNK_SIZE;
    int cy = y / VoxelChunk::CHUNK_SIZE;
    int cz = z / VoxelChunk::CHUNK_SIZE;
    lx = x % VoxelChunk::CHUNK_SIZE;
    ly = y % VoxelChunk::CHUNK_SIZE;
    lz = z % VoxelChunk::CHUNK_SIZE;
    chunkIndex = cx + m_ChunkCountX * (cz + m_ChunkCountZ * cy);
}

void VoxelGrid::BuildInstanceList(std::vector<VoxelRenderer::InstanceData>& outInstances,
    std::vector<CellCoord>* outCoords) const
{
    outInstances.clear();
    outInstances.reserve(m_Cells.size());
    if (outCoords) { outCoords->clear(); outCoords->reserve(m_Cells.size()); }

    for (const auto& cell : m_Cells)
    {
        CellType t = GetCell(cell.x, cell.y, cell.z);
        if (t == CellType::Empty) continue;

        VoxelRenderer::InstanceData inst = {};
        inst.position[0] = cell.x * m_CellSize;
        inst.position[1] = cell.y * m_CellSize;
        inst.position[2] = cell.z * m_CellSize;
        inst.scale = m_CellSize;
        inst.colorType = (t == CellType::Walkable) ? 0 : 1;
        outInstances.push_back(inst);

        if (outCoords) outCoords->push_back({ (int16_t)cell.x, (int16_t)cell.y, (int16_t)cell.z });
    }
}

void VoxelGrid::SetCell(int x, int y, int z, CellType type)
{
    if (x < 0 || x >= m_SizeX) return;
    if (y < 0 || y >= m_SizeY) return;
    if (z < 0 || z >= m_SizeZ) return;

    int chunkIdx, lx, ly, lz;
    ToChunkCoord(x, y, z, chunkIdx, lx, ly, lz);
    m_Chunks[chunkIdx].Set(lx, ly, lz, type);
}

bool VoxelGrid::RaycastVoxel(const Math::Vector3& origin, const Math::Vector3& dir, float maxDistance, int& outX, int& outY, int& outZ) const
{
    const float half = m_CellSize * 0.5f;
    // 복셀 크기보다 작게 잡아야 뚫고 가는거 방지
    const float step = m_CellSize * 0.25f;

    // 직전 검사한 복셀 좌표 저장
    int lastX = INT_MIN, lastY = INT_MIN, lastZ = INT_MIN;
    float traveled = 0.0f;

    while (traveled < maxDistance)
    {
        Math::Vector3 samplePos = origin + dir * traveled;

        int x = (int)std::floor((samplePos.GetX() + half) / m_CellSize);
        int y = (int)std::floor((samplePos.GetY() + half) / m_CellSize);
        int z = (int)std::floor((samplePos.GetZ() + half) / m_CellSize);

        // 맵 밖이면 통과
        if (x >= 0 && x < m_SizeX && y >= 0 && y < m_SizeY && z >= 0 && z < m_SizeZ)
        {
            if (x != lastX || y != lastY || z != lastZ)
            {
                if (GetCell(x, y, z) != CellType::Empty)
                {
                    outX = x; outY = y; outZ = z;
                    return true;
                }
                lastX = x; lastY = y; lastZ = z;
            }
        }

        traveled += step;
    }
    return false;
}

VoxelGrid::CellType VoxelGrid::GetCell(int x, int y, int z) const
{
    if (x < 0 || x >= m_SizeX) return CellType::Blocked;
    if (y < 0 || y >= m_SizeY) return CellType::Blocked;
    if (z < 0 || z >= m_SizeZ) return CellType::Blocked;

    int chunkIdx, lx, ly, lz;
    ToChunkCoord(x, y, z, chunkIdx, lx, ly, lz);
    return m_Chunks[chunkIdx].Get(lx, ly, lz);
}

bool VoxelGrid::IsWalkable(int x, int y, int z) const
{
    return GetCell(x, y, z) == CellType::Walkable;
}

Math::Vector3 VoxelGrid::GetWorldPos(int x, int y, int z) const
{
    return Math::Vector3(x * m_CellSize, y * m_CellSize,z * m_CellSize);
}



bool VoxelGrid::IsCellExposed(int x, int y, int z) const
{
    // y == m_SizeY-1(맵 천장)은 자동 노출로 치지 않음 
    // 그리드 상한에 닿은 셀은 렌더에서 빠질 수 있음.
    bool isBottom = (y == 0);
    bool isWallX = (x == 0 || x == m_SizeX - 1);
    bool isWallZ = (z == 0 || z == m_SizeZ - 1);
    if (isBottom || isWallX || isWallZ) return true;

    static const int dx[] = { 1, -1, 0, 0, 0, 0 };
    static const int dy[] = { 0, 0, 1, -1, 0, 0 };
    static const int dz[] = { 0, 0, 0, 0, 1, -1 };

    for (int d = 0; d < 6; d++)
    {
        int nx = x + dx[d];
        int ny = y + dy[d];
        int nz = z + dz[d];
        if (nx < 0 || nx >= m_SizeX || ny < 0 || ny >= m_SizeY || nz < 0 || nz >= m_SizeZ)
            continue;
        if (GetCell(nx, ny, nz) == CellType::Empty) return true;
    }
    return false;
}

int VoxelGrid::GetGroundY(int x, int z) const
{
    // RefreshSurfaceColumn이 아래->위 순서로 채우므로 data[0]이 최하단 표면.
    // 터널처럼 컬럼에 표면이 여러 개여도 원래 지면을 잡는다.
    SurfaceSpan span = GetSurfaceYList(x, z);
    return (span.count > 0) ? (int)span.data[0] : -1;
}

bool VoxelGrid::CheckWalkableCondition(int x, int y, int z) const
{
    constexpr int HEADROOM_CELLS = 3 + 1;
    for (int k = 1; k <= HEADROOM_CELLS; k++)
    {
        int headY = y + k;
        if (headY >= m_SizeY) break;
        if (GetCell(x, headY, z) != CellType::Empty) return false;
    }
    return true;
}

void VoxelGrid::AddNarrowingCliffs(const DirectX::XMFLOAT3& start, const DirectX::XMFLOAT3& end,
    float outerHalfWidth, float minHalfWidth, float cliffHeight)
{
    // BuildFromVolumeSource와 달리 그리드를 다시 채우지 않음
    // 이미 완성된 지형(터널 등) 위에 절벽 복셀만 추가하는 증분 방식
    const float cellSize = m_CellSize;

    const float dirX = end.x - start.x;
    const float dirZ = end.z - start.z;
    const float axisLenSq = dirX * dirX + dirZ * dirZ;
    if (axisLenSq < 1e-6f)
    {
        return;
    }
    const float axisLen = std::sqrt(axisLenSq);
    const int   cliffCells = std::max(1, (int)std::round(cliffHeight / cellSize));

    std::vector<std::pair<int, int>> touchedColumns;   // 절벽을 세운 컬럼들
    std::unordered_set<int64_t>      touchedSet;       // 위 목록의 멤버십 검사용(y=0 고정 컬럼 키)
    int clampedCount = 0;                              // 그리드 상한에 잘린 컬럼 수

    // 통로 밖을 맵 경계까지 채우므로 x/z 전체를 순회한다.
    // rawT 필터가 대부분을 즉시 걸러내고, Startup에서 1회만 도는 비용이라 허용
    for (int z = 0; z < m_SizeZ; ++z)
    {
        for (int x = 0; x < m_SizeX; ++x)
        {
            const float wx = x * cellSize;
            const float wz = z * cellSize;

            // start->end 축에 투영해 진행률(t)과 축까지의 수직거리 구하기 (2D, xz만)
            const float toPx = wx - start.x;
            const float toPz = wz - start.z;
            const float rawT = (toPx * dirX + toPz * dirZ) / axisLenSq;
            if (rawT < 0.0f || rawT > 1.0f) continue;   // 축 구간 밖 - 자유 통행 유지

            const float dist = std::abs(toPx * dirZ - toPz * dirX) / axisLen;

            // 대칭 깔때기: t=0.5(중앙)에서 minHalfWidth, t=0/1(양끝)에서 outerHalfWidth
            const float shape = std::abs(rawT - 0.5f) * 2.0f;   // 0(중앙) ~ 1(양끝)
            const float halfWidth = minHalfWidth + (outerHalfWidth - minHalfWidth) * shape;

            if (dist <= halfWidth) continue;   // 통로 내부 - 건드리지 않음

            // --- 통로 밖: 이 컬럼의 현재 지면 위로 절벽을 쌓음 ---
            const int groundY = GetGroundY(x, z);
            if (groundY < 0) continue;         // 표면 없는 컬럼(빈 공간) - 스킵

            int topY = groundY + cliffCells;
            if (topY >= m_SizeY) { topY = m_SizeY - 1; ++clampedCount; }

            for (int y = groundY; y <= topY; ++y)
                SetCell(x, y, z, CellType::Blocked);

            touchedColumns.push_back({ x, z });
            touchedSet.insert(MakeCellKey(x, 0, z));
        }
    }

    if (touchedColumns.empty())
    {
        return;
    }

    // 영향받은 컬럼들의 기존 렌더 엔트리를 한 번에 제거 (컬럼마다 스캔하지 않도록)
    // 이 과정에서 m_Cells 순서가 바뀐다 -> BuildInstanceList 이전에만 호출할 것
    m_Cells.erase(std::remove_if(m_Cells.begin(), m_Cells.end(), [&](const CellCoord& c) {
        return touchedSet.count(MakeCellKey(c.x, 0, c.z)) > 0;
        }), m_Cells.end());

    // 해당 컬럼만 재스캔해서 노출된 셀만 렌더 목록에 재등록 + 표면 캐시 갱신
    // (벽에 가려진 이웃 컬럼의 셀이 렌더 목록에 남을 수 있으나, 보이지 않으므로 시각적 무해)
    for (const auto& col : touchedColumns)
    {
        const int x = col.first, z = col.second;
        for (int y = 0; y < m_SizeY; ++y)
        {
            if (GetCell(x, y, z) == CellType::Empty) continue;
            if (!IsCellExposed(x, y, z))  continue;
            m_Cells.push_back({ (int16_t)x, (int16_t)y, (int16_t)z });
        }
        RefreshSurfaceColumn(x, z);
    }

    // ValidateWalkable()은 호출하지 않음 - 절벽 셀은 Blocked로 직접 지정했고,
    // 재호출하면 절벽 꼭대기가 Walkable로 승격되어 NPC가 절벽 위로 올라가버린다.
}

void VoxelGrid::OverwriteCells(const std::vector<DirectX::XMINT3>& cells, CellType type, TerrainEditDelta& outDelta)
{
    outDelta.removed.clear();
    outDelta.added.clear();

    std::vector<std::pair<int, int>> touchedCol;
    std::unordered_set<int64_t> touchedSet;

    for (const auto& c : cells)
    {
        if (false == IsInBounds(c.x, c.y, c.z))  continue;

        SetCell(c.x, c.y, c.z, type);

        int64_t colKey = MakeCellKey(c.x, 0, c.z);
        if (touchedSet.insert(colKey).second)
        {
            touchedCol.push_back({ c.x, c.z });
        }
    }
    if (touchedCol.empty())  return;

    // 빠질 엔트리 먼저 기록(erase 사용전에)
    for (const auto& c : m_Cells)
    {
        if (touchedSet.count(MakeCellKey(c.x, 0, c.z)) > 0)
        {
            outDelta.removed.emplace_back(DirectX::XMINT3{ c.x, 0, c.z });
        }
    }

    // 영향 컬럼의 기존 렌더 엔트리 일괄 제거 TODO : 인덱스 구조 변경시, 이 오버헤드도 같이 리팩토링 할것
    m_Cells.erase(std::remove_if(m_Cells.begin(), m_Cells.end(), [&](const CellCoord& c) {
        return touchedSet.count(MakeCellKey(c.x, 0, c.z)) > 0;
        }), m_Cells.end());



    // 1) 표면 캐시부터 최신화 (walkable 재판정이 이 캐시를 참조하므로 선행 필수)
    for (const auto& col : touchedCol)
    {
        RefreshSurfaceColumn(col.first, col.second);
    }

    // 2) walkable 재판정 -> m_Chunks 확정
    for (const auto& col : touchedCol)
    {
        const int x = col.first, z = col.second;
        for (int y = 0; y < m_SizeY; ++y)
        {
            if (GetCell(x, y, z) == CellType::Empty) continue;
            bool walkable = IsSurface(x, y, z) && CheckWalkableCondition(x, y, z);
            SetCell(x, y, z, walkable ? CellType::Walkable : CellType::Blocked);
        }
    }

    // 3) 확정된 타입으로 렌더 목록 재등록 (GetCell이 이제 최종 타입을 돌려줌)
    for (const auto& col : touchedCol)
    {
        const int x = col.first, z = col.second;
        for (int y = 0; y < m_SizeY; ++y)
        {
            CellType t = GetCell(x, y, z);       // walkable 재판정이 반영된 최종값
            if (t == CellType::Empty) continue;
            if (!IsCellExposed(x, y, z)) continue;
            m_Cells.push_back({ (int16_t)x, (int16_t)y, (int16_t)z });
            outDelta.added.push_back({ (int16_t)x, (int16_t)y, (int16_t)z, t });
        }
    }
}
