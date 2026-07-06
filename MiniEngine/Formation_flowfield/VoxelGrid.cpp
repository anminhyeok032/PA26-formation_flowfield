#include "VoxelGrid.h"
#include "HeightMap.h"
#include <algorithm>
#include <cmath>

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

void VoxelGrid::BuildFromHeightMap(const HeightMap& hm)
{
    BuildCells(hm);
    ValidateWalkable();
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

                bool isBottom = (y == 0);
                bool isWallX = (x == 0 || x == m_SizeX - 1);
                bool isWallZ = (z == 0 || z == m_SizeZ - 1);

                // 6방향 중 하나라도 Empty(또는 맵 밖)면 공기에 노출된 면이 있는 것
                bool isExposed = isBottom || isWallX || isWallZ;
                if (!isExposed)
                {
                    for (int d = 0; d < 6; d++)
                    {
                        int nx = x + dx[d];
                        int ny = y + dy[d];
                        int nz = z + dz[d];
                        if (nx < 0 || nx >= m_SizeX || ny < 0 || ny >= m_SizeY || nz < 0 || nz >= m_SizeZ)
                            continue;
                        if (GetCell(nx, ny, nz) == CellType::Empty)
                        {
                            isExposed = true;
                            break;
                        }
                    }
                }

                if (!isExposed) continue; // 완전히 파묻힌 내부 복셀은 렌더 스킵

                VoxelCell cell;
                cell.x = x;
                cell.y = y;
                cell.z = z;
                cell.type = CellType::Blocked; // ValidateWalkable에서 덮어씀
                m_Cells.push_back(cell);
            }
        }
    }
}

void VoxelGrid::BuildFromHeightMapWithTunnel(const HeightMap& ground,
    float startX, float startZ,
    float endX, float endZ,
    float tunnelHeightWorld,
    float tunnelRadiusWorld,
    float archThicknessWorld,
    bool  openAtStart,
    bool  openAtEnd)
{
    float cellSize = ground.GetVoxelSize();
    int sizeX = (int)(ground.GetWorldWidth() / cellSize);
    int sizeZ = (int)(ground.GetWorldDepth() / cellSize);

    // 아치가 지표면 위로 솟아오르는 만큼 맵 높이(sizeY)에 여유를 더함
    float extraWorldHeight = tunnelHeightWorld + archThicknessWorld + 1.0f; // +1은 여유분
    int sizeY = (int)((ground.GetMaxHeight() + extraWorldHeight) / cellSize) + 1;

    int archThicknessCells = std::max(1, (int)(archThicknessWorld / cellSize));

    // 터널 경로 벡터 (시작->끝)
    float dx = endX - startX;
    float dz = endZ - startZ;
    float pathLenSq = dx * dx + dz * dz;
    float pathLen = std::sqrt(pathLenSq);

    // 입/출구 개방 전환 구간 길이 — 이 거리 안에서 아치 높이가 0->최대로 서서히 자라남
    float transitionLen = std::max(cellSize, tunnelRadiusWorld * 2.0f);

    auto isSolid = [&](int x, int y, int z) -> bool
    {
        float wx = (x + 0.5f) * cellSize;
        float wz = (z + 0.5f) * cellSize;

        int groundY = (int)(ground.SampleHeight(wx, wz) / cellSize);
        groundY = std::max(0, std::min(groundY, sizeY - 1));

        // 1. 기존 지표면까지는 그대로 solid (터널 바닥, 기존 지형 유지)
        if (y <= groundY) return true;

        // 2. 이 컬럼이 터널 경로에서 얼마나 떨어져 있는지 계산 (선분 최단거리)
        float t = pathLenSq > 0.0f
            ? ((wx - startX) * dx + (wz - startZ) * dz) / pathLenSq
            : 0.0f;
        t = std::max(0.0f, std::min(t, 1.0f));

        float closestX = startX + t * dx;
        float closestZ = startZ + t * dz;
        float perpDist = std::sqrt((wx - closestX) * (wx - closestX) +
            (wz - closestZ) * (wz - closestZ));

        // 터널 폭 바깥이면 이 함수는 관여하지 않음 -> 기존과 동일하게 지표면 위는 공중
        if (perpDist > tunnelRadiusWorld) return false;

        // 3. 시작/끝 지점까지의 경로상 거리 (양 끝에서 0이 되도록)
        float distFromStart = t * pathLen;
        float distFromEnd = (1.0f - t) * pathLen;

        // 개방 전환 비율 계산: 강제 개방하는 쪽 끝에서는 0에 가까울수록 openFrac도 0에 가까워짐
        float openFracStart = openAtStart ? std::min(1.0f, distFromStart / transitionLen) : 1.0f;
        float openFracEnd = openAtEnd ? std::min(1.0f, distFromEnd / transitionLen) : 1.0f;
        float openFrac = std::min(openFracStart, openFracEnd); // 둘 중 더 "닫힌"(개방 안 된) 쪽을 따름

        // 4. 아치 높이 = 목표 높이(헤드룸+두께) * 개방비율
        //    양 끝(openFrac=0)에서는 추가 높이 0 -> 지표면 그대로, 즉 완전 개방
        float addHeight = (tunnelHeightWorld + archThicknessWorld) * openFrac;
        int addCells = (int)(addHeight / cellSize);

        if (addCells <= 0) return false; // 이 지점은 아치 없음 -> 공중 (입/출구 부분)

        int archTopY = groundY + addCells;
        int archBotY = std::max(groundY, archTopY - archThicknessCells); // 지표면 아래로는 못 내려감

        if (archBotY <= archTopY && y >= archBotY && y <= archTopY)
            return true; // 아치 본체 -> solid

        return false; // 터널 내부 간격 또는 아치 위 공중 -> air
    };

    BuildFromVolumeSource(isSolid, sizeX, sizeY, sizeZ, cellSize);
    ValidateWalkable();
}


void VoxelGrid::BuildCells(const HeightMap& hm)
{
    m_Cells.clear();
    m_CellSize = hm.GetVoxelSize(); // 복셀 크기

    // 복셀 수를 맵 크기 / 복셀 크기로 결정
    float worldW = hm.GetWorldWidth();  // = 픽셀수 × worldScale
    float worldD = hm.GetWorldDepth();

    m_SizeX = (int)(worldW / m_CellSize); // 복셀 수 = 맵크기 / 복셀크기
    m_SizeZ = (int)(worldD / m_CellSize);
    m_SizeY = (int)(hm.GetMaxHeight() / m_CellSize) + 1;

    // 청크 배열 할당 (전체 Empty로 초기화됨)
    AllocateChunks();

    // 1단계: 높이맵 읽어서 3D 그리드에 채워진 공간 마킹
    //        m_Grid만 채움 (m_Cells는 아직 안 넣음)
    for (int z = 0; z < m_SizeZ; z++)
    {
        for (int x = 0; x < m_SizeX; x++)
        {
            // 이 복셀의 월드 중심 좌표
            float wx = (x + 0.5f) * m_CellSize;
            float wz = (z + 0.5f) * m_CellSize;

            
            float worldH = hm.SampleHeight(wx, wz);
            //float worldH = hm.GetHeight(wx, wz);
            int   surfY = (int)(worldH / m_CellSize);
            surfY = std::max(0, std::min(surfY, m_SizeY - 1));

            // y=0 ~ surfY 공간을 Blocked로 마킹 (패스 2에서 Walkable로 바뀔 수 있음)
            for (int y = 0; y <= surfY; y++)
            {
                SetCell(x, y, z, CellType::Blocked);  // 배열 직접 대입 대신 SetCell
            }
        }
    }

    // 2단계: 렌더링할 복셀만 m_Cells에 추가
    //        조건: 표면 OR 바닥(y=0) OR 4면 벽(맵 가장자리)
    for (int z = 0; z < m_SizeZ; z++)
    {
        for (int x = 0; x < m_SizeX; x++)
        {
            int surfY = GetSurfaceY(x, z);

            for (int y = 0; y <= surfY; y++)
            {
                // 이 복셀을 렌더링해야 하는지 판단
                bool isTopSurface = (y == surfY);
                bool isBottom = (y == 0);
                bool isWallX = (x == 0 || x == m_SizeX - 1);
                bool isWallZ = (z == 0 || z == m_SizeZ - 1);

                // 4면 벽: 가장자리 x,z 열의 모든 y
                // 바닥: 모든 x,z에서 y=0
                // 표면: 각 x,z의 최상단 y
                // 벽면: 이웃 셀보다 높은 y는 옆면이 공기에 노출됨
                bool isExposedSide = false;
                if (!isTopSurface && !isBottom && !isWallX && !isWallZ)
                {
                    const int dx[] = { 1, -1,  0, 0 };
                    const int dz[] = { 0,  0,  1,-1 };

                    for (int d = 0; d < 4; d++)
                    {
                        int nx = x + dx[d];
                        int nz = z + dz[d];

                        // 맵 밖이면 공기에 노출된 것
                        if (nx < 0 || nx >= m_SizeX ||
                            nz < 0 || nz >= m_SizeZ)
                        {
                            isExposedSide = true;
                            break;
                        }

                        // 이웃의 surfY보다 현재 y가 높으면
                        // 이 y는 이웃 방향에서 보일 수 있는 옆면
                        if (y > GetSurfaceY(nx, nz))
                        {
                            isExposedSide = true;
                            break;
                        }
                    }
                }

                // 내부면 렌더 스킵
                if (!isTopSurface && !isBottom && !isWallX && !isWallZ && !isExposedSide)   continue;

                VoxelCell cell;
                cell.x = x;
                cell.y = y;
                cell.z = z;
                cell.type = CellType::Blocked; // 패스 2에서 덮어씀
                m_Cells.push_back(cell);
            }
        }
    }
}

void VoxelGrid::ValidateWalkable()
{
    for (auto& cell : m_Cells)
    {
        // 표면 복셀만 판정 (위쪽 y+1이 비어있어야 표면)
        if (false == IsSurface(cell.x, cell.y, cell.z))
        {
            cell.type = CellType::Blocked; // 내부 복셀은 항상 Blocked
            continue;
        }

        // 표면 복셀 walkable 판정 조건들
        bool walkable = true;


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
        // GetSurfaceY 비교 같은 우회 로직 불필요 — GetCell이 이미 정확한 Empty/Blocked를
        // 반환하므로 그대로 신뢰하면 됨 (컬럼당 표면 1개 가정이 깨져도 안전).
        const int HEADROOM_CELLS = 3 + 1; // 캐릭터 키 3칸 + 여유 1칸

        for (int k = 1; k <= HEADROOM_CELLS; k++)
        {
            int headY = cell.y + k;

            // 맵 맨 위를 넘어가면 그 위는 무조건 뚫려있는 것으로 간주하고 통과
            if (headY >= m_SizeY) break;

            if (GetCell(cell.x, headY, cell.z) != CellType::Empty)
            {
                walkable = false; // 천장(아치)이 너무 낮음 -> 통행 불가
                break;
            }
        }

        // 조건 3 — TODO : 장애물 마킹 (동적 변경 시)
        

        cell.type = walkable ? CellType::Walkable : CellType::Blocked;

        // 청크에도 반영 (FlowField 계산 시 빠른 접근용)
        SetCell(cell.x, cell.y, cell.z, cell.type);
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

std::vector<int> VoxelGrid::GetSurfaceYList(int x, int z) const
{
    // (x,z) 컬럼을 아래(y=0)에서 위로 훑으면서
    // "Solid(채워짐) 다음에 Empty가 나오는" 경계를 전부 표면으로 기록.
    // 단층 지형이면 결과가 1개, 동굴이 있으면 2개 이상이 됨.
    std::vector<int> surfaces;

    bool prevFilled = false; // y=-1은 가상의 Empty(바닥 아래는 없음)로 취급
    for (int y = 0; y < m_SizeY; y++)
    {
        bool curFilled = (GetCell(x, y, z) != CellType::Empty);

        // Filled -> Empty로 바뀌는 경계 = 직전 y(prevFilled였던 y-1)가 밟을 수 있는 표면
        if (prevFilled && !curFilled)
        {
            surfaces.push_back(y - 1);
        }

        prevFilled = curFilled;
    }

    // 컬럼 맨 위(y = m_SizeY-1)까지 채워진 채로 끝난 경우(=그 위가 곧바로 맵 경계)도
    // 표면으로 포함시킴 - GetSurfaceY의 "above >= m_SizeY이면 표면" 규칙과 동일하게 맞춤
    if (true == prevFilled)
    {
        surfaces.push_back(m_SizeY - 1);
    }

    return surfaces;
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

void VoxelGrid::BuildInstanceList(std::vector<VoxelRenderer::InstanceData>& outInstances) const
{
    outInstances.clear();
    outInstances.reserve(m_Cells.size());

    for (const auto& cell : m_Cells)
    {
        // Empty는 렌더링하지 않음
        if (cell.type == CellType::Empty) continue;

        VoxelRenderer::InstanceData inst = {};
        inst.position[0] = cell.x * m_CellSize;
        inst.position[1] = cell.y * m_CellSize;
        inst.position[2] = cell.z * m_CellSize;
        inst.scale = m_CellSize;

        // Empty=0, Walkable=1(흰색), Blocked=2(빨강)
        inst.colorType = (cell.type == CellType::Walkable) ? 0 : 1;
        outInstances.push_back(inst);
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
