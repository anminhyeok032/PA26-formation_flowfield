#include "Player.h"
#include "VoxelGrid.h"
#include "NpcConstants.h"
#include "GameInput.h"
#include <cmath>


bool Player::Init(const VoxelGrid& grid)
{
    const float voxelSize = grid.GetCellSize();
    m_Width = voxelSize / 1.5f;
    m_HalfHeight = voxelSize * 3.0f / 1.5f;


    const float cellSize = grid.GetCellSize();
    const int cx = grid.GetSizeX() / 2;
    const int cz = grid.GetSizeZ() / 2;
    const int cy = grid.GetSurfaceY(cx, cz);
    
    m_CurrCell = { cx, cy, cz };
    m_TargetCell = m_CurrCell;

    m_Position = GetStandPos(grid, m_CurrCell);
    m_TargetWorldPos = m_Position;
    m_CellChanged = false;

    return true;
}


Math::Vector3 Player::GetStandPos(const VoxelGrid& grid, const DirectX::XMINT3& cell) const
{
    Math::Vector3 cellCenter = grid.GetWorldPos(cell.x, cell.y, cell.z);
    const float cellSize = grid.GetCellSize();

    // NpcManager::GetNpcStandPos와 동일 공식
    // 셀 중심 -> 셀 윗면(절반) -> 반높이 -> 여유 0.1
    const float standY = cellCenter.GetY() + (cellSize * 0.5f) + m_HalfHeight + 0.1f;
    return Math::Vector3(cellCenter.GetX(), standY, cellCenter.GetZ());
}

bool Player::PickInputCell(const VoxelGrid& grid, DirectX::XMINT3& outCell)
{
    // ix = 오른쪽, iz = 앞
    float ix = 0.0f, iz = 0.0f;
    if (GameInput::IsPressed(GameInput::kKey_up))    iz += 1.0f;
    if (GameInput::IsPressed(GameInput::kKey_down))  iz -= 1.0f;
    if (GameInput::IsPressed(GameInput::kKey_right)) ix += 1.0f;
    if (GameInput::IsPressed(GameInput::kKey_left))  ix -= 1.0f;

    if (ix == 0.0f && iz == 0.0f) return false;

    // OrbitCamera의 orientation = MakeYRotation(h) * MakeXRotation(p)
    // world = ix * right + iz * forward
    const float s = std::sin(m_MoveYaw);
    const float c = std::cos(m_MoveYaw);
    const float wx = -ix * c + iz * s;
    const float wz = ix * s + iz * c;

    // 셀 이동은 8방향뿐 연속 각도를 45도 단위로 반올림해 확정
    const float angle = std::atan2(wz, wx);
    const int oct = ((int)std::lround(angle / (DirectX::XM_PI / 4.0f)) + 8) % 8;

    static const int kDx[8] = { 1, 1, 0, -1, -1, -1,  0,  1 };
    static const int kDz[8] = { 0, 1, 1,  1,  0, -1, -1, -1 };

    const int dx = kDx[oct];
    const int dz = kDz[oct];

    // NPC와 같은 함수 로직 사용
    GetWalkableNeighbors(grid, m_CurrCell, m_NeighborScratch);

    for (const auto& n : m_NeighborScratch)
    {
        if (n.pos.x - m_CurrCell.x == dx && n.pos.z - m_CurrCell.z == dz)
        {
            outCell = n.pos;
            return true;
        }
    }

    // 대각선이 막혔으면 축 하나만이라도 - 벽에 비스듬히 붙었을 때 멈추지 않게
    if (dx != 0 && dz != 0)
    {
        for (const auto& n : m_NeighborScratch)
        {
            const int ndx = n.pos.x - m_CurrCell.x;
            const int ndz = n.pos.z - m_CurrCell.z;

            if ((ndx == dx && ndz == 0) || (ndx == 0 && ndz == dz))
            {
                outCell = n.pos;
                return true;
            }
        }
    }
    return false;
}


void Player::Update(const VoxelGrid& grid, float dt)
{
    if (m_CurrCell.x < 0) return;

    Math::Vector3 delta = m_TargetWorldPos - m_Position;
    const float distSq = Math::LengthSquare(delta);

    // NpcManager::Update의 ARRIVE_EPS_SQ와 같은 기준
    constexpr float ARRIVE_EPS_SQ = 0.0001f;

    if (distSq < ARRIVE_EPS_SQ)
    {
        // 도착 - 다음 셀 결정 NPC의 AdvanceCell 자리에 입력 판정이 들어간다
        m_Position = m_TargetWorldPos;
        m_CurrCell = m_TargetCell;

        DirectX::XMINT3 next;
        if (PickInputCell(grid, next))
        {
            m_TargetCell = next;
            m_TargetWorldPos = GetStandPos(grid, next);

            // 셀 전환 확정 시점에 알린다 - 도착까지 기다리면 필드가 한 칸씩 뒤처진다
            m_CellChanged = true;
        }
        return;
    }

    const float step = NPC_SPEED * m_SpeedScale * dt;
    if (step * step >= distSq)
    {
        m_Position = m_TargetWorldPos;   // 오버슈트 방지
    }
    else
    {
        m_Position = m_Position + Math::Normalize(delta) * step;
    }
}


NpcRenderer::InstanceData Player::MakeInstance() const
{
    NpcRenderer::InstanceData inst = {};
    inst.position[0] = m_Position.GetX();
    inst.position[1] = m_Position.GetY();
    inst.position[2] = m_Position.GetZ();
    inst.scaleXZ = m_Width;     
    inst.scaleY = m_HalfHeight;
    inst.colorType = 10;           // 0=기본, 1=선택됨, 10=플레이어
    return inst;
}
