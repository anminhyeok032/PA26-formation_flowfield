#include "Player.h"
#include "VoxelGrid.h"
#include "NpcConstants.h"
#include "GameInput.h"
#include <cmath>

namespace
{
    // 중앙에서 시작. 못 찾으면 나선형으로 넓혀가며 재시도한다.
    // 맵 중앙이 물이나 절벽일 수 있어서 한 번만 찔러보면 실패한다
    constexpr int SPAWN_SEARCH_RINGS = 16;
    constexpr int SPAWN_RING_STEP = 4;      // 셀 단위 - 촘촘히 돌 이유가 없다
}

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

    // NpcManager::GetNpcStandPos와 동일 공식.
    // 셀 중심 -> 셀 윗면(절반) -> 반높이 -> 여유 0.1
    const float standY = cellCenter.GetY() + (cellSize * 0.5f) + m_HalfHeight + 0.1f;
    return Math::Vector3(cellCenter.GetX(), standY, cellCenter.GetZ());
}

bool Player::PickInputCell(const VoxelGrid& grid, DirectX::XMINT3& outCell)
{
    int dx = 0, dz = 0;
    if (GameInput::IsPressed(GameInput::kKey_up))    dz += 1;
    if (GameInput::IsPressed(GameInput::kKey_down))  dz -= 1;
    if (GameInput::IsPressed(GameInput::kKey_right)) dx -= 1;
    if (GameInput::IsPressed(GameInput::kKey_left))  dx += 1;

    if (dx == 0 && dz == 0) return false;

    // 입력 방향을 좌표에 그냥 더하면 y를 알 수 없다.
    // 계단은 y가 바뀌고, 그 값은 GetWalkableNeighbors가 이미 계산해 뒀다.
    // NPC와 같은 함수를 쓰므로 통행 규칙이 자동으로 일치한다
    GetWalkableNeighbors(grid, m_CurrCell, m_NeighborScratch);

    for (const auto& n : m_NeighborScratch)
    {
        if (n.pos.x - m_CurrCell.x == dx && n.pos.z - m_CurrCell.z == dz)
        {
            outCell = n.pos;
            return true;
        }
    }

    // 대각선이 막혔으면 축 하나만이라도 시도 - 벽에 비스듬히 붙었을 때
    // 완전히 멈추지 않고 미끄러지게 한다
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
