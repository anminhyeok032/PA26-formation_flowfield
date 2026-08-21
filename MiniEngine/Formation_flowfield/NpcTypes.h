#pragma once
#include "CorridorFlowField.h"
#include "NpcConstants.h"
#include <DirectXMath.h>
#include <unordered_set>
#include <vector>
#include <memory>

// NpcManager 비대화로 인해 분리된 순수 데이터 구조체 모음
// (NpcMoveData / LeafGroup / NpcGroup)

struct NpcMoveData
{
    std::vector<Math::Vector3>      position;           // 현재 좌표
    std::vector<Math::Vector3>      targetWorldPos;     // 목표 셀 월드 좌표
    std::vector<DirectX::XMINT3>    currCell;           // 현재 셀
    std::vector<DirectX::XMINT3>    targetCell;         // 목표 셀
    std::vector<uint8_t>            active;             // 이동중 = 1;
    std::vector<float>              halfHeight;         // scaleY 복사본

    std::vector<DirectX::XMINT3> lastDir;               // 직전 이동 방향 (dx,dy,dz)

    std::vector<float>  blockedTime;                    // 다른 NPC가 길막한 시간
    std::vector<int>    leafId;                         // 해당 NPC가 속한 leaf 인덱스(-1 = 무소속)

    std::vector<uint8_t> stopReason;                    // 0=정상도착, 1=필드밖(무효화) - 지형변경으로 인한

    std::vector<uint8_t>         state;                 // NPC_STATE_*
    std::vector<float>           stateTimer;            // Wander 대기 / Lost 타임아웃 겸용
    std::vector<float>           propagationTimer;      // Alerted 지연 누적
    std::vector<uint32_t>        noiseSeed;             // 개체별 결정론적 난수 시드



    size_t size() const { return position.size(); }     // 길막당한 프레임 체크용

    void Resize(size_t n)
    {
        position.resize(n);
        targetWorldPos.resize(n);
        currCell.resize(n);
        targetCell.resize(n);
        active.resize(n);
        halfHeight.resize(n);

        lastDir.resize(n, { 0,0,0 });

        blockedTime.assign(n, 0.0f);
        //congestionTime.assign(n, 0.0f);

        leafId.resize(n, -1);
        stopReason.resize(n, 0);

        state.assign(n, NPC_STATE_IDLE);
        stateTimer.assign(n, 0.0f);
        propagationTimer.assign(n, 0.0f);
        noiseSeed.resize(n);
        for (size_t i = 0; i < n; ++i)
        {
            noiseSeed[i] = (uint32_t)(i * 2654435761u) | 1u;   // Knuth 승수, 0 방지
        }
    }

};


// path / flowfield 소유 단위 - 여러 leaf여도 외부에는 하나로 보이게 캡슐화
struct LeafGroup
{
    int                             leafId = -1;
    int                             parentId = -1;
    // 공유 포인터로 worker에서 새 필드로 통째로 교체 -> 여러 leaf가 같은 필드 가질수 있도록
    std::shared_ptr<const CorridorFlowField> field = EmptyField();      // leaf 소유 flowfield
    std::vector<int>                members;    // Npc index
    //std::unordered_set<int64_t>     excluded;   // 분리시, 막은(벽) 병목 복셀들(head는 항상 empty)
    int                             depth = 0;  // leaf depth (head==0)
    std::vector<uint32_t>            path;       // 이 leaf의 A* 경로(분리시, 겹칩 판별)
    bool                            active = false;

    // 공용 빈 필드 - leaf들마다 따로 안만들려고
    static const std::shared_ptr<const CorridorFlowField>& EmptyField()
    {
        static const std::shared_ptr<const CorridorFlowField> s_Empty
            = std::make_shared<const CorridorFlowField>();
        return s_Empty;
    }

    void Reset()
    {
        members.clear();
        //excluded.clear();
        path.clear();
        field = EmptyField();
        depth = 0;
        active = false;
    }
};

// 사용자 인식 기준 단위 - 선택/dest/도착 단위
struct NpcGroup
{
    int                         groupId = 0;
    DirectX::XMINT3             goal{ 0,0,0 };
    std::vector<int>            leafIds;
    bool                        hasGoal = false;
    bool                        isChasing = false;

    void Reset()
    {
        leafIds.clear();
        hasGoal = false;
    }
};


// NPC 깨우는 Trigger - 플레이어 근접
struct Stimulus
{
    DirectX::XMINT3 cell{ -1, -1, -1 };
    int     radiusCells = 0;
    uint8_t targetState = NPC_STATE_ALERTED;    // trigger 유발 state
   
    uint8_t requiredFlag = NSF_AGRO_TARGET;     // 패닉 플래그
};

// 어그로 전파 범위 및 해제 판정 단위
// 여러 behaviorGroup의 chase 멤버가 하나의 leaf 공유
struct BehaviorGroup
{
    std::vector<int> members;


    // 그룹 공용 거점 - 배회 중심이자 복귀 목표
    DirectX::XMINT3 anchorCell{ -1,-1,-1 };

    // 배회 반경 - 멤버 수는 스폰 후 안 바뀌므로 Init에서 한 번 계산해 캐시
    int wanderRadius = 3;

    // broadpays용
    DirectX::XMINT3 aabbMin{ 0,0,0 };
    DirectX::XMINT3 aabbMax{ 0,0,0 };

    // init에서 배정하는 npc region 인덱스
    int regionIndex = -1;

    bool HasAnyChasing(const NpcMoveData& move) const
    {
        for (int i : members)
        {
            if (move.state[i] == NPC_STATE_CHASE || move.state[i] == NPC_STATE_ALERTED)
                return true;
        }
        return false;
    }

    // 진정 안된 그룹 - 휴면 시키면 안된다
    bool HasAnyAwake(const NpcMoveData& move) const
    {
        for (int i : members)
        {
            const uint8_t s = move.state[i];
            if(s == NPC_STATE_CHASE || s == NPC_STATE_ALERTED || s == NPC_STATE_LOST)
                return true;
        }
        return false;
    }
};
