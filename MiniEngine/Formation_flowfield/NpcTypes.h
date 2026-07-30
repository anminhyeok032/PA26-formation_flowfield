#pragma once
#include "CorridorFlowField.h"
#include <DirectXMath.h>
#include <unordered_set>
#include <vector>

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

    std::vector<int> claimedSlot;                       // -1 = 미청구
    std::vector<DirectX::XMINT3> lastDir;               // 직전 이동 방향 (dx,dy,dz)

    std::vector<float> blockedTime;
    std::vector<float> congestionTime;                  // 분리 트리거용 (길음)
    std::vector<int> leafId;                            // 해당 NPC가 속한 leaf 인덱스(-1 = 무소속)

    std::vector<uint8_t> stopReason;                    // 0=정상도착, 1=필드밖(무효화)

    size_t size() const { return position.size(); }     // 길막당한 프레임 체크용

    void Resize(size_t n)
    {
        position.resize(n);
        targetWorldPos.resize(n);
        currCell.resize(n);
        targetCell.resize(n);
        active.resize(n);
        halfHeight.resize(n);

        claimedSlot.assign(n, -1);
        lastDir.resize(n, { 0,0,0 });

        blockedTime.assign(n, 0.0f);
        congestionTime.assign(n, 0.0f);

        leafId.resize(n, -1);
        stopReason.resize(n, 0);
    }

};


// path / flowfield 소유 단위 - 여러 leaf여도 외부에는 하나로 보이게 캡슐화
struct LeafGroup
{
    int                             leafId = -1;
    int                             parentId = -1;
    CorridorFlowField               field;      // leaf 소유 flowfield
    std::vector<int>                members;    // Npc index
    std::unordered_set<int64_t>     excluded;   // 분리시, 막은(벽) 병목 복셀들(head는 항상 empty)
    int                             depth = 0;  // leaf depth (head==0)
    std::vector<int64_t>            path;       // 이 leaf의 A* 경로(분리시, 겹칩 판별)
    bool                            active = false;

    void Reset()
    {
        members.clear();
        excluded.clear();
        path.clear();
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

    void Reset()
    {
        leafIds.clear();
        hasGoal = false;
    }
};
