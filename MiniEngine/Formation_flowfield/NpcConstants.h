#pragma once

// NPC 이동 속도(셀/초)
constexpr float NPC_SPEED = 3.0f;

// 다른 방향 찾아갈때, 기존 방향으로 가게 하는 보정값
constexpr float NPC_INER = 0.4f;                

const int TARGET_COUNT = 1000;

// 1칸 움직이는데 비율 (x / 1.0f)%
constexpr float NPC_WAIT_RATIO = 0.1f;   

// 증가한 마스크가 최초 대비 해당 배수 넘길시, 메모리 위해 전체 재구성 한번 하기
constexpr float CHASE_MASK_GROWTH_LIMIT = 1.1f;


// 목적지 주변 확보 범위
constexpr int CHASE_GOAL_HOPS = 2;
