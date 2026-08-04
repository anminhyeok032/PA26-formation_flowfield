#pragma once

// NPC 이동 속도(셀/초)
constexpr float NPC_SPEED = 3.0f;

// 다른 방향 찾아갈때, 기존 방향으로 가게 하는 보정값
constexpr float NPC_INER = 0.4f;                

const int TARGET_COUNT = 1000;

// 1칸 움직이는데 비율 (x / 1.0f)%
constexpr float NPC_WAIT_RATIO = 0.5f;   
