#pragma once
#include <cstdint>

// ------ NPC STATE ------
constexpr uint8_t	NPC_STATE_IDLE		= 0;		// 배회 / 필드 사용x
constexpr uint8_t	NPC_STATE_ALERTED	= 1;		// 어그로 판정 / 추격 시작 전 // 필드 사용 x
constexpr uint8_t	NPC_STATE_CHASE		= 2;		// 추격 / 필드 사용 o
constexpr uint8_t	NPC_STATE_LOST		= 3;		// anchorCell 복귀	/ 필드 사용 x

// ------ NPC AGRO ------
constexpr int		AGRO_RADIUS_CELLS	= 25;		// sphere 반경 = 마스크 크기
constexpr int		DEAGRO_RADIUS_CELLS	= 40;		// 감지의 1.6배 - 경계 진동 방지
constexpr float		DEAGRO_HOLD_SEC		= 3.0f;		// 해제 지연 - 경계 지속 토글 방지

// ------ AGRO 전파 ------
constexpr float		PROPAGATION_DELAY_SEC		= 0.4f;	// Alerted -> Chase 지연 시간
constexpr float		PROPAGATION_JITTER			= 0.2f;	// 개체별 +- 비율
constexpr int		PROPAGATION_RADIUS_CELLS	= 8;	// 무제한이면 첫 개체가 그룹 전체를 한번에 켜서 파동이 사라짐
constexpr int		PROPAGATION_FANOUT			= 3;	// 1회당 최대 전파 개수
constexpr int		MAX_PROPAGATIONS_PER_FRAME	= 32;	// 전체 확산 안전 장치

// ------ 필드 요청 -------
constexpr float		FIELD_REQUEST_COOLDOWN_SEC = 0.2f;	// summit 쿨타임

// ----- 배회 / 복귀 ------
constexpr int		WANDER_RADIUS_CELLS		= 6;
constexpr float		WANDER_PAUSE_MIN_SEC	= 1.0f;
constexpr float		WANDER_PAUSE_MAX_SEC	= 3.0f;
constexpr float		LOST_TIMEOUT_SEC		= 15.0f;


// ------ 그룹 크기 분포 ------
constexpr int		GROUP_SIZE_BUCKETS[4] = { 1, 4, 20, 120 };
constexpr int		GROUP_SIZE_WEIGHTS[4] = { 40, 35, 20, 5 };	// 합계 100
constexpr int		SPAWN_SEED_ATTEMPTS = 4000;		// 시드 후보 시도 상한
constexpr int		SPAWN_MIN_SEED_DIST = 24;		// 시드 간 최소 간격(복셀 단위)
constexpr int       SPAWN_PLAYER_EXCLUSION_CELLS = AGRO_RADIUS_CELLS * 2;   // 초기화 플레이어 주변 반경 비우기용
 

// NPC 이동 속도(셀/초)
constexpr float NPC_SPEED = 10.0f;

// 다른 방향 찾아갈때, 기존 방향으로 가게 하는 보정값
constexpr float NPC_INER = 0.4f;                

const int TARGET_COUNT = 1000;

// 1칸 움직이는데 비율 (x / 1.0f)%
constexpr float NPC_WAIT_RATIO = 0.05f;   

// 증가한 마스크가 최초 대비 해당 배수 넘길시, 메모리 위해 전체 재구성 한번 하기
constexpr float CHASE_MASK_GROWTH_LIMIT = 1.1f;


// 목적지 주변 확보 범위
constexpr int CHASE_GOAL_HOPS = 6;


// xorshift32 - 개체별 결정론적 난수. std::rand는 스레드 안전하지 않고 전역 상태를 공유한다
static inline uint32_t NextRand(uint32_t& s)
{
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}
