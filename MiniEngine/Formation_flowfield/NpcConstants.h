#pragma once
#include <cstdint>
#include <cmath>

// ------ NPC 설정 -------
constexpr float NPC_SPEED       = 3.0f;     // NPC 이동 속도(셀/초)
constexpr float NPC_INER        = 0.4f;     // 다른 방향 찾아갈때, 기존 방향으로 가게 하는 보정값
constexpr int   TARGET_COUNT    = 1'0000;
constexpr float NPC_WAIT_RATIO  = 0.05f;    // 1칸 움직이는데 비율 (x / 1.0f)%


// ------ Flowfield mask 설정값 ------
constexpr float CHASE_MASK_GROWTH_LIMIT = 1.1f; // 증가한 마스크가 최초 대비 해당 배수 넘길시, 메모리 위해 전체 재구성 한번 하기
constexpr int   CHASE_GOAL_HOPS         = 6;    // 목적지 주변 확보 범위


// ------ NPC STATE ------
// 상태를 추가할 때 고쳐야 할 곳은 아래 표 하나다.
// 조건식을 코드 곳곳에 열거하면(예: state != IDLE && state != LOST)
// 새 상태가 어느 술어에 속하는지 매번 재판단해야 하고, 빠뜨려도 컴파일이 통과한다
constexpr uint8_t   NPC_STATE_IDLE      = 0;        // 배회 / 필드 사용 x
constexpr uint8_t   NPC_STATE_ALERTED   = 1;        // 어그로 판정, 추격 시작 전 / 필드 사용 x
constexpr uint8_t   NPC_STATE_CHASE     = 2;        // 추격 / 필드 사용 o
constexpr uint8_t   NPC_STATE_LOST      = 3;        // anchorCell 복귀 / 필드 사용 x
constexpr int       NPC_STATE_COUNT     = 4;

// 상태가 어느 시스템에 속하는지를 나타내는 술어
// 각 축의 경계가 서로 다르기 때문에(감지는 IDLE|LOST, 앵커는 ALERTED|CHASE)
// state 값 하나로는 표현이 안 되고, 조건식으로 흩어두면 유지가 안 된다
enum NpcStateFlags : uint8_t
{
    NSF_NONE            = 0,
    NSF_AGRO_TARGET     = 1 << 0,   // 자극(Stimulus)에 반응하는가      - UpdateAgro
    NSF_MASK_ANCHOR     = 1 << 1,   // 회랑 마스크를 잡아당기는가        - MakeRequest / RebuildChaseLeaf / UpdateDeagro
    NSF_FIELD_USER      = 1 << 2,   // FlowField를 구독해 이동하는가     - AdvanceCell
    NSF_PROPAGATABLE    = 1 << 3,   // 그룹 내 전파로 감염되는가         - PropagateAgro
};

// 상태 <-> 속성 매핑 테이블
constexpr uint8_t NPC_STATE_FLAGS[NPC_STATE_COUNT] =
{
    /* IDLE    */ NSF_AGRO_TARGET | NSF_PROPAGATABLE,
    /* ALERTED */ NSF_MASK_ANCHOR,                      // 곧 추격하므로 마스크를 미리 확보
    /* CHASE   */ NSF_MASK_ANCHOR | NSF_FIELD_USER,
    /* LOST    */ NSF_AGRO_TARGET | NSF_PROPAGATABLE,   // LOST에서도 전파
};

inline bool StateHas(uint8_t state, uint8_t flag)
{
    return (NPC_STATE_FLAGS[state] & flag) != 0;
}

// ------ NPC AGRO ------
constexpr int		AGRO_RADIUS_CELLS	= 25;		// sphere 반경 = 마스크 크기
constexpr int		DEAGRO_RADIUS_CELLS	= 50;		// 감지의 2배 - 경계 진동 방지


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
constexpr float     LOST_STUCK_GIVEUP_SEC   = 5.0f;     // 복귀중 이 이상 막히면 거점으로 삼아버림


// ------ 그룹 크기 분포 ------
constexpr int		GROUP_SIZE_BUCKETS[4] = { 1, 4, 20, 120 };
constexpr int		GROUP_SIZE_WEIGHTS[4] = { 40, 35, 20, 5 };	// 합계 100
constexpr int		SPAWN_SEED_ATTEMPTS = 4000;		// 시드 후보 시도 상한
constexpr int		SPAWN_MIN_SEED_DIST = 24;		// 시드 간 최소 간격(복셀 단위)
constexpr int       SPAWN_PLAYER_EXCLUSION_CELLS = AGRO_RADIUS_CELLS * 2;   // 초기화 플레이어 주변 반경 비우기용
 
// ----- 그룹 거점(anchor) -----
// 인원수 비례 거점 반경
inline int GroupWanderRadius(int memberCount)
{
    const int r = (int)std::ceil(std::sqrt((float)memberCount)) + 2;
    return (r < 3) ? 3 : r;
}
constexpr int ANCHOR_DIST_CELLS = 100;  // 그룹이 거점에서 100 이상 멀어지면 새 거점으로 지정



// ------ 공간 분할 ------
// Region은 Agro Radius보다 충분히 커야됨 - 청크사이즈의 배수여야 함
constexpr int REGION_SIZE_CELLS = 64;
// 잠든 그룹 깨우는 범위
// WAKEUP_RADIUS - AGRO_RADIUS = 100 - 25 = 75
constexpr int WAKEUP_RADIUS_CELLS = AGRO_RADIUS_CELLS * 4;
// 활성 리전 반경. 플레이어가 리전 어디에 있든 이 거리까지는 활성이 보장된다
constexpr int ACTIVE_REGION_RINGS = (WAKEUP_RADIUS_CELLS + REGION_SIZE_CELLS - 1) / REGION_SIZE_CELLS;




// xorshift32 - 개체별 결정론적 난수. std::rand는 스레드 안전하지 않고 전역 상태를 공유한다
static inline uint32_t NextRand(uint32_t& s)
{
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}
