#pragma once
#include "CorridorFlowField.h"
#include <DirectXMath.h>
#include <vector>	
#include <memory>


enum class FieldBuildMode
{
	// 최초 목적지 - A* + 멤버 시드 / 캐시를 새로 만든다
	FullRebuild,

	// 추격 갱신 - goal 노드에서 홉 확장분만 캐시에 추가 (A* x)
	ChaseIncremental,
};


// 워커 스레드 전용 마스크 캐시
// 워커 스레드 전용 IO
struct FieldMaskCache
{
	std::unordered_set<uint32_t> nodes;    // 직전 노드 집합 - 신규 노드 차집합용
	std::unordered_set<int64_t>  mask;     // 직전 셀 마스크 - 신규 셀들 누적

	uint64_t generation = 0;               // 이 캐시를 만든 시점의 지형 상태
	int      hops = 0;                     // FullRebuild가 쓴 홉 수 - 신규도 같은 폭을 유지
	size_t   baseSize = 0;                 // 최초 마스크 셀 수 - 폭주 판정 기준선
	int      leafId = -1;                  // 다른 leaf 요청이 오면 캐시 무효

	bool IsEmpty() const { return mask.empty(); }
	void Clear() { *this = FieldMaskCache{}; }
};


// 워커 스레드에 넘기는 작업 모음
struct FieldBuildRequest
{
	FieldBuildMode mode = FieldBuildMode::FullRebuild;

	int		leafId = -1;
	DirectX::XMINT3 goalCell{ -1, -1, -1 };

	// A* 출발점 - 무게중심
	DirectX::XMINT3 startCell{ -1,-1,-1 };

	// 마스크 시드용 - 유효한 (x >= 0) 멤버 시작 셀만 담기
	std::vector<DirectX::XMINT3> memberCells;

	// margin 계산 입력
	int		memberCount = 0;
	int		maxDistFromCentroid = 0;

	// 해당 요청 지형 상태
	uint64_t	generation = 0;

	// 기본 생성 상태 = 요청 없음
	bool IsValid() const 
	{ 
		if (leafId < 0 || goalCell.x < 0)	return false;
		if (mode == FieldBuildMode::ChaseIncremental) return true;
		return startCell.x >= 0;
	}

	void Clear() { *this = FieldBuildRequest{}; }

};


// worker result
struct FieldBuildResult
{
	int		leafId = -1;
	uint64_t	generation = 0;
	bool	success = false;

	std::shared_ptr<const CorridorFlowField> field;
	std::vector<uint32_t>		nodePath;	// path 갱신용

	void Clear() { *this = FieldBuildResult{}; }
};
