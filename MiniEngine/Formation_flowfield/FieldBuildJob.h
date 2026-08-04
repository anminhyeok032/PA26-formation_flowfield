#pragma once
#include "CorridorFlowField.h"
#include <DirectXMath.h>
#include <vector>	
#include <memory>

// 워커 스레드에 넘기는 작업 모음
struct FieldBuildRequest
{
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
	bool IsValid() const { return leafId >= 0 && startCell.x >= 0; }

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
