#pragma once
#include "VoxelGrid.h"
#include <DirectXMath.h>
#include <vector>

struct NeighborInfo
{
	DirectX::XMINT3 pos;
	float cost;
};

// 8 방향 높이차 포함
// A*와 FlowField(BFS) 둘 다 반드시 이 함수 하나만 사용해야 함 — 각자 구현하면
// 미묘하게 달라져서 두 시스템이 서로 다른 그래프를 가정하게 될 위험이 있음.
void GetWalkableNeighbors(const VoxelGrid& grid, const DirectX::XMINT3& cur, std::vector<NeighborInfo>& outNeighbors);


// (xyz)가 해당 xz의 몇층인지 반환 - 표면 없으면 -1
int FindSurfaceSlot(const VoxelGrid& grid, int x, int y, int z);