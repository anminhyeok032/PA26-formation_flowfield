#pragma once
#include "VoxelGrid.h"
#include <DirectXMath.h>
#include <vector>

// 4방향(대각선 없음) 이웃 탐색. 터널 등 다층 표면(GetSurfaceYList)을 고려해서
// 현재 y와 높이차 1칸 이내인 표면으로만 연결을 허용.
// A*와 FlowField(BFS) 둘 다 반드시 이 함수 하나만 사용해야 함 — 각자 구현하면
// 미묘하게 달라져서 두 시스템이 서로 다른 그래프를 가정하게 될 위험이 있음.
void GetWalkableNeighbors(const VoxelGrid& grid, const DirectX::XMINT3& cur, std::vector<DirectX::XMINT3>& outNeighbors);
