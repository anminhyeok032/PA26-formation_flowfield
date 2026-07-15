#pragma once
#include <DirectXMath.h>
#include <vector>
#include <unordered_set>
#include <cstdint>

class VoxelGrid;

// 그룹 인원수 -> 필요한 margin(셀 단위, 걸음 수).
int ComputeMarginCells(int memberCount, int bufferCells = 2);


// A* 경로가 지나가는 path를 기반으로 bfs(margin까지만)해서 범위 return
// 이 마스크는 CorridorFlowField가 "이 청크까지만 계산하고 그 밖은 확장하지 않는다"는 경계로 사용됨.
std::unordered_set<int64_t> BuildLayerMask(const VoxelGrid& grid,
    const std::vector<DirectX::XMINT3>& path, 
    const std::vector<DirectX::XMINT3>& extraSeeds, 
    int marginCells);
