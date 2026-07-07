#pragma once
#include <DirectXMath.h>
#include <vector>
#include <unordered_set>


// 그룹 인원수를 기준으로 필요한 margin(청크 단위)을 계산.
// 대형은 정사각형(ceil(sqrt(memberCount)) x 같은 값)이라고 가정하고,
// 그 절반 폭 + 여유(bufferCells)가 몇 개의 청크에 걸치는지로 결정.
int ComputeMarginChunks(int memberCount, int chunkSize, int bufferCells = 2);

// A* 경로가 지나가는 청크들 + 수평(XZ) 방향 여유(margin)를 포함한 청크 마스크 생성.
// 이 마스크는 CorridorFlowField가 "이 청크까지만 계산하고 그 밖은 확장하지 않는다"는 경계로 사용됨.
std::unordered_set<int64_t> BuildChunkMask(const std::vector<DirectX::XMINT3>& path, int marginChunks);
