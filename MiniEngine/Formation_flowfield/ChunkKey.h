#pragma once
#include <cstdint>

// 코드 시인성을 위해 청크키와 복셀 키 나눠놓음

// 8바이트 비트 마스킹 해서 청크를 hash key로 만드는 helper함수
static int64_t MakeChunkKey(int cx, int cy, int cz)
{
    // 만약 좌표가 2,097,151넘으면 21비트보다 크게 잡을것
    return (int64_t)(cx & 0x1FFFFF) |
        ((int64_t)(cy & 0x1FFFFF) << 21) |
        ((int64_t)(cz & 0x1FFFFF) << 42);
}

// 8바이트 비트 마스킹 해서 복셀을 hash key로 만드는 helper함수
static int64_t MakeCellKey(int x, int y, int z)
{
    // 만약 좌표가 2,097,151넘으면 21비트보다 크게 잡을것
    return (int64_t)(x & 0x1FFFFF) |
        ((int64_t)(y & 0x1FFFFF) << 21) |
        ((int64_t)(z & 0x1FFFFF) << 42);
}

// MakeChunkKey의 역변환. 해시맵을 순회하며 "이 청크가 어느 좌표였는지" 되짚을 때 사용.
inline void DecodeChunkKey(int64_t key, int& outCx, int& outCy, int& outCz)
{
    outCx = (int)(key & 0x1FFFFF);
    outCy = (int)((key >> 21) & 0x1FFFFF);
    outCz = (int)((key >> 42) & 0x1FFFFF);
}
