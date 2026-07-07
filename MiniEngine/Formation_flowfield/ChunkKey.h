#pragma once
#include <cstdint>

static int64_t MakeChunkKey(int cx, int cy, int cz)
{
    return (int64_t)(cx & 0xFFFFF) |
        ((int64_t)(cy & 0xFFFFF) << 20) |
        ((int64_t)(cz & 0xFFFFF) << 40);
}


// MakeChunkKey의 역변환. 해시맵을 순회하며 "이 청크가 어느 좌표였는지" 되짚을 때 사용.
inline void DecodeChunkKey(int64_t key, int& outCx, int& outCy, int& outCz)
{
    outCx = (int)(key & 0xFFFFF);
    outCy = (int)((key >> 20) & 0xFFFFF);
    outCz = (int)((key >> 40) & 0xFFFFF);
}