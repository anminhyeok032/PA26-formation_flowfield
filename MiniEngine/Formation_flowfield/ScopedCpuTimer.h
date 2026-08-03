#pragma once
#include "SystemTime.h"

enum class CoreScope : int
{
    NpcUpdateCore = 0,
    DestCollectStart,
    DestBuildMask,
    DestFieldDijkstra,
    DestFieldComputeDir,
    DestInitMovement,
    DestFieldBuild,
    FieldDijkstra,
    FieldComputeDir,
    CorridorField_Build,

    // --- ChunkGraph: 빌드 단계 (지형 로드 시 1회) ---
    ChunkGraph_Build,          // 전체 (아래 둘을 포함)
    ChunkGraph_LabelAll,       //   ㄴ 전 청크 flood fill
    ChunkGraph_BuildEdges,     //   ㄴ 전 청크 간선 생성

    // --- ChunkGraph: 갱신 (지형 편집 시) ---
    ChunkGraph_Refresh,        // 증분 갱신 전체. 내부에서 Build로 폴백될 수 있음

    // --- ChunkGraph: 질의 (목적지 설정 / leaf 분리마다) ---
    ChunkGraph_NodeIdOf,       // 셀->노드 변환 (질의당 2회, 각각 청크 1개 라벨링)
    ChunkGraph_FindPath,       // 성분 그래프 A*  (기존 DestAStar 대체)
    ChunkGraph_ExpandNodes,    // margin 확장 BFS
    ChunkGraph_MaskCells,      // 노드 -> 셀 마스크 전개

    //MaskCells,        // 마스크에 들어간 셀 수
    //FieldChunks,      // FlowField가 할당한 청크 수
    //FieldReached,     // Dijkstra가 도달한 셀 수 (마스크 대비 실제 유효율)
    //ExpandedNodes,    // ExpandNodes가 반환한 노드 수

    COUNT
};


class CoreTimer
{
public:
    struct Stat { double totalMs = 0; uint64_t calls = 0; double maxMs = 0; };

    explicit CoreTimer(CoreScope s)
        : m_Scope((int)s), m_Start(SystemTime::GetCurrentTick()) {}

    ~CoreTimer()
    {
        double ms = SystemTime::TimeBetweenTicks(m_Start, SystemTime::GetCurrentTick()) * 1000.0;
        Stat& st = Stats()[m_Scope];        // 배열 인덱싱 - 해싱 없음
        st.totalMs += ms;
        st.calls++;
        if (ms > st.maxMs) st.maxMs = ms;
    }

    static Stat* Stats() { static Stat s[(int)CoreScope::COUNT]; return s; }
    static void Report(const char* filePath = "Report/core_timing.txt");
    static void Reset() { for (int i = 0; i < (int)CoreScope::COUNT; ++i) Stats()[i] = Stat{}; }

private:
    int     m_Scope;
    int64_t m_Start;
};

enum class CoreCount : int
{
    ExpandedNodes,      // ExpandNodes가 반환한 노드 수
    MaskCells,          // 마스크에 들어간 셀 수
    FieldChunks,        // FlowField가 할당한 청크 수
    FieldChunkBytes,    // sizeof(FlowFieldChunk) - private nested라 내부에서만 알 수 있음
    FieldReached,       // Dijkstra가 실제 도달한 셀 수
    COUNT
};

class CoreCounter
{
public:
    // 누적이 아니라 덮어쓰기.
    // 홉 스윕은 1회성 실험이라 누적하면 직전 실행값이 묻힌다
    static void Set(CoreCount c, uint64_t n) { Values()[(int)c] = n; Valid()[(int)c] = true; }

    static uint64_t Get(CoreCount c) { return Values()[(int)c]; }

    // 0과 "측정 안 함"은 다르다 - 마스크가 0셀인 건 버그 신호이므로 반드시 보여야 한다
    static uint64_t* Values() { static uint64_t v[(int)CoreCount::COUNT]{}; return v; }
    static bool* Valid() { static bool     b[(int)CoreCount::COUNT]{}; return b; }

    // label은 스윕 구간 식별용 ("hop=8" 등). 없으면 nullptr
    static void Report(const char* label = nullptr,
        const char* filePath = "Report/core_timing.txt");

    static void Reset()
    {
        for (int i = 0; i < (int)CoreCount::COUNT; ++i) { Values()[i] = 0; Valid()[i] = false; }
    }
};

#define CORE_CONCAT_INNER(a, b) a##b
#define CORE_CONCAT(a, b) CORE_CONCAT_INNER(a, b)
#define CORE_SCOPE(s) CoreTimer CORE_CONCAT(_t_, __LINE__)(CoreScope::s)