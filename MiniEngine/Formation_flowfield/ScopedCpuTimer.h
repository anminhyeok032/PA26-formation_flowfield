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

#define CORE_CONCAT_INNER(a, b) a##b
#define CORE_CONCAT(a, b) CORE_CONCAT_INNER(a, b)
#define CORE_SCOPE(s) CoreTimer CORE_CONCAT(_t_, __LINE__)(CoreScope::s)