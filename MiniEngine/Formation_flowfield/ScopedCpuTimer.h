
#pragma once
#include "SystemTime.h"

enum class CoreScope : int
{
    NpcUpdateCore = 0,   // Sync/Upload 제외한 순수 이동 로직
    DestCollectStart,
    DestAStar,
    DestBuildMask,
    DestFieldDijkstra,
    DestFieldComputeDir,
    DestInitMovement,
    DestFieldBuild,
    FieldDijkstra,
    FieldComputeDir,
    CorridorField_Build,
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

#define CORE_SCOPE(s) CoreTimer _t_##__LINE__(CoreScope::s)
