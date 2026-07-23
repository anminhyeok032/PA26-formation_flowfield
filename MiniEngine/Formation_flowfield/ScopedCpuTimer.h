#pragma once
#include "SystemTime.h"
#include "Utility.h"
#include <unordered_map>

// Release 빌드에서 CPU 구간 시간을 재는 RAII 타이머.
// 누적/호출횟수를 정적으로 모아두고, Report()로 한 번에 출력.
class ScopedCpuTimer
{
private:
    const char* m_Name;
    int64_t     m_Start;

public:
    struct Stat { double totalMs = 0.0; uint64_t calls = 0; double maxMs = 0.0; };

    explicit ScopedCpuTimer(const char* name)
        : m_Name(name), m_Start(SystemTime::GetCurrentTick()) {}

    ~ScopedCpuTimer()
    {
        double ms = SystemTime::TimeBetweenTicks(m_Start, SystemTime::GetCurrentTick()) * 1000.0;
        Stat& s = Stats()[m_Name];
        s.totalMs += ms;
        s.calls++;
        if (ms > s.maxMs) s.maxMs = ms;
    }

    static std::unordered_map<std::string, Stat>& Stats()
    {
        static std::unordered_map<std::string, Stat> s_Stats;
        return s_Stats;
    }

    //static void Report()
    //{
    //    Utility::Printf("---- CPU Timing ----\n");
    //    for (const auto& kv : Stats())
    //    {
    //        Utility::Printf("%-40s total %8.2fms  calls %6llu  avg %7.4fms  max %7.4fms\n",
    //            kv.first.c_str(), kv.second.totalMs, kv.second.calls,
    //            kv.second.totalMs / (double)kv.second.calls, kv.second.maxMs);
    //    }
    //}
    static void Reset() { Stats().clear(); }

    static void Report(const char* filePath = "perf_report.txt")
    {
        // 정렬: 총 시간 내림차순 (병목부터 보이게)
        std::vector<std::pair<std::string, Stat>> sorted(Stats().begin(), Stats().end());
        std::sort(sorted.begin(), sorted.end(),
            [](const auto& a, const auto& b) { return a.second.totalMs > b.second.totalMs; });

        FILE* fp = nullptr;
        fopen_s(&fp, filePath, "w");

        auto emit = [&](const char* line)
        {
            Utility::Print(line);          // 디버거 붙어 있으면 출력 창에도
            if (fp) fputs(line, fp);       // 항상 파일에
        };

        char buf[256];
        sprintf_s(buf, "%-40s %10s %8s %10s %10s\n", "scope", "total(ms)", "calls", "avg(ms)", "max(ms)");
        emit(buf);

        for (const auto& kv : sorted)
        {
            sprintf_s(buf, "%-40s %10.2f %8llu %10.4f %10.4f\n",
                kv.first.c_str(), kv.second.totalMs, kv.second.calls,
                kv.second.totalMs / (double)kv.second.calls, kv.second.maxMs);
            emit(buf);
        }

        if (fp) fclose(fp);
    }
};

#define CPU_SCOPE(name) ScopedCpuTimer _cpuTimer_##__LINE__(name)
