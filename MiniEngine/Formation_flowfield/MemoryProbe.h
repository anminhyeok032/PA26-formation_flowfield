#pragma once
#include <Windows.h>
#include <Psapi.h>
#include "Utility.h"
#pragma comment(lib, "Psapi.lib")

class MemoryProbe
{
public:
    static size_t GetPrivateBytes()
    {
        PROCESS_MEMORY_COUNTERS_EX pmc{};
        GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
        return pmc.PrivateUsage;
    }

    explicit MemoryProbe(const char* name) : m_Name(name), m_Before(GetPrivateBytes()) {}

    void Report()
    {
        size_t after = GetPrivateBytes();
        Utility::Printf("[MemProbe] %-30s before=%8.1fMB after=%8.1fMB delta=%+8.1fMB\n",
            m_Name, m_Before / 1e6, after / 1e6, ((int64_t)after - (int64_t)m_Before) / 1e6);
    }

private:
    const char* m_Name;
    size_t m_Before;
};
