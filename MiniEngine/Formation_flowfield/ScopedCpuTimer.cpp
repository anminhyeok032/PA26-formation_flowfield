#include "pch.h"              // MiniEngine 프로젝트 규약 (없으면 제거)
#include "ScopedCpuTimer.h"
#include "Utility.h"
#include "GraphicsCore.h"     // Graphics::GetFrameCount()
#include <algorithm>
#include <vector>
#include <cstdio>

namespace
{
    // CoreScope enum과 순서·개수가 정확히 일치해야 함.
    // enum에 항목을 추가하면 여기도 같은 위치에 추가할 것 (아래 static_assert가 잡아줌).
    const char* kScopeNames[] =
    {
        "NpcUpdateCore",
        "DestCollectStart",
        "DestAStar",
        "DestBuildMask",
        "DestFieldDijkstra",
        "DestFieldComputeDir",
        "DestInitMovement",
        "DestFieldBuild",
        "FieldDijkstra",
        "FieldComputeDir"
    };
    static_assert(_countof(kScopeNames) == (int)CoreScope::COUNT,
        "kScopeNames가 CoreScope enum과 개수가 다름 - 둘을 함께 갱신할 것");
}

void CoreTimer::Report(const char* filePath)
{
    Stat* stats = Stats();

    // 한 번도 호출되지 않은 스코프는 제외하고, total 내림차순 정렬(병목부터 보이게).
    // 인덱스만 정렬해 원본 배열은 건드리지 않음.
    std::vector<int> order;
    order.reserve((int)CoreScope::COUNT);
    for (int i = 0; i < (int)CoreScope::COUNT; ++i)
        if (stats[i].calls > 0) order.push_back(i);

    std::sort(order.begin(), order.end(),
        [stats](int a, int b) { return stats[a].totalMs > stats[b].totalMs; });

    // append 모드 - 여러 번 호출해도 이력이 쌓임(Reset과 조합해 구간별 비교 가능)
    FILE* fp = nullptr;
    fopen_s(&fp, filePath, "a");

    auto emit = [&](const char* line)
    {
        Utility::Print(line);          // 디버거 부착 시 VS 출력 창
        if (fp) fputs(line, fp);       // 항상 파일
    };

    char buf[256];   // Utility::Printf가 내부 버퍼 256B라, 한 줄씩 나눠 출력할 것

   

    sprintf_s(buf, "%-22s %12s %10s %12s %12s\n",
        "scope", "total(ms)", "calls", "avg(ms)", "max(ms)");
    emit(buf);

    for (int i : order)
    {
        const Stat& s = stats[i];
        sprintf_s(buf, "%-22s %12.3f %10llu %12.5f %12.5f\n",
            kScopeNames[i], s.totalMs, (unsigned long long)s.calls,
            s.totalMs / (double)s.calls, s.maxMs);
        emit(buf);
    }

    // NpcUpdateCore의 calls = 실제로 이동이 돌아간 프레임 수.
    // 목적지 파이프라인(Dest*)은 우클릭 횟수이므로 분모가 다르다는 점에 주의.
    const Stat& frameStat = stats[(int)CoreScope::NpcUpdateCore];
    if (frameStat.calls > 0)
    {
        sprintf_s(buf, "\n(active frames = %llu, NpcUpdateCore per-frame = %.5f ms)\n\n",
            (unsigned long long)frameStat.calls,
            frameStat.totalMs / (double)frameStat.calls);
        emit(buf);
    }
    else
    {
        emit("\n");
    }

    if (fp) fclose(fp);
}