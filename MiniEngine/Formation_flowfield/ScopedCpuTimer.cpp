#include "pch.h"
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
        "DestBuildMask",
        "DestFieldDijkstra",
        "DestFieldComputeDir",
        "DestInitMovement",
        "DestFieldBuild",
        "FieldDijkstra",
        "FieldComputeDir",
        "CorridorField_Build",

        // ChunkGraph - 빌드 (지형 로드 시 1회)
        "ChunkGraph_Build",
        "ChunkGraph_LabelAll",
        "ChunkGraph_BuildEdges",

        // ChunkGraph - 갱신 (지형 편집 시)
        "ChunkGraph_Refresh",

        // ChunkGraph - 질의 (목적지 설정 / leaf 분리마다)
        "ChunkGraph_NodeIdOf",
        "ChunkGraph_FindPath",
        "ChunkGraph_ExpandNodes",
        "ChunkGraph_MaskCells",
    };
    static_assert(_countof(kScopeNames) == (int)CoreScope::COUNT,
        "kScopeNames가 CoreScope enum과 개수가 다름 - 둘을 함께 갱신할 것");

    // 다른 스코프를 안에 포함하는(= 시간이 중첩되는) 스코프.
    // total을 단순 합산하면 이중 계산이 되므로 표에 '*'로 표시한다.
    // 기존 스코프의 값은 실제 CORE_SCOPE 삽입 위치에 맞춰 확인 후 조정할 것.
    const bool kIsAggregate[] =
    {
        true,    // NpcUpdateCore        - 프레임 이동 로직 전체
        false,   // DestCollectStart
        false,   // DestBuildMask
        false,   // DestFieldDijkstra
        false,   // DestFieldComputeDir
        false,   // DestInitMovement
        true,    // DestFieldBuild       - Dijkstra + ComputeDir 포함
        false,   // FieldDijkstra
        false,   // FieldComputeDir
        true,    // CorridorField_Build  - Dijkstra + ComputeDir 포함

        true,    // ChunkGraph_Build     - LabelAll + BuildEdges 포함
        false,   // ChunkGraph_LabelAll
        false,   // ChunkGraph_BuildEdges

        true,    // ChunkGraph_Refresh   - 성분 개수가 바뀌면 내부에서 Build로 폴백
        false,   // ChunkGraph_NodeIdOf
        false,   // ChunkGraph_FindPath  - 내부에서 NodeIdOf 2회 호출하지만 별도 집계
        false,   // ChunkGraph_ExpandNodes
        false,   // ChunkGraph_MaskCells
    };
    static_assert(_countof(kIsAggregate) == (int)CoreScope::COUNT,
        "kIsAggregate가 CoreScope enum과 개수가 다름");
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
    fopen_s(&fp, filePath, "ab");

    auto emit = [&](const char* line)
    {
        Utility::Print(line);          // 디버거 부착 시 VS 출력 창
        if (fp) fputs(line, fp);       // 항상 파일
    };

    char buf[256];   // Utility::Printf가 내부 버퍼 256B라, 한 줄씩 나눠 출력할 것

    sprintf_s(buf, "%-26s %12s %10s %12s %12s\n",
        "scope", "total(ms)", "calls", "avg(ms)", "max(ms)");
    emit(buf);

    bool anyAggregate = false;
    for (int i : order)
    {
        const Stat& s = stats[i];

        // 이름이 %-26s 폭을 넘지 않도록 마커까지 합쳐 한 번에 만든다
        char name[32];
        sprintf_s(name, "%s%s", kScopeNames[i], kIsAggregate[i] ? "*" : "");
        anyAggregate |= kIsAggregate[i];

        sprintf_s(buf, "%-26s %12.3f %10llu %12.5f %12.5f\n",
            name, s.totalMs, (unsigned long long)s.calls,
            s.totalMs / (double)s.calls, s.maxMs);
        emit(buf);
    }

    if (anyAggregate)
    {
        emit("\n* = include lower-scope. dont added total - double counted.\n");
    }

    // NpcUpdateCore의 calls = 실제로 이동이 돌아간 프레임 수.
    // 목적지 파이프라인(Dest*)은 우클릭 횟수이므로 분모가 다르다는 점에 주의.
    const Stat& frameStat = stats[(int)CoreScope::NpcUpdateCore];
    if (frameStat.calls > 0)
    {
        sprintf_s(buf, "\n(active frames = %llu, NpcUpdateCore per-frame = %.5f ms)\n",
            (unsigned long long)frameStat.calls,
            frameStat.totalMs / (double)frameStat.calls);
        emit(buf);
    }

    // ChunkGraph 요약 - 계층형 구조가 실제로 이득인지 판단하는 핵심 지표
    const Stat& cgBuild = stats[(int)CoreScope::ChunkGraph_Build];
    const Stat& cgRefresh = stats[(int)CoreScope::ChunkGraph_Refresh];
    const Stat& cgFind = stats[(int)CoreScope::ChunkGraph_FindPath];

    if (cgBuild.calls > 0 || cgFind.calls > 0)
    {
        emit("\n--- ChunkGraph ---\n");

        if (cgBuild.calls > 0)
        {
            // calls > 1 이면 지형 편집 중 전체 재빌드 폴백이 발생했다는 뜻.
            // 시뮬레이션에서 편집당 약 7%로 나왔던 항목 - 실측치가 이걸 넘으면
            // 노드 id 슬랙 할당으로 전환을 검토할 것.
            sprintf_s(buf, "full builds = %llu (1 = first loaded, 2+ = Refresh fallback occ)\n",
                (unsigned long long)cgBuild.calls);
            emit(buf);
        }

        if (cgRefresh.calls > 0)
        {
            // Refresh는 폴백 시 Build를 포함하므로, 순수 증분 비용은 Build 몫을 빼야 한다.
            // 최초 로드 Build 1회는 Refresh 밖에서 일어나므로 그만큼은 빼면 안 됨.
            const double fallbackMs = (cgBuild.calls > 1)
                ? cgBuild.totalMs * (double)(cgBuild.calls - 1) / (double)cgBuild.calls
                : 0.0;
            const double pureIncremental = cgRefresh.totalMs - fallbackMs;

            sprintf_s(buf, "refresh: %lluTimes, total %.3f ms (fallback %.3f ms, pure %.3f ms)\n",
                (unsigned long long)cgRefresh.calls, cgRefresh.totalMs,
                fallbackMs, pureIncremental);
            emit(buf);
        }

        if (cgFind.calls > 0)
        {
            // 질의당 비용. 셀 단위 A*와 비교하려면 USE_CHUNK_GRAPH를 끄고 재측정할 것.
            sprintf_s(buf, "path queries = %llu, per %.5f ms (max %.5f ms)\n",
                (unsigned long long)cgFind.calls,
                cgFind.totalMs / (double)cgFind.calls, cgFind.maxMs);
            emit(buf);
        }
    }

    emit("\n");
    if (fp) fclose(fp);
}