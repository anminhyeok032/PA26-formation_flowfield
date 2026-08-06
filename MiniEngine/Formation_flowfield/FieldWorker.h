#pragma once
#include "FieldBuildJob.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <mutex>


class VoxelGrid;
class ChunkGraph;

// Flowfield 계산 전담 worker
class FieldWorker
{
public:
	void Start(const VoxelGrid& grid, const ChunkGraph& chunkGraph);
	void Stop();
	~FieldWorker() { Stop(); }

	// 요청 제출 - 이전 요청 진행중이면 덮어쓴다 -> queuing 하면 계속 뒤쳐져버리니까
	void Submit(FieldBuildRequest&& req);

	// 완성 결과 수령 - 없으면 null
	// 프레임 시작 시점에만 호출해야함 -> 중간 호출시, 값 깨짐
	FieldBuildResult TryAcquire();


	// 지형 / 그래프 수정시, 필수 호출할 것
	void CancelAndWait();

	bool IsBusy() const { return m_Busy.load(std::memory_order_acquire); }

private:
	void Worker();

	// Worker용 캐쉬
	FieldMaskCache	m_MaskCache;
	
	const VoxelGrid* m_Grid = nullptr;
	const ChunkGraph* m_ChunkGraph = nullptr;

	std::thread		m_Thread;
	mutable std::mutex		m_Mutex;	// mutable 사용시, const 함수 내부에서 변경 가능
	std::condition_variable	m_cvWork;	// 메인 -> worker (요청 도착 / 종료)
	std::condition_variable m_cvIdle;	// worker -> 메인 (작업 종료)


	FieldBuildRequest	m_Pending;	// false면 대기 없음
	FieldBuildResult	m_Ready;	// success == false 면 수령 대기 x

	std::atomic<bool> m_Cancel{ false };
	std::atomic<bool> m_Busy{ false };
	bool				m_Quit = false;

};
