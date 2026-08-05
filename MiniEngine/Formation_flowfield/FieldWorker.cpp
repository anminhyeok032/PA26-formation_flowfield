#include "FieldWorker.h"
#include "LeafSplitController.h"



void FieldWorker::Worker()
{
	while (true)
	{
		FieldBuildRequest req;
		{
			std::unique_lock<std::mutex> lk(m_Mutex);

			// 요청 && 대기 x -> m_Ready 없을때
			m_cvWork.wait(lk, [this] 
				{
					return m_Quit || (m_Pending.IsValid() && !m_Ready.success); 
				});
			if (true == m_Quit)	return;

			req = std::move(m_Pending);
			m_Pending.Clear();
			m_Busy.store(true, std::memory_order_release);
		}

		FieldBuildResult res = LeafSplitController::RunBuild(
			*m_Grid, *m_ChunkGraph, req, m_MaskCache, &m_Cancel);

		{
			std::lock_guard<std::mutex> lk(m_Mutex);
			if (res.success)	m_Ready = std::move(res);
			m_Busy.store(false, std::memory_order_release);
		}
		m_cvIdle.notify_all();	// CancelAndWait 대기 해제
	}
}


void FieldWorker::Start(const VoxelGrid& grid, const ChunkGraph& chunkGraph)
{
	if (m_Thread.joinable()) return;   // 중복 기동 방어

	m_Grid = &grid;
	m_ChunkGraph = &chunkGraph;
	m_Quit = false;

	m_Thread = std::thread(&FieldWorker::Worker, this);
}

void FieldWorker::Stop()
{
	if (false == m_Thread.joinable()) return;

	// 계산 중일 수 있다 - 취소를 먼저 세워야 최대 8.88ms를 기다리지 않는다
	m_Cancel.store(true, std::memory_order_release);
	{
		std::lock_guard<std::mutex> lk(m_Mutex);
		m_Quit = true;
		m_Pending.Clear();
	}
	m_cvWork.notify_all();

	m_Thread.join();

	// join 후에는 워커가 없으므로 락 없이 정리해도 안전하다
	m_Ready.Clear();
	m_Cancel.store(false, std::memory_order_release);
	m_Grid = nullptr;
	m_ChunkGraph = nullptr;
}


void FieldWorker::Submit(FieldBuildRequest&& req)
{
	// 유효 멤버 있는지 확인후, 깨우기
	if (false == req.IsValid())	return;

	{
		std::lock_guard<std::mutex> lk(m_Mutex);
		m_Pending = std::move(req);	// 이전 대기 요청 덮어쓰기
	}

	m_cvWork.notify_one();
}

FieldBuildResult FieldWorker::TryAcquire()
{
	FieldBuildResult res;
	{
		std::lock_guard<std::mutex> lk(m_Mutex);
		if (false == m_Ready.success)	return res;	// 빈거 반환

		res = std::move(m_Ready);
		m_Ready.Clear();	// move 후 비워주기

	}
	
	m_cvWork.notify_one();		// 슬롯 비어서 다시 호출
	return res;
}

void FieldWorker::CancelAndWait()
{
	m_Cancel.store(true, std::memory_order_release);

	{
		std::unique_lock<std::mutex> lk(m_Mutex);
		m_Pending.Clear();
		m_cvIdle.wait(lk, [this] {
			return false == m_Busy.load(std::memory_order_acquire);
			});
		m_Ready.Clear();

		// 지형 바뀔 예정 - 마스크 셀 구성 무효
		m_MaskCache.Clear();
	}

	m_Cancel.store(false, std::memory_order_release);
}
