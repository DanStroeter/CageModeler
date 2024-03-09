#pragma once

#include <memory>
#include <queue>
#include <future>
#include <type_traits>

#include <Thread/ThreadSafeQueue.h>
#include <Thread/FunctionWrapper.h>

class WorkStealingQueue
{
public:
	WorkStealingQueue() = default;

	WorkStealingQueue(const WorkStealingQueue& other) = delete;
	WorkStealingQueue& operator=(const WorkStealingQueue& other) = delete;

	WorkStealingQueue(WorkStealingQueue&& other) = delete;
	WorkStealingQueue& operator=(WorkStealingQueue&& other) = delete;

	void Push(FunctionWrapper data);

	bool IsEmpty() const;

	bool TryPop(FunctionWrapper& result);

	bool TrySteal(FunctionWrapper& result);

private:
	std::deque<FunctionWrapper> _queue;
	mutable std::mutex _mutex;
};

class ThreadPool
{
public:
	explicit ThreadPool(const uint32_t numMaxThreads);
	~ThreadPool();

	template <typename FunctionType, typename ResultType = decltype(std::declval<FunctionType>()())>
	auto Submit(FunctionType f) -> std::future<ResultType>
	{
		std::packaged_task<ResultType()> task(std::move(f));
		std::future<ResultType> result(task.get_future());

		if (_localWorkQueue != nullptr)
		{
			_localWorkQueue->Push(std::move(task));
		}
		else
		{
			_poolWorkQueue.Push(std::move(task));
		}

		return result;
	}

private:
	void WorkerThread(const uint32_t threadIndex);
	void RunPendingTask();

	bool PopTaskFromLocalQueue(FunctionWrapper& task);
	bool PopTaskFromPoolQueue(FunctionWrapper& task);
	bool PopTaskFromOtherThreadQueue(FunctionWrapper& task) const;

private:
	std::atomic<bool> _isDone;

	ThreadSafeQueue<FunctionWrapper> _poolWorkQueue;

	std::vector<std::unique_ptr<WorkStealingQueue>> _queues;
	std::vector<std::thread> _threads;

	static thread_local WorkStealingQueue* _localWorkQueue;
	static thread_local uint32_t _localIndex;

	JoinThreads _threadsJoiner;
};
