#include <Thread/ThreadPool.h>
#include <Logging/Logging.h>

void WorkStealingQueue::Push(FunctionWrapper data)
{
	std::lock_guard lock(_mutex);

	_queue.push_front(std::move(data));
}

bool WorkStealingQueue::IsEmpty() const
{
	std::lock_guard lock(_mutex);

	return _queue.empty();
}

bool WorkStealingQueue::TryPop(FunctionWrapper& result)
{
	std::lock_guard lock(_mutex);

	if (_queue.empty())
	{
		return false;
	}

	result = std::move(_queue.front());
	_queue.pop_front();

	return true;
}

bool WorkStealingQueue::TrySteal(FunctionWrapper& result)
{
	std::lock_guard lock(_mutex);

	if (_queue.empty())
	{
		return false;
	}

	result = std::move(_queue.back());
	_queue.pop_back();

	return true;
}

thread_local WorkStealingQueue* ThreadPool::_localWorkQueue = nullptr;
thread_local uint32_t ThreadPool::_localIndex = 0;

ThreadPool::ThreadPool(const uint32_t numMaxThreads)
	: _isDone(false)
	, _threadsJoiner(_threads)
{
	const auto maxThreadCount = (std::max)(std::thread::hardware_concurrency(), numMaxThreads);

	_queues.reserve(maxThreadCount);
	_threads.reserve(maxThreadCount);

	for (uint32_t index = 0; index < maxThreadCount; ++index)
	{
		_queues.push_back(std::make_unique<WorkStealingQueue>());
		_threads.emplace_back(&ThreadPool::WorkerThread, this, index);

		LOG_DEBUG("Spawning thread {} with index {}.", std::hash<std::thread::id>()(_threads.back().get_id()), index);
	}
}

ThreadPool::~ThreadPool()
{
	_isDone.store(true);
}

void ThreadPool::WorkerThread(const uint32_t threadIndex)
{
	_localWorkQueue = _queues[threadIndex].get();
	_localIndex = threadIndex;

	while (!_isDone)
	{
		RunPendingTask();
	}
}

void ThreadPool::RunPendingTask()
{
	FunctionWrapper task;

	if (PopTaskFromLocalQueue(task) ||
		PopTaskFromPoolQueue(task) ||
		PopTaskFromOtherThreadQueue(task))
	{
		task();
	}
	else
	{
		std::this_thread::yield();
	}
}

bool ThreadPool::PopTaskFromLocalQueue(FunctionWrapper& task)
{
	return _localWorkQueue != nullptr && _localWorkQueue->TryPop(task);
}

bool ThreadPool::PopTaskFromPoolQueue(FunctionWrapper& task)
{
	return _poolWorkQueue.TryPop(task);
}

bool ThreadPool::PopTaskFromOtherThreadQueue(FunctionWrapper& task) const
{
	for (std::size_t i = 0; i < _queues.size(); ++i)
	{
		const auto index = (_localIndex + i + 1) % static_cast<uint32_t>(_queues.size());

		if (_queues[index]->TrySteal(task))
		{
			return true;
		}
	}

	return false;
}
