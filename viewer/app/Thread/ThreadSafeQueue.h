#pragma once

#include <queue>
#include <memory>
#include <condition_variable>

template <typename T>
class ThreadSafeQueue
{
public:
	ThreadSafeQueue() = default;

	ThreadSafeQueue(const ThreadSafeQueue& other)
	{
		std::lock_guard<std::mutex> _lock(other._mutex);

		_queue = other._queue;
	}

	ThreadSafeQueue& operator=(const ThreadSafeQueue& other)
	{
		std::lock_guard<std::mutex> _lock(other._mutex);

		_queue = other._queue;

		return *this;
	}

	ThreadSafeQueue(ThreadSafeQueue&& other) noexcept
	{
		std::lock_guard<std::mutex> lock(other._mutex);

		_queue = std::move(other._queue);
	}

	ThreadSafeQueue& operator=(ThreadSafeQueue&& other) noexcept
	{
		std::lock_guard<std::mutex> lock(other._mutex);

		_queue = std::move(other._queue);

		return *this;
	}

	void Push(T value);

	void WaitAndPop(T& value);

	std::shared_ptr<T> WaitAndPop();

	bool TryPop(T& value);

	std::shared_ptr<T> TryPop();

	bool IsEmpty() const;

private:
	mutable std::mutex _mutex;
	std::queue<T> _queue;
	std::condition_variable _cond;
};

template <typename T>
void ThreadSafeQueue<T>::Push(T value)
{
	std::lock_guard lock(_mutex);

	_queue.push(std::move(value));
	_cond.notify_one();
}

template <typename T>
void ThreadSafeQueue<T>::WaitAndPop(T& value)
{
	std::lock_guard lock(_mutex);

	_cond.wait(lock, [this]()
		{
			return !_queue.empty();
		});
	value = std::move(_queue.front());
	_queue.pop();
}

template <typename T>
std::shared_ptr<T> ThreadSafeQueue<T>::WaitAndPop()
{
	std::lock_guard lock(_mutex);

	_cond.wait(lock, [this]()
		{
			return !_queue.empty();
		});
	std::shared_ptr<T> result = std::make_shared<T>(std::move(_queue.front()));
	_queue.pop();

	return result;
}

template <typename T>
bool ThreadSafeQueue<T>::TryPop(T& value)
{
	std::lock_guard lock(_mutex);

	if (_queue.empty())
	{
		return false;
	}

	value = std::move(_queue.front());
	_queue.pop();

	return true;
}

template <typename T>
std::shared_ptr<T> ThreadSafeQueue<T>::TryPop()
{
	std::lock_guard lock(_mutex);

	if (_queue.empty())
	{
		return nullptr;
	}

	std::shared_ptr<T> result = std::make_shared<T>(std::move(_queue.front()));
	_queue.pop();

	return result;
}

template <typename T>
bool ThreadSafeQueue<T>::IsEmpty() const
{
	std::lock_guard lock(_mutex);

	return _queue.empty();
}
