#pragma once

#include <memory>

/// Forward declare the pointer to use in the guard.
template <typename T> class HazardPointer;

/**
 * A simple guard around the hazard pointer in RAII style to access the underlying atomic data pointer.
 * @tparam T Type of data to have the pointer point to.
 */
template <typename T>
class HazardPointerGuard
{
public:
	explicit HazardPointerGuard(HazardPointer<T>* hazardPointer)
		: _hazardPointer(hazardPointer)
	{ }
	HazardPointerGuard(const HazardPointerGuard& other) = delete;
	HazardPointerGuard& operator=(const HazardPointerGuard& other) = delete;
	HazardPointerGuard(HazardPointerGuard&& other) = delete;
	HazardPointerGuard& operator=(HazardPointerGuard&& other) = delete;

	/// Release the pointer safely.
	~HazardPointerGuard();

	[[nodiscard]] operator bool() const;
	[[nodiscard]] T* operator->() const;
	[[nodiscard]] T& operator*() const;

private:
	HazardPointer<T>* _hazardPointer;
};

template <typename T>
class HazardPointer
{
public:
	friend class HazardPointerGuard<T>;

	/// Acquires a hazard pointer to the given atomic data.
	[[nodiscard]] static HazardPointerGuard<T> Acquire(std::atomic<T*>& data);

	/// Updates the head of the pointer with the new data.
	[[nodiscard]] static T* Update(std::atomic<T*>& oldData, T* newData);

private:
	/// Creates a new instance of the pointer.
	[[nodiscard]] static HazardPointer* Create();

	/// Releases the pointer safely which will be called when the guard scope is destroyed.
	void Release();

private:
	/// The currently active hazard pointer.
	static inline std::atomic<HazardPointer*> _head { };

	/// The actual data that the pointer points to.
	std::atomic<T*> _data { };

	/// Pointer to the next element.
	HazardPointer* _next = nullptr;

	/// Whether the pointer is active or has been retired.
	std::atomic_flag _isActive = ATOMIC_FLAG_INIT;
};

template<typename T>
HazardPointerGuard<T>::~HazardPointerGuard()
{
	_hazardPointer->Release();
}

template<typename T>
HazardPointerGuard<T>::operator bool() const
{
	return _hazardPointer->_data.load(std::memory_order_acquire) != nullptr;
}

template<typename T>
T* HazardPointerGuard<T>::operator->() const
{
	return _hazardPointer->_data.load(std::memory_order_acquire);
}

template<typename T>
T& HazardPointerGuard<T>::operator*() const
{
	return *(_hazardPointer->_data.load(std::memory_order_acquire));
}

template<typename T>
HazardPointerGuard<T> HazardPointer<T>::Acquire(std::atomic<T*>& data)
{
	auto ptr = Create();

	while (ptr->_data.exchange(data.load(std::memory_order_acquire))) {}
	// do
	// {
	// 	ptr->_data = data.load(std::memory_order_acquire);
	// } while (ptr->_data != data);

	return HazardPointerGuard(ptr);
}

template<typename T>
HazardPointer<T>* HazardPointer<T>::Create()
{
	auto headPointer = _head.load(std::memory_order_acquire);
	while (headPointer != nullptr)
	{
		if (!headPointer->_isActive.test_and_set())
		{
			return headPointer;
		}

		headPointer = headPointer->_next;
	}

	// Create a new pointer and set it as active.
	headPointer = new HazardPointer();
	headPointer->_isActive.test_and_set();

	// Re-point the next pointer.
	HazardPointer* oldHead = nullptr;
	do
	{
		oldHead = _head.load(std::memory_order_acquire);
		headPointer->_next = oldHead;
	} while(!_head.compare_exchange_weak(oldHead, headPointer));

	return headPointer;
}

template<typename T>
T* HazardPointer<T>::Update(std::atomic<T*>& oldData, T* newData)
{
	auto oldPtr = oldData.exchange(newData);

	while (true)
	{
		bool isUsed = false;
		auto head = _head.load(std::memory_order_acquire);

		// Find the first unused pointer and return it.
		while (head != nullptr)
		{
			if (oldPtr == head->_data)
			{
				isUsed = true;

				break;
			}

			head = head->_next;
		}

		if (!isUsed)
		{
			return oldPtr;
		}
	}
}

template<typename T>
void HazardPointer<T>::Release()
{
	_data.store(nullptr, std::memory_order_release);
	_isActive.clear(std::memory_order_release);
}

/**
 * A very simple implementation of a double buffer using a hazard pointer to update the contents and read back.
 * @tparam T The buffer class.
 */
template <typename T>
class DoubleBuffer
{
public:
	DoubleBuffer()
		: _readBuffer(new T)
		, _writeBuffer(new T)
	{ }

	HazardPointerGuard<T> LockRead() const
	{
		return HazardPointer<T>::Acquire(_readBuffer);
	}

	template <typename... Args>
	void Update(Args&&... args)
	{
		_writeBuffer->Update(std::forward<Args>(args)...);
		_writeBuffer = HazardPointer<T>::Update(_readBuffer, _writeBuffer);
		_writeBuffer->Update(std::forward<Args>(args)...);
	}

private:
	mutable std::atomic<T*> _readBuffer = nullptr;
	T* _writeBuffer = nullptr;
};