#pragma once

#include <Rendering/Core/Device.h>
#include <Core/Memory.h>

template <class T>
class AlignedDeviceVector
{
public:
	AlignedDeviceVector() = delete;
	explicit AlignedDeviceVector(const RenderResourceRef<Device>& device)
		: _device(device)
		, _data(nullptr)
		, _bufferAlignment(_device->GetMinimumMemoryAlignment<T>())
	{
	}

	~AlignedDeviceVector()
	{
		Destroy();
	}

	void Resize(const std::size_t capacity)
	{
		if (_data != nullptr)
		{
			Destroy();
		}

		_capacity = capacity;

		const auto bufferSize = capacity * _bufferAlignment;

		_data = static_cast<T*>(Memory::AlignedAlloc(bufferSize, _bufferAlignment));
	}

	// Create an object in aligned storage
	template<typename... Args>
	void Emplace(Args&&... args)
	{
		CheckFormat(_size < _capacity - 1, "Size of {} is greater than capacity {}.", _size, _capacity);

		// Construct value in memory of aligned storage using inplace operator new.
		::new(reinterpret_cast<T*>(reinterpret_cast<uint64_t>(_data) + (_size * _bufferAlignment))) T(std::forward<Args>(args)...);
		++_size;
	}

	T* Data() const
	{
		return _data;
	}

	T& operator[](const std::size_t pos)
	{
		return *std::launder(reinterpret_cast<T*>(reinterpret_cast<uint64_t>(_data) + (pos * _bufferAlignment)));
	}

	const T& operator[](const std::size_t pos) const
	{
		return *std::launder(reinterpret_cast<T*>(reinterpret_cast<uint64_t>(_data) + (pos * _bufferAlignment)));
	}

private:
	void Destroy()
	{
		for (std::size_t pos = 0; pos < _size; ++pos)
		{
			std::destroy_at(std::launder(reinterpret_cast<T*>(&_data[pos])));
		}

		Memory::AlignedFree(_data);
	}

private:
	RenderResourceRef<Device> _device;

	T* _data;
	std::size_t _size = 0;
	std::size_t _capacity = 0;
	std::size_t _bufferAlignment = 0;
};