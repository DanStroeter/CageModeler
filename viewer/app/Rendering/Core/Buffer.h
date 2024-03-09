#pragma once

#include <Rendering/Core/Device.h>

class Device;

template <typename DataType>
struct BufferData
{
	BufferData() = default;
	BufferData(DataType deviceBuffer, const VkDeviceMemory deviceMemory)
		: _deviceBuffer(deviceBuffer)
		, _deviceMemory(deviceMemory)
	{ }

	/// The buffer on the device. (VkImage or VkBuffer)
	DataType _deviceBuffer = VK_NULL_HANDLE;

	/// The device memory after allocation.
	VkDeviceMemory _deviceMemory = VK_NULL_HANDLE;
};

struct Buffer : BufferData<VkBuffer>
{
	Buffer() = default;
	Buffer(const VkBuffer buffer, const VkDeviceMemory deviceMemory, const uint64_t allocatedSize)
		: BufferData(buffer, deviceMemory)
		, _allocatedSize(allocatedSize)
	{ }

	/**
	 * Releases the memory and the buffer from the GPU.
	 * @todo Move that into the resource manager or somewhere else where we can track individual allocations.
	 * @param device A reference to the Vulkan device.
	 */
	void ReleaseResource(const RenderResourceRef<Device>& device) const
	{
		vkDestroyBuffer(device, _deviceBuffer, nullptr);
		vkFreeMemory(device, _deviceMemory, nullptr);
	}

	/// Size of the buffer in bytes.
	uint64_t _allocatedSize { 0 };
};

struct MemoryMappedBuffer : Buffer
{
	using Buffer::Buffer;

	void CopyFrom(const Buffer& other)
	{
		_deviceBuffer = other._deviceBuffer;
		_deviceMemory = other._deviceMemory;
		_allocatedSize = other._allocatedSize;
	}

	/// Memory mapped on the host and device.
	void* _mappedData = nullptr;
};

struct Image : BufferData<VkImage>
{
	Image() = default;
	Image(const VkImage image, const VkDeviceMemory deviceMemory)
		: BufferData(image, deviceMemory)
	{ }

	/**
	 * Releases the memory and the buffer from the GPU.
	 * @todo Move that into the resource manager or somewhere else where we can track individual allocations.
	 * @param device A reference to the Vulkan device.
	 */
	void ReleaseResource(const RenderResourceRef<Device>& device) const
	{
		vkDestroyImage(device, _deviceBuffer, nullptr);
		vkFreeMemory(device, _deviceMemory, nullptr);
	}
};
