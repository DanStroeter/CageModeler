#pragma once

#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/Device.h>
#include <Rendering/Core/Synchronization.h>
#include <Rendering/Utils/VulkanUtils.h>

/**
 * The resource manager of all Vulkan resources used by the application (all allocations on the GPU such as VkImage
 * instances, VkBuffer and others). This class will keep track of mapped memory between CPU and GPU and be responsible
 * for the allocation and de-allocation of resources. All operations are currently executed immediately, so they can be
 * improved by deferring them and scheduling them on a command buffer if we know the order prior to the execution.
 */
class RenderResourceManager final
{
public:
	explicit RenderResourceManager(const RenderResourceRef<Device>& device);
	~RenderResourceManager();

	/**
	 * Creates a new image resource with given properties.
	 * @param width The width of the image.
	 * @param height The height of the image.
	 * @param format The format of the image.
	 * @param tiling The tiling of the image.
	 * @param sampleCountFlags Sample count flags.
	 * @param bufferUsage The buffer usage. If the usage flags contain VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT we will
	 *  lazily allocate the resource.
	 * @return A new image resource with storage on the GPU with the given properties.
	 */
	[[nodiscard]] Image CreateImage(const uint32_t width,
	                                const uint32_t height,
	                                const VkFormat format,
	                                const VkImageTiling tiling,
	                                const VkSampleCountFlagBits sampleCountFlags,
	                                const VkImageUsageFlags bufferUsage) const;

	/**
	 * Creates a view of an image resource.
	 * @param image An image resource instance.
	 * @param format The desired image view format.
	 * @param aspectFlags THe aspect flags.
	 * @return A view of an image resource.
	 */
	[[nodiscard]] VkImageView CreateImageView(const Image& image, const VkFormat format, const VkImageAspectFlags aspectFlags) const;

	/**
	 * Creates a new texture sampler.
	 * @return A new texture sampler.
	 */
	[[nodiscard]] VkSampler CreateTextureSampler() const;

	/**
	 * Creates a new buffer and allocates memory on the GPU, then copies the contents of the buffer data into the buffer
	 * memory. This will happen using an intermediate transfer buffer.
	 * @tparam T The type of objects to allocate on the GPU.
	 * @param bufferData A span of objects to put into the buffer.
	 * @param bufferUsage Buffer usage flags.
	 * @param properties Memory property flags.
	 * @return A new buffer and GPU allocated memory.
	 */
	template <typename T>
	[[nodiscard]] MemoryMappedBuffer CreateBufferAndMapMemory(std::span<T> bufferData,
		const VkBufferUsageFlags bufferUsage,
		const VkMemoryPropertyFlags properties = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) const;

	/**
	 * Creates a new buffer and allocates memory on the GPU, then copies the contents of the buffer data into the buffer
	 * memory. This will happen using an intermediate transfer buffer. The memory will be aligned to the GPU UBO alignment size.
	 * @tparam T The type of objects to allocate on the GPU.
	 * @param bufferData A span of objects to put into the buffer.
	 * @param bufferUsage Buffer usage flags.
	 * @param properties Memory property flags.
	 * @return A new buffer and GPU allocated memory.
	 */
	template <typename T>
	[[nodiscard]] MemoryMappedBuffer CreateBufferAndMapMemoryAligned(std::span<T> bufferData,
		const VkBufferUsageFlags bufferUsage,
		const VkMemoryPropertyFlags properties = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) const;

	/**
	 * Creates a new semaphore for synchronization.
	 * @return A new Vulkan semaphore.
	 */
	[[nodiscard]] RenderResourceRef<Semaphore> CreateSemaphore() const
	{
		return CreateRenderResource<Semaphore>(_device);
	}

	/**
	 * Creates a new fence object for synchronization.
	 * @param fenceFlags Creation flags of the fence.
	 * @return A new Vulkan fence.
	 */
	[[nodiscard]] RenderResourceRef<Fence> CreateFence(const VkFenceCreateFlags fenceFlags) const
	{
		return CreateRenderResource<Fence>(_device, fenceFlags);
	}

	/**
	 * Allocates a buffer on the device.
	 * @param deviceSize The size of the buffer.
	 * @param bufferUsage The usage flags.
	 * @param properties Memory properties of the buffer.
	 * @return A device buffer.
	 */
	[[nodiscard]] Buffer AllocateDeviceBuffer(const VkDeviceSize deviceSize,
											  const VkBufferUsageFlags bufferUsage,
											  const VkMemoryPropertyFlags properties) const;

	/**
	 * Gets the proper buffer alignment for the given type.
	 * @tparam T The type of object to find the alignment of.
	 * @return The buffer alignment of the object type.
	 */
	template <typename T>
	[[nodiscard]] std::size_t GetBufferAlignment() const
	{
		return std::max(static_cast<std::size_t>(_device->GetPhysicalDeviceProperties().limits.minUniformBufferOffsetAlignment), sizeof(T));
	}

private:
	RenderResourceRef<Device> _device;
};

template <typename T>
MemoryMappedBuffer RenderResourceManager::CreateBufferAndMapMemory(std::span<T> bufferData,
	const VkBufferUsageFlags bufferUsage,
	const VkMemoryPropertyFlags properties) const
{
	CHECK_VK_HANDLE(_device);

	// Create a new result to copy the staging data into.
	const auto buffer = AllocateDeviceBuffer(bufferData.size_bytes(),
		bufferUsage,
		properties);
	MemoryMappedBuffer result;
	result.CopyFrom(buffer);

	// Map the actual memory block on to the GPU.
	vkMapMemory(_device, result._deviceMemory, 0, result._allocatedSize, 0, &result._mappedData);

	return result;
}

template<typename T>
MemoryMappedBuffer RenderResourceManager::CreateBufferAndMapMemoryAligned(std::span<T> bufferData,
	const VkBufferUsageFlags bufferUsage,
	const VkMemoryPropertyFlags properties) const
{
	CHECK_VK_HANDLE(_device);

	const auto alignmentSize = _device->GetMinimumMemoryAlignment<decltype(std::decay_t<T>())>();
	const auto alignedSizeInBytes = alignmentSize * bufferData.size();

	// Create a new result to copy the staging data into.
	const auto buffer = AllocateDeviceBuffer(alignedSizeInBytes,
		bufferUsage,
		properties);
	MemoryMappedBuffer result;
	result.CopyFrom(buffer);

	// Map the actual memory block on to the GPU.
	vkMapMemory(_device, result._deviceMemory, 0, result._allocatedSize, 0, &result._mappedData);

	return result;
}
