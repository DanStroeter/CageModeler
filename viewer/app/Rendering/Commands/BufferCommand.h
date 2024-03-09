#pragma once

#include <Rendering/Commands/RenderCommand.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/RenderResourceManager.h>

class CreateBufferCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	/**
	 * Creates a new buffer and allocates memory on the GPU, then copies the contents of the buffer data into the buffer
	 * memory. This will happen using an intermediate transfer buffer.
	 * @tparam T The type of buffer data.
	 * @param context A context containing relevant data to schedule and execute the command.
	 * @param bufferData The buffer data.
	 * @param bufferUsage Buffer usage flags.
	 */
	template <typename T>
	Buffer Execute(const RenderCommandExecutionContext& context,
		std::span<T> bufferData,
		const VkBufferUsageFlags bufferUsage) const
	{
		CHECK_VK_HANDLE(context._device);

		const auto renderResourceManager = context._renderResourceManager.lock();
		if (renderResourceManager == nullptr)
		{
			return { };
		}

		const auto stagingBuffer = renderResourceManager->AllocateDeviceBuffer(bufferData.size_bytes(),
			VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

		// Map the actual memory block on to the GPU.
		void *data;
		vkMapMemory(context._device, stagingBuffer._deviceMemory, 0, stagingBuffer._allocatedSize, 0, &data);
		memcpy(data, bufferData.data(), stagingBuffer._allocatedSize);
		vkUnmapMemory(context._device, stagingBuffer._deviceMemory);

		const auto deviceBuffer = renderResourceManager->AllocateDeviceBuffer(bufferData.size_bytes(),
			bufferUsage,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

		// Copy the buffer from the staging buffer to the device buffer.
		{
			const auto commandBuffer = VulkanUtils::BeginOneTimeCommandBuffer(context._device, context._commandPool);

			VkBufferCopy copyRegion { };
			copyRegion.srcOffset = 0;
			copyRegion.dstOffset = 0;
			copyRegion.size = stagingBuffer._allocatedSize;

			vkCmdCopyBuffer(commandBuffer, stagingBuffer._deviceBuffer, deviceBuffer._deviceBuffer, 1, &copyRegion);

			VulkanUtils::EndOneTimeCommandBuffer(context._device, context._commandPool, commandBuffer, context._submitQueue);
		}

		stagingBuffer.ReleaseResource(context._device);

		return deviceBuffer;
	}
};

class UpdateBufferCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	/**
	 * Updates an existing buffer and allocates memory on the GPU, then copies the contents of the buffer data into the buffer
	 * memory. This will happen using an intermediate transfer buffer.
	 * @tparam T The type of buffer data.
	 * @param context A context containing relevant data to schedule and execute the command.
	 * @param bufferData The buffer data.
	 * @param dstBuffer Already allocated buffer on the GPU.
	 * @return A new buffer with contents from the input data.
	 */
	template <typename T>
	void Execute(const RenderCommandExecutionContext& context,
		std::span<T> bufferData,
		const Buffer& dstBuffer) const
	{
		CHECK_VK_HANDLE(context._device);
		CHECK_VK_HANDLE(dstBuffer._deviceBuffer);
		CHECK_VK_HANDLE(dstBuffer._deviceMemory);

		CheckFormat(dstBuffer._allocatedSize == bufferData.size_bytes(), "Buffer sizes are different, use CreateBuffer to create a new buffer on the GPU.");

		const auto renderResourceManager = context._renderResourceManager.lock();
		if (renderResourceManager == nullptr)
		{
			return;
		}

		const auto stagingBuffer = renderResourceManager->AllocateDeviceBuffer(bufferData.size_bytes(),
			VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

		// Map the actual memory block on to the GPU.
		void *data;
		vkMapMemory(context._device, stagingBuffer._deviceMemory, 0, stagingBuffer._allocatedSize, 0, &data);
		memcpy(data, bufferData.data(), stagingBuffer._allocatedSize);
		vkUnmapMemory(context._device, stagingBuffer._deviceMemory);

		// Copy the buffer from the staging buffer to the device buffer.
		{
			const auto commandBuffer = VulkanUtils::BeginOneTimeCommandBuffer(context._device, context._commandPool);

			VkBufferCopy copyRegion { };
			copyRegion.srcOffset = 0;
			copyRegion.dstOffset = 0;
			copyRegion.size = stagingBuffer._allocatedSize;

			vkCmdCopyBuffer(commandBuffer, stagingBuffer._deviceBuffer, dstBuffer._deviceBuffer, 1, &copyRegion);

			VulkanUtils::EndOneTimeCommandBuffer(context._device, context._commandPool, commandBuffer, context._submitQueue);
		}

		stagingBuffer.ReleaseResource(context._device);
	}
};

class CopyBufferCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	/**
	 * Copies the contents of the source buffer into the destination buffer. The copy happens immediately on the GPU.
	 * @param context A context containing relevant data to schedule and execute the command.
	 * @param srcBuffer A reference to a source buffer.
	 * @param dstBuffer A reference to the destination buffer.
	 */
	void Execute(const RenderCommandExecutionContext& context,
		const Buffer& srcBuffer,
		const Buffer& dstBuffer) const
	{
		CHECK_VK_HANDLE(context._commandPool);
		CHECK_VK_HANDLE(context._submitQueue);

		const auto commandBuffer = VulkanUtils::BeginOneTimeCommandBuffer(context._device, context._commandPool);

		VkBufferCopy copyRegion { };
		copyRegion.srcOffset = 0;
		copyRegion.dstOffset = 0;
		copyRegion.size = srcBuffer._allocatedSize;

		vkCmdCopyBuffer(commandBuffer, srcBuffer._deviceBuffer, dstBuffer._deviceBuffer, 1, &copyRegion);

		VulkanUtils::EndOneTimeCommandBuffer(context._device, context._commandPool, commandBuffer, context._submitQueue);
	}
};

class CopyBufferToImageCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	/**
	 * Copies a memory from a buffer into an image.
	 * @param context A context containing relevant data to schedule and execute the command.
	 * @param sourceBuffer A buffer reference.
	 * @param targetImage An image reference.
	 * @param width The width of the image.
	 * @param height The height of the image.
	 */
	void Execute(const RenderCommandExecutionContext& context,
		const Buffer& sourceBuffer,
		const Image& targetImage,
		const uint32_t width,
		const uint32_t height) const
	{
		CHECK_VK_HANDLE(context._device);
		CHECK_VK_HANDLE(context._commandPool);
		CHECK_VK_HANDLE(context._submitQueue);

		const auto commandBuffer = VulkanUtils::BeginOneTimeCommandBuffer(context._device, context._commandPool);

		VkBufferImageCopy copy { };
		copy.bufferOffset = 0;
		copy.bufferRowLength = 0;
		copy.bufferImageHeight = 0;
		copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		copy.imageSubresource.mipLevel = 0;
		copy.imageSubresource.baseArrayLayer = 0;
		copy.imageSubresource.layerCount = 1;
		copy.imageOffset = { 0, 0, 0 };
		copy.imageExtent = { width, height, 1 };

		vkCmdCopyBufferToImage(commandBuffer, sourceBuffer._deviceBuffer, targetImage._deviceBuffer, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);

		VulkanUtils::EndOneTimeCommandBuffer(context._device, context._commandPool, commandBuffer, context._submitQueue);
	}
};

class QueueWaitCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	/**
	 * Waits for the graphics queue to finish work.
	 * @param context A context containing relevant data to schedule and execute the command.
	 */
	void Execute(const RenderCommandExecutionContext& context) const
	{
		CHECK_VK_HANDLE(context._submitQueue);

		vkQueueWaitIdle(context._submitQueue);
	}
};