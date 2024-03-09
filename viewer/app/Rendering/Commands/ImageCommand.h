#pragma once

#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Commands/BufferCommand.h>

class TransitionImageCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	/**
	 * Takes care of transitioning the image layout by automatically figuring out how to do it based on the old and new
	 * layouts. This is going to be issued as a 1 time off command and execute immediately on the GPU.
	 * @param context A context containing relevant data to schedule and execute the command.
	 * @param image An image resource instance.
	 * @param format The desired image format.
	 * @param oldLayout The old transition layout.
	 * @param newLayout THe new transition layout.
	 */
	void Execute(
		const RenderCommandExecutionContext& context,
		const Image& image,
		const VkFormat format,
		const VkImageLayout oldLayout,
		const VkImageLayout newLayout) const
	{
		CHECK_VK_HANDLE(context._device);
		CHECK_VK_HANDLE(context._commandPool);

		const auto commandBuffer = VulkanUtils::BeginOneTimeCommandBuffer(context._device, context._commandPool);

		VkImageMemoryBarrier barrier { };
		barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		barrier.oldLayout = oldLayout;
		barrier.newLayout = newLayout;
		barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		barrier.image = image._deviceBuffer;
		barrier.subresourceRange.baseMipLevel = 0;
		barrier.subresourceRange.levelCount = 1;
		barrier.subresourceRange.baseArrayLayer = 0;
		barrier.subresourceRange.layerCount = 1;

		if (newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
		{
			barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;

			if (VulkanUtils::FormatHasStencilComponent(format))
			{
				barrier.subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
			}
		}
		else
		{
			barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		}

		VkPipelineStageFlags sourceStage { };
		VkPipelineStageFlags destinationStage { };

		if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
		{
			barrier.srcAccessMask = 0;
			barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

			sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
			destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
		}
		else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
		{
			barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

			sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
			destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
		}
		else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
		{
			barrier.srcAccessMask = 0;
			barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

			sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
			destinationStage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
		}
		else
		{
			CheckFormat(1, "Should not go in here.");
		}

		vkCmdPipelineBarrier(commandBuffer, sourceStage, destinationStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);

		VulkanUtils::EndOneTimeCommandBuffer(context._device, context._commandPool, commandBuffer, context._submitQueue);
	}
};

class CreateImageCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	/**
	 * Creates an image by uploading the image data directly to the GPU in a one-time command and returns the
	 * allocated GPU memory as a result.
	 * @param context A context containing relevant data to schedule and execute the command.
	 * @param imageData A buffer holding the image data.
	 * @param imageWidth The width of the image.
	 * @param imageHeight The height of the image.
	 * @param imageFormat The desired image format.
	 * @return A new image containing the device memory that holds the image data on the GPU.
	 */
	Image Execute(const RenderCommandExecutionContext& context,
		const std::vector<unsigned char*>& imageData,
		const std::uint32_t imageWidth,
		const std::uint32_t imageHeight,
		const VkFormat imageFormat) const
	{
		CHECK_VK_HANDLE(context._device);

		const auto renderResourceManager = context._renderResourceManager.lock();
		if (renderResourceManager == nullptr)
		{
			return { };
		}

		const auto imageSize = static_cast<VkDeviceSize>(4 * imageWidth * imageHeight);
		const auto stagingBuffer = renderResourceManager->AllocateDeviceBuffer(imageSize,
			VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

		void *data;
		vkMapMemory(context._device, stagingBuffer._deviceMemory, 0, imageSize, 0, &data);
		memcpy(data, imageData.data(), imageSize);
		vkUnmapMemory(context._device, stagingBuffer._deviceMemory);

		const auto image = renderResourceManager->CreateImage(imageWidth,
			imageHeight,
			imageFormat,
			VK_IMAGE_TILING_OPTIMAL,
			VK_SAMPLE_COUNT_1_BIT,
			VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);

		// Copy the buffer from the staging buffer to the device buffer.
		if (const auto commandScheduler = context._renderCommandScheduler.lock())
		{
			commandScheduler->ExecuteCommand<TransitionImageCommand>(image, imageFormat, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
			commandScheduler->ExecuteCommand<CopyBufferToImageCommand>(stagingBuffer, image, imageWidth, imageHeight);
			commandScheduler->ExecuteCommand<TransitionImageCommand>(image, imageFormat, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
		}

		stagingBuffer.ReleaseResource(context._device);

		return image;
	}
};
