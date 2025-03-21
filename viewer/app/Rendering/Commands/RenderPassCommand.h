#pragma once

#include <Rendering/Commands/RenderCommand.h>
#include <Rendering/Core/Synchronization.h>
#include <Rendering/Utils/VulkanUtils.h>

class SubmitCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	using RenderCommand::RenderCommand;

	/**
	 * Takes care of transitioning the image layout by automatically figuring out how to do it based on the old and new
	 * layouts. This is going to be issued as a 1 time off command and execute immediately on the GPU.
	 * @param waitSemaphores Semaphores to wait on before we do the submit.
	 * @param signalSemaphores Semaphores to signal after the submit is finished.
	 * @param waitStages Stages to wait on before we do the submit.
	 * @param commandBuffers Command buffers to be submitted.
	 * @param signalFence A fence to signal when the submit is finished.
	 */
	void Execute(const std::span<VkSemaphore> waitSemaphores,
		const std::span<VkSemaphore> signalSemaphores,
		const std::span<VkPipelineStageFlags> waitStages,
		const std::span<VkCommandBuffer> commandBuffers,
		const RenderResourceRef<Fence>& signalFence) const
	{
		CHECK_VK_HANDLE(_device);
		CHECK_VK_HANDLE(_commandPool);

		VkSubmitInfo drawSubmitInfo { };
		drawSubmitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
		drawSubmitInfo.waitSemaphoreCount = static_cast<uint32_t>(waitSemaphores.size());
		drawSubmitInfo.pWaitSemaphores = waitSemaphores.data();
		drawSubmitInfo.pWaitDstStageMask = waitStages.data();
		drawSubmitInfo.commandBufferCount = static_cast<uint32_t>(commandBuffers.size());
		drawSubmitInfo.pCommandBuffers = commandBuffers.data();
		drawSubmitInfo.signalSemaphoreCount = static_cast<uint32_t>(signalSemaphores.size());
		drawSubmitInfo.pSignalSemaphores = signalSemaphores.data();
		VK_CHECK(vkQueueSubmit(_submitQueue, 1, &drawSubmitInfo, signalFence));
	}
};

class PresentCommand : public RenderCommand<RenderCommandQueueType::Present>
{
public:
	using RenderCommand::RenderCommand;

	/**
	 * Schedules a present command onto the present queue.
	 * @param swapchains Swapchains that will be presented.
	 * @param signalSemaphores Semaphores that we will wait on before we execute the presentation.
	 * @param imageIndex Index of the swapchain image that should be presented.
	 */
	VkResult Execute(const std::span<VkSwapchainKHR> swapchains,
		const std::span<VkSemaphore> signalSemaphores,
		const uint32_t& imageIndex) const
	{
		CHECK_VK_HANDLE(_device);
		CHECK_VK_HANDLE(_commandPool);

		VkPresentInfoKHR presentInfo { };
		presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
		presentInfo.waitSemaphoreCount = static_cast<uint32_t>(signalSemaphores.size());
		presentInfo.pWaitSemaphores = signalSemaphores.data();
		presentInfo.swapchainCount = static_cast<uint32_t>(swapchains.size());
		presentInfo.pSwapchains = swapchains.data();
		presentInfo.pImageIndices = &imageIndex;
		presentInfo.pResults = nullptr;
		const auto presentResult = vkQueuePresentKHR(_submitQueue, &presentInfo);

		return presentResult;
	}
};

class BeginRenderPass : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	using RenderCommand::RenderCommand;

	/**
	 * Schedules a present command onto the present queue.
	 * @param renderPass The current render pass to begin.
	 * @param commandBuffer The command buffer to schedule commands on.
	 * @param framebuffer The framebuffer we are rendering onto.
	 * @param swapchainExtent The swapchain extent.
	 */
	void Execute(const VkRenderPass renderPass,
		const VkCommandBuffer commandBuffer,
		const VkFramebuffer framebuffer,
		const VkExtent2D swapchainExtent) const
	{
		CHECK_VK_HANDLE(_device);
		CHECK_VK_HANDLE(_commandPool);

		CHECK_VK_HANDLE(renderPass);
		CHECK_VK_HANDLE(framebuffer);

		// Begin recording commands on the command buffer to render the static mesh.
		VkCommandBufferBeginInfo beginInfo { };
		beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
		beginInfo.flags = 0;
		beginInfo.pInheritanceInfo = nullptr;
		VK_CHECK(vkBeginCommandBuffer(commandBuffer, &beginInfo));

		std::array<VkClearValue, 4> clearValues { };
		clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 1.0f } };
		clearValues[1].depthStencil = { 1.0f, 0 };

		// Begin the render pass here.
		VkRenderPassBeginInfo renderPassInfo { };
		renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
		renderPassInfo.renderPass = renderPass;
		renderPassInfo.framebuffer = framebuffer;
		renderPassInfo.renderArea.offset = VkOffset2D { 0, 0 };
		renderPassInfo.renderArea.extent = swapchainExtent;
		renderPassInfo.clearValueCount = 1;
		renderPassInfo.pClearValues = clearValues.data();
		renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());

		vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

		// Create a new viewport and a scissor area for the rendering area because they are dynamic.
		VkViewport viewport { };
		viewport.x = 0;
		viewport.y = 0;
		viewport.width = static_cast<float>(swapchainExtent.width);
		viewport.height = static_cast<float>(swapchainExtent.height);
		viewport.minDepth = 0.0f;
		viewport.maxDepth = 1.0f;
		vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

		VkRect2D scissor { };
		scissor.offset = VkOffset2D { 0, 0 };
		scissor.extent = swapchainExtent;
		vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
	}
};

class EndRenderPass : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	using RenderCommand::RenderCommand;

	/**
	 * Schedules a present command onto the present queue.
	 * @param commandBuffer The command buffer to schedule commands on.
	 */
	void Execute(const VkCommandBuffer commandBuffer) const
	{
		CHECK_VK_HANDLE(_device);
		CHECK_VK_HANDLE(_commandPool);

		// End and submit render pass
		vkCmdEndRenderPass(commandBuffer);

		// End the command buffer recording here.
		VK_CHECK(vkEndCommandBuffer(commandBuffer));
	}
};

