#pragma once

#include <Rendering/RenderPipelineManager.h>
// #include <Rendering/Core/Pipeline.h>

class RenderPipelineManager;
class RenderResourceManager;
class RenderCommandScheduler;

/**
 * Queue types that are resolved in the command scheduler.
 */
enum class RenderCommandQueueType : uint8_t
{
	Render,
	Present,
	Compute
};

/**
 * Base class for each render command.
 * @tparam QueueType The type of queue to be used for the specific command.
 */
template <RenderCommandQueueType QueueType>
class RenderCommand
{
public:
	RenderCommand(const std::weak_ptr<RenderCommandScheduler>& renderCommandScheduler,
		const std::weak_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::weak_ptr<RenderResourceManager>& renderResourceManager,
		const RenderResourceRef<Device>& device,
		const VkCommandPool commandPool,
		const VkQueue submitQueue)
		: _renderCommandScheduler(renderCommandScheduler)
		, _renderPipelineManager(renderPipelineManager)
		, _renderResourceManager(renderResourceManager)
		, _device(device)
		, _commandPool(commandPool)
		, _submitQueue(submitQueue)
	{ }

	static constexpr RenderCommandQueueType GetQueueType()
	{
		return QueueType;
	}

protected:
	/**
	 * Get the pipeline object from the manager by locking the weak pointer and asking for it from the handle.
	 * @param pipelineHandle The pipeline handle.
	 * @return The pipeline object from the manager.
	 */
	PipelineObject GetPipelineObject(const PipelineHandle pipelineHandle) const;

	/**
	 * Begins a one-time Vulkan command to be executed immediately on the GPU.
	 * @return A one-time Vulkan command buffer to be executed immediately on the GPU.
	 */
	[[nodiscard]] VkCommandBuffer BeginOneTimeCommandBuffer() const;

	/**
	 * Ends the one-time command and submits it to the given queue immediately while waiting for it to finish. This
	 * call will block the CPU waiting for it to complete.
	 * @param commandBuffer A command buffer that we recorded the commands onto.
	 */
	void EndOneTimeCommandBuffer(const VkCommandBuffer commandBuffer) const;

protected:
	/// Pointer back to the scheduler if we want to execute another command within the current one.
	std::weak_ptr<RenderCommandScheduler> _renderCommandScheduler { };
	std::weak_ptr<RenderPipelineManager> _renderPipelineManager { };
	std::weak_ptr<RenderResourceManager> _renderResourceManager { };

	RenderResourceRef<Device> _device { };
	VkCommandPool _commandPool = VK_NULL_HANDLE;
	VkQueue _submitQueue = VK_NULL_HANDLE;
};

template <RenderCommandQueueType QueueType>
PipelineObject RenderCommand<QueueType>::GetPipelineObject(const PipelineHandle pipelineHandle) const
{
	const auto pipelineManagerPtr = _renderPipelineManager.lock();
	if (pipelineManagerPtr == nullptr)
	{
		return PipelineObject();
	}

	const auto& pipelineObject = pipelineManagerPtr->GetPipelineObject(pipelineHandle);
	CheckFormat(pipelineObject._handle != VK_NULL_HANDLE, "Pipeline object is a nullptr.");

	return pipelineObject;
}

template <RenderCommandQueueType QueueType>
VkCommandBuffer RenderCommand<QueueType>::BeginOneTimeCommandBuffer() const
{
	CHECK_VK_HANDLE(_device);
	CHECK_VK_HANDLE(_commandPool);

	VkCommandBufferAllocateInfo allocateInfo { };
	allocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocateInfo.commandPool = _commandPool;
	allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocateInfo.commandBufferCount = 1;

	VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
	VK_CHECK(vkAllocateCommandBuffers(_device, &allocateInfo, &commandBuffer));

	VkCommandBufferBeginInfo beginInfo { };
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	// @TODO Fix me with fence.
	vkBeginCommandBuffer(commandBuffer, &beginInfo);

	return commandBuffer;
}

template <RenderCommandQueueType QueueType>
void RenderCommand<QueueType>::EndOneTimeCommandBuffer(const VkCommandBuffer commandBuffer) const
{
	vkEndCommandBuffer(commandBuffer);

	VkSubmitInfo submitInfo { };
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &commandBuffer;

	// @TODO Fix me with fence.
	vkQueueSubmit(_submitQueue, 1, &submitInfo, VK_NULL_HANDLE);
	vkQueueWaitIdle(_submitQueue);

	vkFreeCommandBuffers(_device, _commandPool, 1, &commandBuffer);
}

class LambdaRenderCommand : public RenderCommand<RenderCommandQueueType::Render>
{
public:
	using RenderCommand::RenderCommand;

	template <typename FunctionType>
	void Execute(FunctionType&& f) const
	{
		CHECK_VK_HANDLE(_device);
		CHECK_VK_HANDLE(_commandPool);
		CHECK_VK_HANDLE(_submitQueue);

		const auto commandBuffer = BeginOneTimeCommandBuffer();

		f(commandBuffer);

		EndOneTimeCommandBuffer(commandBuffer);
	}
};
