#pragma once

#include <Rendering/Core/Pipeline.h>

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
	static constexpr RenderCommandQueueType GetQueueType()
	{
		return QueueType;
	}
};

/**
 * A context used for each command where we can access common data such as the current device, the command pool where
 * the command buffer came from and the current submission queue.
 */
struct RenderCommandExecutionContext
{
	/**
	 * Get the pipeline object from the manager by locking the weak pointer and asking for it from the handle.
	 * @param pipelineHandle The pipeline handle.
	 * @return The pipeline object from the manager.
	 */
	PipelineObject GetPipelineObject(const PipelineHandle pipelineHandle) const;

	/// TODO: Change that - really bad way!
	/// Pointer back to the scheduler if we want to execute another command within the current one.
	std::weak_ptr<RenderCommandScheduler> _renderCommandScheduler;

	RenderResourceRef<Device> _device;
	std::weak_ptr<RenderPipelineManager> _renderPipelineManager;
	std::weak_ptr<RenderResourceManager> _renderResourceManager;
	VkCommandPool _commandPool = VK_NULL_HANDLE;
	VkQueue _submitQueue = VK_NULL_HANDLE;
};