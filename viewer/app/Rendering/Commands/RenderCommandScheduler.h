#pragma once

#include <Rendering/Core/RenderResource.h>
#include <Rendering/Core/Device.h>
#include <Rendering/Commands/RenderCommand.h>

/// Holds all allocated Vulkan queues.
struct DeviceQueues
{
	VkQueue _graphicsQueue;
	VkQueue _presentQueue;
};

class RenderCommandScheduler : public std::enable_shared_from_this<RenderCommandScheduler>
{
public:
	RenderCommandScheduler(const RenderResourceRef<Device>& device,
		const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::shared_ptr<RenderResourceManager>& renderResourceManager);
	~RenderCommandScheduler();

	/**
	 * Execute a new command
	 * @tparam CommandType The command type to execute.
	 * @tparam Args Type of additional parameters.
	 * @param args Additional parameters during execution.
	 * @return Any result if the function returns something.
	 */
	template <typename CommandType, typename... Args>
	auto ExecuteCommand(Args&&... args) -> decltype(std::declval<CommandType>().Execute(std::forward<Args>(args)...))
	{
		const auto queue = GetQueueForType(CommandType::GetQueueType());

		CommandType newCommand(this->weak_from_this(),
							   _renderPipelineManager,
							   _renderResourceManager,
							   _device,
							   _commandPool,
							   queue);

		return newCommand.Execute(std::forward<Args>(args)...);
	}

	/**
	 * Returns a new command buffer we can schedule commands into.
	 * @param imageIndex The image index for the command buffer depending on the number of simultaneous images being rendered.
	 * @return A new command buffer to schedule commands into.
	 */
	[[nodiscard]] VkCommandBuffer GetCommandBuffer(const std::size_t imageIndex) const
	{
		return _commandBuffers[imageIndex];
	}

	/**
	 * @todo Move that into the private section and don't expose the queues.
	 * Returns the graphics and presentation queues.
	 * @return Handles to the graphics and present queues.
	 */
	[[nodiscard]] DeviceQueues GetDeviceQueues() const;

private:
	/**
	 * Creates a command pool to create command buffers from.
	 * @return A new command pool.
	 */
	[[nodiscard]] VkCommandPool CreateCommandPool(const QueueFamilyIndices& queueFamilyIndices) const;

	/**
	 * Creates command buffers with the maximum number of frames in flight, so we can submit commands to each one
	 * during for all frames. This can be improved a lot by allowing different threads to have different command
	 * buffers and submit concurrently.
	 * @return An array of command buffers the size of frames in flight.
	 */
	[[nodiscard]] std::vector<VkCommandBuffer> CreateCommandBuffers() const;

	/**
	 * Returns the Vulkan queue instance for a specific queue type (render, present or compute).
	 * @param queueType A specific queue type.
	 * @return The Vulkan queue for the specific queue type.
	 */
	[[nodiscard]] VkQueue GetQueueForType(const RenderCommandQueueType queueType) const;

private:
	RenderResourceRef<Device> _device;
	std::weak_ptr<RenderPipelineManager> _renderPipelineManager;
	std::weak_ptr<RenderResourceManager> _renderResourceManager;

	/// A command pool to create command buffers from.
	/// @todo Support multi-threaded execution and scheduling.
	VkCommandPool _commandPool = VK_NULL_HANDLE;

	/// All command buffers spawned from the command pool that will be used to execute commands on.
	std::vector<VkCommandBuffer> _commandBuffers;

	/// Device queues.
	DeviceQueues _queues { };
};