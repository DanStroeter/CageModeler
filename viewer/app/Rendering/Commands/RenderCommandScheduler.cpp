#include <Rendering/RenderPipelineManager.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/Device.h>

PipelineObject RenderCommandExecutionContext::GetPipelineObject(const PipelineHandle pipelineHandle) const
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

RenderCommandScheduler::RenderCommandScheduler(const RenderResourceRef<Device>& device,
	const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
	const std::shared_ptr<RenderResourceManager>& renderResourceManager)
	: _device(device)
	, _renderPipelineManager(renderPipelineManager)
	, _renderResourceManager(renderResourceManager)
{
	_queues = GetDeviceQueues();

	// Create the command pool and buffers.
	_commandPool = CreateCommandPool(_device->GetQueueFamilies());
	_commandBuffers = CreateCommandBuffers();
}

RenderCommandScheduler::~RenderCommandScheduler()
{
	vkFreeCommandBuffers(_device, _commandPool, static_cast<uint32_t>(_commandBuffers.size()), _commandBuffers.data());
	vkDestroyCommandPool(_device, _commandPool, nullptr);
}

DeviceQueues RenderCommandScheduler::GetDeviceQueues() const
{
	const auto queueFamilies = _device->GetQueueFamilies();

	const auto graphicsQueue = _device->GetDeviceQueue(queueFamilies._graphics.value(), 0);
	CHECK_VK_HANDLE(graphicsQueue);

	const auto presentQueue = _device->GetDeviceQueue(queueFamilies._present.value(), 0);
	CHECK_VK_HANDLE(presentQueue);

	return DeviceQueues { graphicsQueue, presentQueue };
}

VkCommandPool RenderCommandScheduler::CreateCommandPool(const QueueFamilyIndices& queueFamilyIndices) const
{
	VkCommandPoolCreateInfo poolInfo { };
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	poolInfo.queueFamilyIndex = queueFamilyIndices._graphics.value();

	VkCommandPool commandPool = VK_NULL_HANDLE;
	VK_CHECK(vkCreateCommandPool(_device, &poolInfo, nullptr, &commandPool));

	return commandPool;
}

std::vector<VkCommandBuffer> RenderCommandScheduler::CreateCommandBuffers() const
{
	CHECK_VK_HANDLE(_device);
	CHECK_VK_HANDLE(_commandPool);

	std::vector<VkCommandBuffer> commandBuffers;
	commandBuffers.resize(VulkanUtils::NumRenderFramesInFlight);

	VkCommandBufferAllocateInfo allocateInfo { };
	allocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocateInfo.commandPool = _commandPool;
	allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocateInfo.commandBufferCount = static_cast<uint32_t>(commandBuffers.size());

	VK_CHECK(vkAllocateCommandBuffers(_device, &allocateInfo, commandBuffers.data()));

	return commandBuffers;
}

VkQueue RenderCommandScheduler::GetQueueForType(const RenderCommandQueueType queueType) const
{
	if (queueType == RenderCommandQueueType::Render)
	{
		return _queues._graphicsQueue;
	}
	else if (queueType == RenderCommandQueueType::Present)
	{
		return _queues._presentQueue;
	}
	else
	{
		CheckFormat(true, "We should have a queue type.");
	}

	return nullptr;
}
