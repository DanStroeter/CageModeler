#include <imgui_impl_sdl3.h>
#include <imgui_impl_vulkan.h>

#include <UI/UserInterfaceBackend.h>
#include <UI/WindowSubsystem.h>
#include <Input/InputSubsystem.h>
#include <Rendering/RenderSubsystem.h>
#include <Rendering/Commands/RenderCommandScheduler.h>

static void CheckVulkanUIInitialization(VkResult err)
{
	if (err == 0)
	{
		return;
	}

	LOG_CRITICAL("Could not initialize UI Vulkan backend.");

	if (err < 0)
	{
		abort();
	}
}

UserInterfaceBackend::UserInterfaceBackend(const SubsystemPtr<InputSubsystem>& inputSubsystem,
	const RenderResourceRef<Instance>& instance,
	const RenderResourceRef<Device>& device,
	const RenderResourceRef<Swapchain>& swapchain,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler)
	: _instance(instance)
	, _device(device)
	, _swapchain(swapchain)
	, _renderCommandScheduler(renderCommandScheduler)
{
	CheckFormat(_device->GetQueueFamilies()._graphics.has_value(), "No graphics queue index.");

	inputSubsystem->AddEventProcessedDelegate([](const SDL_Event& event)
		{
			ImGui_ImplSDL3_ProcessEvent(&event);
		});

	_numImages = static_cast<uint32_t>(_swapchain->GetImages().size());

	// Need to explicitly get the instance here, otherwise the dereference operator will get a pointer to the wrong object.
	auto instanceRef = _instance->GetReference();
	auto deviceRef = _device->GetReference();

	struct Funcs
	{
		VkInstance _instance = nullptr;
		VkDevice _device = nullptr;
	};

	Funcs funcs { instanceRef, deviceRef };

	ImGui_ImplVulkan_LoadFunctions([](const char* functionName, void* funcs)
		{
			const auto* functions = static_cast<Funcs*>(funcs);
			const PFN_vkVoidFunction instanceAddr = vkGetInstanceProcAddr(functions->_instance, functionName);
			const PFN_vkVoidFunction deviceAddr = vkGetDeviceProcAddr(functions->_device, functionName);

			return deviceAddr ? deviceAddr : instanceAddr;
		}, &funcs);

	const auto [width, height] = _swapchain->GetExtent();

	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.DisplaySize.x = static_cast<float>(width);
	io.DisplaySize.y = static_cast<float>(height);
	io.FontGlobalScale = 1.0f;
	io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

	// Initialize some Dear ImGui specific resources.
	_descriptorPool = CreateDescriptorPool();
	_renderPass = CreateRenderPass();

	_commandPools.resize(_numImages, VK_NULL_HANDLE);
	_commandBuffers.resize(_numImages, VK_NULL_HANDLE);
	for (std::size_t i = 0; i < _numImages; ++i)
	{
		_commandPools[i] = CreateCommandPool(_device->GetQueueFamilies()._graphics.value());
		_commandBuffers[i] = CreateCommandBuffer(_commandPools[i]);
	}

	_framebuffers = CreateFramebuffers();
}

void UserInterfaceBackend::Init(SDL_Window& window)
{
	const auto graphicsQueue = _renderCommandScheduler->GetDeviceQueues()._graphicsQueue;

	// Setup Platform/Renderer bindings.
	ImGui_ImplSDL3_InitForVulkan(&window);
	ImGui_ImplVulkan_InitInfo initInfo { };
	initInfo.Instance = _instance;
	initInfo.PhysicalDevice = _device->GetPhysicalDeviceHandle();
	initInfo.Device = _device;
	initInfo.QueueFamily = _device->GetQueueFamilies()._graphics.value();
	initInfo.Queue = graphicsQueue;
	initInfo.DescriptorPool = _descriptorPool;
	initInfo.MinImageCount = static_cast<uint32_t>(_numImages);
	initInfo.ImageCount = static_cast<uint32_t>(_numImages);
	initInfo.CheckVkResultFn = &CheckVulkanUIInitialization;
	CheckFormat(ImGui_ImplVulkan_Init(&initInfo, _renderPass), "Could not initialize ImGUI.");

	const auto commandBuffer = VulkanUtils::BeginOneTimeCommandBuffer(_device, _commandPools[0]);
	ImGui_ImplVulkan_CreateFontsTexture(commandBuffer);
	VulkanUtils::EndOneTimeCommandBuffer(_device, _commandPools[0], commandBuffer, graphicsQueue);

	ImGui_ImplVulkan_DestroyFontUploadObjects();
}

UserInterfaceBackend::~UserInterfaceBackend()
{
	// Wait for the device to finish whatever it's doing.
	_device->WaitIdle();

	// Release all resources that we have created such as framebuffers, command buffers, etc.
	ReleaseResource();

	vkDestroyRenderPass(_device, _renderPass, nullptr);

	for (const auto commandPool : _commandPools)
	{
		vkDestroyCommandPool(_device, commandPool, nullptr);
	}

	// ImGui resources that we no longer need have to be freed throug ImGui as well.
	ImGui_ImplVulkan_Shutdown();
	ImGui_ImplSDL3_Shutdown();
	ImGui::DestroyContext();
	vkDestroyDescriptorPool(_device, _descriptorPool, nullptr);
}

void UserInterfaceBackend::ReleaseResource()
{
	// Resources to destroy on _swapchain recreation
	for (const auto framebuffer : _framebuffers)
	{
		vkDestroyFramebuffer(_device, framebuffer, nullptr);
	}

	_framebuffers.clear();

	for (std::size_t i = 0; i < _commandPools.size(); ++i)
	{
		vkFreeCommandBuffers(_device, _commandPools[i], 1, &_commandBuffers[i]);
	}

	_commandPools.clear();
	_commandBuffers.clear();
}


void UserInterfaceBackend::CreateResources(const RenderResourceRef<Swapchain>& newSwapchain)
{
	_swapchain = newSwapchain;
	_numImages = static_cast<uint32_t>(newSwapchain->GetImages().size());

	const auto [width, height] = _swapchain->GetExtent();

	ImGuiIO& io = ImGui::GetIO();
	io.DisplaySize.x = static_cast<float>(width);
	io.DisplaySize.y = static_cast<float>(height);

	ImGui_ImplVulkan_SetMinImageCount(static_cast<uint32_t>(_numImages));

	// Initialize some Dear ImGui specific resources.
	_descriptorPool = CreateDescriptorPool();
	_renderPass = CreateRenderPass();

	ReleaseResource();

	CheckFormat(_device->GetQueueFamilies()._graphics.has_value(), "Graphics family index invalid or not set.");
	const auto graphicsFamilyIndex = _device->GetQueueFamilies()._graphics.value();

	_commandPools.resize(_numImages, VK_NULL_HANDLE);
	_commandBuffers.resize(_numImages, VK_NULL_HANDLE);
	for (std::size_t i = 0; i < _numImages; ++i)
	{
		_commandPools[i] = CreateCommandPool(graphicsFamilyIndex);
		_commandBuffers[i] = CreateCommandBuffer(_commandPools[i]);
	}

	_framebuffers = CreateFramebuffers();
}

VkRenderPass UserInterfaceBackend::CreateRenderPass() const
{
	// Create an attachment description for the render pass
	VkAttachmentDescription attachmentDescription { };
	attachmentDescription.format = _swapchain->GetOptimalFormat().format;
	attachmentDescription.samples = VK_SAMPLE_COUNT_1_BIT;
	attachmentDescription.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;  // Need UI to be drawn on top of main.
	attachmentDescription.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	attachmentDescription.initialLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
	attachmentDescription.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;  // Last pass so we want to present after.
	attachmentDescription.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	attachmentDescription.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;

	// Create a color attachment reference
	VkAttachmentReference attachmentReference { };
	attachmentReference.attachment = 0;
	attachmentReference.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	// Create a subpass
	VkSubpassDescription subpass { };
	subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpass.colorAttachmentCount = 1;
	subpass.pColorAttachments = &attachmentReference;

	// Create a subpass dependency to synchronize our main and UI render passes
	// We want to render the UI after the geometry has been written to the framebuffer
	// so we need to configure a subpass dependency as such
	VkSubpassDependency subpassDependency { };
	subpassDependency.srcSubpass = VK_SUBPASS_EXTERNAL;  // Create external dependency
	subpassDependency.dstSubpass = 0;                    // The geometry subpass comes first
	subpassDependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	subpassDependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	subpassDependency.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;  // Wait on writes
	subpassDependency.dstStageMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

	// Finally create the UI render pass
	VkRenderPassCreateInfo createInfo { };
	createInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	createInfo.attachmentCount = 1;
	createInfo.pAttachments = &attachmentDescription;
	createInfo.subpassCount = 1;
	createInfo.pSubpasses = &subpass;
	createInfo.dependencyCount = 1;
	createInfo.pDependencies = &subpassDependency;

	VkRenderPass renderPass = VK_NULL_HANDLE;
	VK_CHECK(vkCreateRenderPass(_device, &createInfo, nullptr, &renderPass));

	return renderPass;
}

VkDescriptorPool UserInterfaceBackend::CreateDescriptorPool() const
{
	const std::vector<VkDescriptorPoolSize> poolSizes = {{ VK_DESCRIPTOR_TYPE_SAMPLER,                100 },
														 { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 100 },
														 { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,          100 },
														 { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,          100 },
														 { VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER,   100 },
														 { VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER,   100 },
														 { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,         100 },
														 { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,         100 },
														 { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 100 },
														 { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 100 },
														 { VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT,       100 }};

	const auto size = static_cast<uint32_t>(poolSizes.size());

	VkDescriptorPoolCreateInfo poolCreateInfo { };
	poolCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	poolCreateInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
	poolCreateInfo.maxSets = 100 * size;
	poolCreateInfo.poolSizeCount = size;
	poolCreateInfo.pPoolSizes = poolSizes.data();

	VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
	VK_CHECK(vkCreateDescriptorPool(_device, &poolCreateInfo, nullptr, &descriptorPool));

	return descriptorPool;
}

VkCommandPool UserInterfaceBackend::CreateCommandPool(const uint32_t graphicsFamilyIndex) const
{
	VkCommandPoolCreateInfo commandPoolCreateInfo { };
	commandPoolCreateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	commandPoolCreateInfo.queueFamilyIndex = graphicsFamilyIndex;
	commandPoolCreateInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	VkCommandPool commandPool = VK_NULL_HANDLE;
	VK_CHECK(vkCreateCommandPool(_device, &commandPoolCreateInfo, nullptr, &commandPool));

	return commandPool;
}

VkCommandBuffer UserInterfaceBackend::CreateCommandBuffer(const VkCommandPool commandPool) const
{
	VkCommandBufferAllocateInfo allocInfo { };
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.commandPool = commandPool;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandBufferCount = 1;

	VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
	VK_CHECK(vkAllocateCommandBuffers(_device, &allocInfo, &commandBuffer));

	return commandBuffer;
}

std::vector<VkFramebuffer> UserInterfaceBackend::CreateFramebuffers() const
{
	std::vector<VkFramebuffer> framebuffers(_numImages);

	const auto [width, height] = _swapchain->GetExtent();

	VkImageView attachment[1];
	VkFramebufferCreateInfo info { };
	info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
	info.renderPass = _renderPass;
	info.attachmentCount = 1;
	info.pAttachments = attachment;
	info.width = width;
	info.height = height;
	info.layers = 1;

	const auto& imageViews = _swapchain->GetImageViews();

	// Create a framebuffer for each image view.
	for (std::size_t i = 0; i < imageViews.size(); i++)
	{
		attachment[0] = imageViews[i];
		VK_CHECK(vkCreateFramebuffer(_device, &info, nullptr, &framebuffers[i]));
	}

	return framebuffers;
}

VkCommandBuffer UserInterfaceBackend::RecordCommandBuffer(const uint32_t bufferIndex, const uint32_t currentFrameIndex) const
{
	VkCommandBufferBeginInfo cmdBufferBegin { };
	cmdBufferBegin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	cmdBufferBegin.flags |= VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	const auto commandBuffer = _commandBuffers[currentFrameIndex];
	vkResetCommandBuffer(commandBuffer, 0);

	VK_CHECK(vkBeginCommandBuffer(commandBuffer, &cmdBufferBegin));

	const auto [width, height] = _swapchain->GetExtent();

	constexpr VkClearValue clearColor = { { { 0.0f, 0.0f, 0.0f, 1.0f } } };
	VkRenderPassBeginInfo renderPassBeginInfo { };
	renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
	renderPassBeginInfo.renderPass = _renderPass;
	renderPassBeginInfo.framebuffer = _framebuffers[bufferIndex];
	renderPassBeginInfo.renderArea.extent.width = width;
	renderPassBeginInfo.renderArea.extent.height = height;
	renderPassBeginInfo.clearValueCount = 1;
	renderPassBeginInfo.pClearValues = &clearColor;

	vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

	// Grab and record the draw data for Dear ImGui.
	ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);

	// End and submit render pass
	vkCmdEndRenderPass(commandBuffer);

	VK_CHECK(vkEndCommandBuffer(commandBuffer));

	return commandBuffer;
}

void UserInterfaceBackend::BeginRender() const
{
	// Start the Dear ImGui frame
	ImGui_ImplVulkan_NewFrame();
	ImGui_ImplSDL3_NewFrame();
	ImGui::NewFrame();
}

void UserInterfaceBackend::EndRender() const
{
	ImGui::Render();
}

