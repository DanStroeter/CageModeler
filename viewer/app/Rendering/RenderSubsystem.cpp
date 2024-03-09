#include <Rendering/RenderSubsystem.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Commands/RenderPassCommand.h>
#include <Rendering/RenderPipelineManager.h>
#include <Rendering/Utils/VulkanValidation.h>
#include <Navigation/CameraSubsystem.h>
#include <Input/InputSubsystem.h>
#include <UI/WindowSubsystem.h>
#include <Configuration.h>

#include "Core/RenderProxyCollector.h"

void RenderSubsystem::Initialize(const SubsystemsCollection& collection)
{
	LOG_DEBUG("Vulkan initialization started...");

	// Initialize Volk before we do anything Vulkan related.
	VK_CHECK(volkInitialize());

	_cameraSubsystem = GetDependencySubsystem<CameraSubsystem>(collection);
	_windowSubsystem = GetDependencySubsystem<WindowSubsystem>(collection);
	const auto instanceExtensions = _windowSubsystem->GetRequiredInstanceExtensions();

	_instance = CreateRenderResource<Instance>(instanceExtensions);

	// Load the Vulkan instance function pointers.
	volkLoadInstance(_instance->GetReference());

	// Creates a new Vulkan surface for the presentation in SDL.
	const auto vulkanSurface = _windowSubsystem->CreateVulkanSurface(_instance);
	_surface = CreateRenderResource<RenderSurface>(vulkanSurface, _instance);

	// Set up the debug messenger instance if we have validation layers.
	if constexpr (MCD::EnableVulkanValidation)
	{
		const auto debugCreateInfo = VulkanValidation::CreateInfo();
		VK_CHECK(VulkanValidation::CreateDebugUtilsMessengerEXT(_instance, &debugCreateInfo, nullptr, &_debugMessenger));
	}

	// Creates the logical and physical devices used for graphics and presentation.
	_device = CreateRenderResource<Device>(_instance, _surface);

	// Create the pipeline factory to create all graphics pipelines.
	_renderPipelineManager = std::make_shared<RenderPipelineManager>(_device);

	// Load the _device function pointers.
	volkLoadDevice(_device);

	// Allocate a new manager for the resources we allocate using Vulkan.
	_renderResourceManager = std::make_shared<RenderResourceManager>(_device);
	_renderCommandScheduler = std::make_shared<RenderCommandScheduler>(_device, _renderPipelineManager, _renderResourceManager);

	// Create the descriptor pool for all descriptor sets.
	_descriptorPool = CreateRenderResource<DescriptorPool>(_device);

	// Initialize the swapchain after the resource manager is created.
	_swapchain = CreateRenderResource<Swapchain>(_renderResourceManager,
		_renderCommandScheduler,
		_surface,
		_device,
		_instance,
		_windowSubsystem->GetDrawableSize());

	CreateFrameSynchronization();

	// Create the main render pass and the framebuffers.
	_renderPass = CreateRenderPass();
	_framebuffers = _swapchain->CreateFramebuffers(_renderPass);

	// Create the render proxy collector which will gather all proxies and render them in this subsystem.
	_renderProxyCollector = std::make_shared<RenderProxyCollector>(_device, _renderCommandScheduler, _renderResourceManager);

	// Initalize the UI backend and the editor mediator code.
	const auto inputSubsystem = collection.GetDependencySubsystem<InputSubsystem>();
	_uiBackend = std::make_unique<UserInterfaceBackend>(inputSubsystem,
		_instance,
		_device,
		_swapchain,
		_renderCommandScheduler);

	LOG_DEBUG("Vulkan initialization finished.");
}

void RenderSubsystem::Deinitialize()
{
	_device->WaitIdle();

	ReleaseResource();

	if constexpr (MCD::EnableVulkanValidation)
	{
		VulkanValidation::DestroyDebugUtilsMessengerEXT(_instance, _debugMessenger, nullptr);
	}
}

void RenderSubsystem::InitializeEditor(const std::shared_ptr<Editor>& editor)
{
	_sceneRenderer = std::make_shared<SceneRenderer>(_device,
		_descriptorPool,
		_renderPipelineManager,
		_renderProxyCollector,
		_renderCommandScheduler,
		_renderResourceManager,
		_renderPass);

	_editor = editor;
	_editor->Initialize(_sceneRenderer);

	// Finishes the ImGui setup.
	_uiBackend->Init(*_windowSubsystem->_window);
}

void RenderSubsystem::ReleaseResource()
{
	CHECK_VK_HANDLE(_device);

	for (const auto framebuffer : _framebuffers)
	{
		vkDestroyFramebuffer(_device, framebuffer, nullptr);
	}

	_swapchain.Reset();
}

void RenderSubsystem::OnWindowResized()
{
	// Get the drawable size and clamp it to the Vulkan min/max values.
	const auto drawableSize = _windowSubsystem->GetDrawableSize();
	const auto& swapchainSupport = _device->GetSwapchainSupport();
	const auto width = std::clamp(drawableSize.width,
		swapchainSupport._surfaceCapabilities.minImageExtent.width,
		swapchainSupport._surfaceCapabilities.maxImageExtent.width);
	const auto height = std::clamp(drawableSize.height,
		swapchainSupport._surfaceCapabilities.minImageExtent.height,
		swapchainSupport._surfaceCapabilities.maxImageExtent.height);
	const VkExtent2D swapchainExtent { width, height };

	// If nothing changed, then don't do anything further.
	if (swapchainExtent.width == _swapchain->GetExtent().width &&
		swapchainExtent.height == _swapchain->GetExtent().height)
	{
		return;
	}

	_device->WaitIdle();

	ReleaseResource();

	_swapchain = CreateRenderResource<Swapchain>(_renderResourceManager,
		_renderCommandScheduler,
		_surface,
		_device,
		_instance,
		swapchainExtent);
	_renderPass = CreateRenderPass();

	_uiBackend->ReleaseResource();
	_renderPipelineManager->ReleaseResource();

	_sceneRenderer->CreateResources();

	_framebuffers = _swapchain->CreateFramebuffers(_renderPass);

	_uiBackend->CreateResources(_swapchain);
}

void RenderSubsystem::Render(const double deltaTime)
{
	CHECK_VK_HANDLE(_device);

	// Wait for the image fence to be released.
	_device->WaitForFence(_imageInFlightFences[_currentFrameIndex]);

	const auto [result, imageIndex] = _swapchain->AcquireNextImage(_imageAvailableSemaphores[_currentFrameIndex], VK_NULL_HANDLE);

	// Window size has changed.
	if (result == VK_ERROR_OUT_OF_DATE_KHR)
	{
		OnWindowResized();

		return;
	}

	// Could not get any Vulkan image means we need to abort.
	if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
	{
		LOG_ERROR("Vulkan image cannot be acquired.");

		abort();
	}

	// Only reset the fence if we are submitting work.
	_device->ResetFence(_imageInFlightFences[_currentFrameIndex]);

	const auto currentCommandBuffer = _renderCommandScheduler->GetCommandBuffer(_currentFrameIndex);
	vkResetCommandBuffer(currentCommandBuffer, 0);

	// Begins a new render pass.
	_renderCommandScheduler->ExecuteCommand<BeginRenderPass>(_renderPass, currentCommandBuffer, _framebuffers[imageIndex], _swapchain->GetExtent());

	// Update the projection matrix and upload the data to the GPU in the uniform buffer and render the scene.
	const auto& camera = _cameraSubsystem->GetCamera();
	_sceneRenderer->Render(deltaTime, _currentFrameIndex, camera.GetViewInfo());

	// Completes the current render pass.
	_renderCommandScheduler->ExecuteCommand<EndRenderPass>(currentCommandBuffer);

	std::vector<VkCommandBuffer> commandBuffers;
	commandBuffers.reserve(2);
	commandBuffers.push_back(currentCommandBuffer);

	// First render the UI subsystem which will create the buffers required to record into the command buffer.
	_uiBackend->BeginRender();
	_editor->RecordUI();
	_uiBackend->EndRender();

	// Record the actual UI draw data into the buffer.
	const auto uiCommandBuffer = _uiBackend->RecordCommandBuffer(imageIndex, _currentFrameIndex);
	commandBuffers.push_back(uiCommandBuffer);

	std::vector<VkSemaphore> waitSemaphores { _imageAvailableSemaphores[_currentFrameIndex] };
	std::array<VkPipelineStageFlags, 1> waitStages { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
	std::array<VkSemaphore, 1> signalSemaphores { _renderFinishedSemaphores[_currentFrameIndex] };

	_renderCommandScheduler->ExecuteCommand<SubmitCommand>(waitSemaphores,
		signalSemaphores,
		waitStages,
		commandBuffers,
		_imageInFlightFences[_currentFrameIndex]);

	std::array<VkSwapchainKHR, 1> swapchains { _swapchain };

	const auto presentResult = _renderCommandScheduler->ExecuteCommand<PresentCommand>(swapchains,
		signalSemaphores,
		imageIndex);

	if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR)
	{
		// Viewport resized.
		OnWindowResized();
	}
	else if (presentResult != VK_SUCCESS)
	{
		LOG_ERROR("Failed to present a Vulkan image.");

		abort();
	}

	_currentFrameIndex = (_currentFrameIndex + 1) % VulkanUtils::NumRenderFramesInFlight;
}

VkRenderPass RenderSubsystem::CreateRenderPass() const
{
	CHECK_VK_HANDLE(_device);

	VkAttachmentReference colorAttachmentRef { };
	colorAttachmentRef.attachment = 0;
	colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	VkAttachmentReference depthAttachmentRef { };
	depthAttachmentRef.attachment = 1;
	depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	VkAttachmentReference colorAttachmentResolveRef { };
	colorAttachmentResolveRef.attachment = 2;
	colorAttachmentResolveRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	VkSubpassDescription subpassDescription { };
	subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpassDescription.colorAttachmentCount = 1;
	subpassDescription.pColorAttachments = &colorAttachmentRef;
	subpassDescription.pDepthStencilAttachment = &depthAttachmentRef;
	subpassDescription.pResolveAttachments = &colorAttachmentResolveRef;

	VkSubpassDependency colorDependency { };
	colorDependency.srcSubpass = VK_SUBPASS_EXTERNAL;
	colorDependency.dstSubpass = 0;
	colorDependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
	colorDependency.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
	colorDependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
	colorDependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

	const auto sampleCount = _device->GetMaximumUsableSampleCount();

	// Output color attachment of the render pass.
	VkAttachmentDescription colorAttachment { };
	colorAttachment.format = _swapchain->GetOptimalFormat().format;
	colorAttachment.samples = sampleCount;
	colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	colorAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	// Multisampling attachment.
	VkAttachmentDescription colorAttachmentResolve { };
	colorAttachmentResolve.format = _swapchain->GetOptimalFormat().format;
	colorAttachmentResolve.samples = VK_SAMPLE_COUNT_1_BIT;
	colorAttachmentResolve.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachmentResolve.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colorAttachmentResolve.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachmentResolve.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	colorAttachmentResolve.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	colorAttachmentResolve.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

	// The depth attachment of the render pass.
	VkAttachmentDescription depthAttachment { };
	depthAttachment.format = _device->FindDepthFormat();
	depthAttachment.samples = sampleCount;
	depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	const std::array attachments { colorAttachment, depthAttachment, colorAttachmentResolve };
	VkRenderPassCreateInfo renderPassCreateInfo { };
	renderPassCreateInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	renderPassCreateInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
	renderPassCreateInfo.pAttachments = attachments.data();
	renderPassCreateInfo.subpassCount = 1;
	renderPassCreateInfo.pSubpasses = &subpassDescription;
	renderPassCreateInfo.dependencyCount = 1;
	renderPassCreateInfo.pDependencies = &colorDependency;

	VkRenderPass renderPass = VK_NULL_HANDLE;
	VK_CHECK(vkCreateRenderPass(_device, &renderPassCreateInfo, nullptr, &renderPass));

	return renderPass;
}

void RenderSubsystem::CreateFrameSynchronization()
{
	_imageAvailableSemaphores.resize(VulkanUtils::NumRenderFramesInFlight);
	_renderFinishedSemaphores.resize(VulkanUtils::NumRenderFramesInFlight);
	_imageInFlightFences.resize(VulkanUtils::NumRenderFramesInFlight);

	for (std::size_t index = 0; index < VulkanUtils::NumRenderFramesInFlight; ++index)
	{
		_imageAvailableSemaphores[index] = _renderResourceManager->CreateSemaphore();
		_renderFinishedSemaphores[index] = _renderResourceManager->CreateSemaphore();
		_imageInFlightFences[index] = _renderResourceManager->CreateFence(VK_FENCE_CREATE_SIGNALED_BIT);
	}
}
