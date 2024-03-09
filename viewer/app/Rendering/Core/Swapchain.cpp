#include <Rendering/Core/Swapchain.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Utils/VulkanUtils.h>
#include <Rendering/Commands/ImageCommand.h>

Swapchain::Swapchain(const std::shared_ptr<RenderResourceManager>& resourceManager,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
	const RenderResourceRef<RenderSurface>& surface,
	const RenderResourceRef<Device>& device,
	const RenderResourceRef<Instance>& instance,
	const VkExtent2D extent)
	: _renderResourceManager(resourceManager)
	, _renderCommandScheduler(renderCommandScheduler)
	, _surface(surface)
	, _device(device)
	, _instance(instance)
{
	CHECK_VK_HANDLE(surface);

	// Creates a new _swapchain to render images onto.
	const auto queueFamilies = _device->GetQueueFamilies();

	// Get details about the _swapchain that we will need for the creation of further states.
	const auto swapchainSupportDetails = _device->GetSwapchainSupport();
	_optimalSurfaceFormat = VulkanUtils::GetOptimalSurfaceFormat(swapchainSupportDetails._surfaceFormats);
	_optimalPresentMode = VulkanUtils::GetOptimalPresentMode(swapchainSupportDetails._presentModes);
	_extent = VulkanUtils::CalculateSwapchainExtent(extent, swapchainSupportDetails._surfaceCapabilities);

	auto minImageCount = swapchainSupportDetails._surfaceCapabilities.minImageCount + 1;

	if (swapchainSupportDetails._surfaceCapabilities.maxImageCount > 0 && minImageCount > swapchainSupportDetails._surfaceCapabilities.maxImageCount)
	{
		minImageCount = swapchainSupportDetails._surfaceCapabilities.maxImageCount;
	}

	VkSwapchainCreateInfoKHR createInfo { };
	createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	createInfo.surface = surface;
	createInfo.minImageCount = minImageCount;
	createInfo.imageFormat = _optimalSurfaceFormat.format;
	createInfo.imageColorSpace = _optimalSurfaceFormat.colorSpace;
	createInfo.imageExtent = _extent;
	createInfo.imageArrayLayers = 1;
	createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

	const auto queueFamilyIndices = queueFamilies.GetGraphicsAndPresent();

	if (queueFamilies._graphics != queueFamilies._present)
	{
		createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
		createInfo.queueFamilyIndexCount = static_cast<uint32_t>(queueFamilyIndices.size());
		createInfo.pQueueFamilyIndices = queueFamilyIndices.data();
	}
	else
	{
		createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
		createInfo.queueFamilyIndexCount = 0;
		createInfo.pQueueFamilyIndices = nullptr;
	}

	createInfo.preTransform = swapchainSupportDetails._surfaceCapabilities.currentTransform;
	createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
	createInfo.presentMode = _optimalPresentMode;
	createInfo.clipped = VK_TRUE;
	createInfo.oldSwapchain = nullptr;

	VK_CHECK(vkCreateSwapchainKHR(_device, &createInfo, nullptr, &_handle));

	// Creates all image views to be presented stacked in the swapchain.
	_images = GetSwapchainImagesInternal();
	_imageViews = CreateSwapchainImageViews();

	// Create the depth image and image view.
	CreateDepthResources();

	// Create the multisampling buffers.
	CreateMultisamplingFramebuffer();
}

void Swapchain::Release()
{
	for (const auto imageView : _imageViews)
	{
		vkDestroyImageView(_device, imageView, nullptr);
	}

	_depthImage.ReleaseResource(_device);
	vkDestroyImageView(_device, _depthImageView, nullptr);

	_colorImage.ReleaseResource(_device);
	vkDestroyImageView(_device, _colorImageView, nullptr);

	vkDestroySwapchainKHR(_device, _handle, nullptr);
}

std::vector<VkImage> Swapchain::GetSwapchainImagesInternal() const
{
	CHECK_VK_HANDLE(_device);
	CHECK_VK_HANDLE(_handle);

	uint32_t imageCount = 0;
	vkGetSwapchainImagesKHR(_device, _handle, &imageCount, nullptr);

	std::vector<VkImage> result(imageCount);
	vkGetSwapchainImagesKHR(_device, _handle, &imageCount, result.data());

	return result;
}

ImageAcquireResult Swapchain::AcquireNextImage(const RenderResourceRef<Semaphore>& semaphore, VkFence fence) const
{
	CHECK_VK_HANDLE(_device);
	CHECK_VK_HANDLE(_handle);

	uint32_t imageIndex = 0;
	const auto acquireResult = vkAcquireNextImageKHR(_device,
		_handle,
		std::numeric_limits<uint64_t>::max(),
		semaphore->GetReference(),
		VK_NULL_HANDLE,
		&imageIndex);

	return ImageAcquireResult { acquireResult, imageIndex };
}

std::vector<VkImageView> Swapchain::CreateSwapchainImageViews() const
{
#if DO_CHECK
	for (const auto& image : _images)
	{
		CHECK_VK_HANDLE(image);
	}
#endif

	Check(IsValid());

	std::vector<VkImageView> imageViews(_images.size());
	for (std::size_t index = 0; index < _images.size(); ++index)
	{
		imageViews[index] = _renderResourceManager->CreateImageView(Image(_images[index], VK_NULL_HANDLE), _optimalSurfaceFormat.format, VK_IMAGE_ASPECT_COLOR_BIT);
	}

	return imageViews;
}

void Swapchain::CreateDepthResources()
{
	const auto depthFormat = _device->FindDepthFormat();
	const auto sampleCount = _device->GetMaximumUsableSampleCount();

	// Create the depth image and the depth image view.
	_depthImage = _renderResourceManager->CreateImage(_extent.width,
		_extent.height,
		depthFormat,
		VK_IMAGE_TILING_OPTIMAL,
		sampleCount,
		VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT);
	_depthImageView = _renderResourceManager->CreateImageView(_depthImage, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);

	// Transition the depth image to the correct format.
	_renderCommandScheduler->ExecuteCommand<TransitionImageCommand>(_depthImage,
		depthFormat,
		VK_IMAGE_LAYOUT_UNDEFINED,
		VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
}

void Swapchain::CreateMultisamplingFramebuffer()
{
	const auto sampleCount = _device->GetMaximumUsableSampleCount();

	_colorImage = _renderResourceManager->CreateImage(_extent.width,
		_extent.height,
		_optimalSurfaceFormat.format,
		VK_IMAGE_TILING_OPTIMAL,
		sampleCount,
		VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
	_colorImageView = _renderResourceManager->CreateImageView(_colorImage, _optimalSurfaceFormat.format, VK_IMAGE_ASPECT_COLOR_BIT);
}

std::vector<VkFramebuffer> Swapchain::CreateFramebuffers(const VkRenderPass renderPass) const
{
#if DO_CHECK
	for (const auto& imageView : _imageViews)
	{
		CHECK_VK_HANDLE(imageView);
	}
#endif

	CHECK_VK_HANDLE(_device);

	std::vector<VkFramebuffer> framebuffers(_imageViews.size());

	for (std::size_t index = 0; index < _imageViews.size(); ++index)
	{
		const std::array attachments { _colorImageView, _depthImageView, _imageViews[index] };

		VkFramebufferCreateInfo createInfo { };
		createInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		createInfo.renderPass = renderPass;
		createInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
		createInfo.pAttachments = attachments.data();
		createInfo.width = _extent.width;
		createInfo.height = _extent.height;
		createInfo.layers = 1;

		VK_CHECK(vkCreateFramebuffer(_device, &createInfo, nullptr, &framebuffers[index]));
	}

	return framebuffers;
}
