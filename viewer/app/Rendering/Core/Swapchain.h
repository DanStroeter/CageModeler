#pragma once

#include <volk.h>

#include <Rendering/Core/RenderSurface.h>
#include <Rendering/Core/Device.h>
#include <Rendering/Core/Instance.h>
#include <Rendering/Core/Buffer.h>

class RenderCommandScheduler;
class Semaphore;
class RenderResourceManager;

struct ImageAcquireResult
{
	VkResult _result;
	uint32_t _imageIndex;
};

class Swapchain : public RenderResource<Swapchain>
{
public:
	Swapchain(const std::shared_ptr<RenderResourceManager>& resourceManager,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
		const RenderResourceRef<RenderSurface>& surface,
		const RenderResourceRef<Device>& device,
		const RenderResourceRef<Instance>& instance,
		const VkExtent2D extent);

	void AddRef()
	{
		// Nothing to do here.
	}

	void Release();

	/**
	 * Acquires the next image in the swapchain.
	 * @param semaphore A semaphore to wait on.
	 * @param fence A fence to signal when the image is returned.
	 * @return The next image in the swapchain.
	 */
	[[nodiscard]] ImageAcquireResult AcquireNextImage(const RenderResourceRef<Semaphore>& semaphore, VkFence fence) const;

	/**
	 * Creates an array of framebuffers for the given render pass.
	 * @param renderPass A Vulkan render pass.
	 * @return An array of framebuffers.
	 */
	std::vector<VkFramebuffer> CreateFramebuffers(VkRenderPass renderPass) const;

	/**
	 * Returns all images from the swapchain.
	 * @return All images from the swapchain.
	 */
	[[nodiscard]] std::vector<VkImage> GetImages() const
	{
		return _images;
	}

	/**
	 * Returns all image views from the swapchain.
	 * @return All image views from the swapchain.
	 */
	[[nodiscard]] std::vector<VkImageView> GetImageViews() const
	{
		return _imageViews;
	}

	[[nodiscard]] bool IsValid() const
	{
		return !_images.empty() &&
			_extent.width != std::numeric_limits<uint32_t>::max() &&
			_extent.height != std::numeric_limits<uint32_t>::max();
	}

	[[nodiscard]] VkExtent2D GetExtent() const
	{
		return _extent;
	}

	[[nodiscard]] VkSurfaceFormatKHR GetOptimalFormat() const
	{
		return _optimalSurfaceFormat;
	}

	[[nodiscard]] VkSwapchainKHR GetReference() const
	{
		return _handle;
	}

private:
	/**
	 * Returns all swapchain images.
	 * @return All swapchain images.
	 */
	[[nodiscard]] std::vector<VkImage> GetSwapchainImagesInternal() const;

	/**
	 * Creates images views for each swapchain image.
	 * @return Image views for each swapchain image.
	 */
	[[nodiscard]] std::vector<VkImageView> CreateSwapchainImageViews() const;

	/*
	 * Creates the depth image, depth image view and transitions the image into the correct format.
	 */
	void CreateDepthResources();

	/**
	 * Creates the image buffer and image view required for the multisampling.
	 */
	void CreateMultisamplingFramebuffer();

private:
	std::shared_ptr<RenderResourceManager> _renderResourceManager = nullptr;
	std::shared_ptr<RenderCommandScheduler> _renderCommandScheduler = nullptr;

	VkSwapchainKHR _handle = VK_NULL_HANDLE;

	RenderResourceRef<RenderSurface> _surface;
	RenderResourceRef<Device> _device;
	RenderResourceRef<Instance> _instance;

	VkExtent2D _extent { std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max() };
	VkSurfaceFormatKHR _optimalSurfaceFormat { };
	VkPresentModeKHR _optimalPresentMode { };

	std::vector<VkImage> _images;
	std::vector<VkImageView> _imageViews;

	/// Multisampling offscreen framebuffer.
	Image _colorImage;
	VkImageView _colorImageView;

	/// Depth image buffer and image view.
	Image _depthImage;
	VkImageView _depthImageView = VK_NULL_HANDLE;
};
