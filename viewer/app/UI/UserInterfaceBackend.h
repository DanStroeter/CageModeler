#pragma once

#include <Core/Subsystem.h>
#include <Rendering/Core/Instance.h>
#include <Rendering/Core/Device.h>
#include <Rendering/Core/Swapchain.h>
#include <Rendering/Utils/VulkanUtils.h>

#include <volk.h>
#include <SDL_events.h>

class RenderCommandScheduler;
class RenderSubsystem;
class WindowSubsystem;
class InputSubsystem;

/**
 * Implements the code needed to render an ImGui interface within the exisitng rendering pipeline.
 */
class UserInterfaceBackend final
{
public:
	UserInterfaceBackend(const SubsystemPtr<InputSubsystem>& inputSubsystem,
		const RenderResourceRef<Instance>& instance,
		const RenderResourceRef<Device>& device,
		const RenderResourceRef<Swapchain>& swapchain,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler);
	~UserInterfaceBackend();

	/**
	 * Initializes the entire stack of ImGui storage needed to render into the viewport.
	 * @param window The window that was already created by SDL2.
	 */
	void Init(SDL_Window& window);

	VkCommandBuffer RecordCommandBuffer(const uint32_t bufferIndex, const uint32_t currentFrameIndex) const;
	void BeginRender() const;
	void EndRender() const;

	/**
	 * Re-creates the swapchain and other related resources. Required when the window has been resized.
	 * @param newSwapchain The new swapchain.
	 */
	void CreateResources(const RenderResourceRef<Swapchain>& newSwapchain);
	void ReleaseResource();

private:
	/**
	 * Creates the ImGUI render pass.
	 * @return The ImGUI render pass.
	 */
	[[nodiscard]] VkRenderPass CreateRenderPass() const;

	/**
	 * Creates the ImGUI descriptor pool.
	 * @return The ImGUI descriptor pool.
	 */
	[[nodiscard]] VkDescriptorPool CreateDescriptorPool() const;

	/**
	 * Creates the ImGUI command pool.
	 * @return The ImGUI command pool.
	 */
	[[nodiscard]] VkCommandPool CreateCommandPool(const uint32_t graphicsFamilyIndex) const;

	/**
	 * Creates a vector of command buffers for ImGUI command submission.
	 * @return A vector of command buffers for ImGUI command submission.
	 */
	[[nodiscard]] VkCommandBuffer CreateCommandBuffer(const VkCommandPool commandPool) const;

	/**
	 * Creates a vector of framebuffers for ImGUI.
	 * @return A vector of ImGUI framebuffers.
	 */
	[[nodiscard]] std::vector<VkFramebuffer> CreateFramebuffers() const;

private:
	std::size_t _numImages = 0;

	RenderResourceRef<Instance> _instance;
	RenderResourceRef<Device> _device;
	RenderResourceRef<Swapchain> _swapchain;
	std::shared_ptr<RenderCommandScheduler> _renderCommandScheduler = nullptr;

	VkRenderPass _renderPass = VK_NULL_HANDLE;
	VkDescriptorPool _descriptorPool = VK_NULL_HANDLE;
	std::vector<VkCommandPool> _commandPools;
	std::vector<VkCommandBuffer> _commandBuffers;
	std::vector<VkFramebuffer> _framebuffers;
};
