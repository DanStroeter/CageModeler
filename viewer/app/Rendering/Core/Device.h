#pragma once

#include <Rendering/Core/RenderResource.h>
#include <Rendering/Core/RenderSurface.h>
#include <Rendering/Core/Instance.h>
#include <Rendering/Utils/VulkanUtils.h>

#include <optional>

class Fence;

/// Queue indices for the graphics, compute and present queues.
struct QueueFamilyIndices
{
	[[nodiscard]] bool IsComplete() const
	{
		return _graphics.has_value() && _present.has_value() && _compute.has_value();
	}

	[[nodiscard]] std::array<uint32_t, 2> GetGraphicsAndPresent() const
	{
		Check(_graphics.has_value() && _present.has_value());

		return { _graphics.value(), _present.value() };
	}

	std::optional<uint32_t> _graphics;
	std::optional<uint32_t> _compute;
	std::optional<uint32_t> _present;
};

/// A wrapper around the VkDevice handle that exposes useful functionality and abstracts away the complexity.
class Device : public RenderResource<Device>
{
public:
	Device(const RenderResourceRef<Instance>& instance, const RenderResourceRef<RenderSurface>& surface);

	void AddRef()
	{
		// Nothing to do here.
	}

	void Release()
	{
		vkDestroyDevice(_device, nullptr);
	}

	/**
	 * Gets the swapchain details from the device based on the surface properties and capabilities.
	 * @return The swapchain details of the surface.
	 */
	[[nodiscard]] const SwapchainSupportDetails& GetSwapchainSupport() const
	{
		return _swapchainSupportDetails;
	}

	/**
	 * Find the queue family indices used for graphics, compute and presentation.
	 * @return All queue family indices.
	 */
	[[nodiscard]] QueueFamilyIndices GetQueueFamilies() const
	{
		return _queueFamilyIndices;
	}

	/**
	 * Force the device to wait idle.
	 */
	void WaitIdle() const;

	/**
	 * Wait for a specific fence on the device indefinitely.
	 * @param fence A reference ot a fence.
	 */
	void WaitForFence(const RenderResourceRef<Fence>& fence) const;

	/**
	 * Reset a fence.
	 * @param fence A reference to a fence.
	 */
	void ResetFence(const RenderResourceRef<Fence>& fence) const;

	/**
	 * Gets a handle to the device queue with the queue family index and queue index.
	 * @param queueFamilyIndex The queue family index (available from QueueFamilyIndices).
	 * @param queueIndex The index of the queue.
	 * @return
	 */
	[[nodiscard]] VkQueue GetDeviceQueue(const uint32_t queueFamilyIndex, const uint32_t queueIndex) const;

	/**
	 * Gets all physical device properties.
	 * @return All physical device properties.
	 */
	[[nodiscard]] VkPhysicalDeviceProperties GetPhysicalDeviceProperties() const
	{
		CHECK_VK_HANDLE(_physicalDevice);

		return _physicalDeviceProperties;
	}

	template <typename T>
	[[nodiscard]] constexpr uint32_t GetMinimumMemoryAlignment() const
	{
		const auto minDeviceAlignment = _physicalDeviceProperties.limits.minUniformBufferOffsetAlignment;
		uint32_t bufferAlignment = sizeof(decltype(std::decay_t<T>()));

		if (minDeviceAlignment > 0)
		{
			bufferAlignment = std::bit_ceil((bufferAlignment + minDeviceAlignment - 1) & ~(minDeviceAlignment - 1));
		}

		return bufferAlignment;
	}

	/**
	 * Returns the maximum usable sample count for multi-sampling.
	 * The maximum usable sample count.
	 */
	VkSampleCountFlagBits GetMaximumUsableSampleCount() const;

	/**
	 * Finds the optimal depth format.
	 * @return The optimal depth format.
	 */
	[[nodiscard]] VkFormat FindDepthFormat() const;

	/**
	 * Returns the VkDevice handle.
	 * @return A reference to the VkDevice handle.
	 */
	[[nodiscard]] VkDevice GetReference() const
	{
		return _device;
	}

	/**
	 * Returns the VkPhysicalDevice handle.
	 * @return A reference to the VkPhysicalDevice handle.
	 */
	[[nodiscard]] VkPhysicalDevice GetPhysicalDeviceHandle() const
	{
		return _physicalDevice;
	}

private:
	struct SupportedPhysicalDeviceFeatures
	{
		bool _barycentricCoordinates = false;
	};

	struct PhysicalDeviceQueryResult
	{
		VkPhysicalDevice _device = VK_NULL_HANDLE;
		SupportedPhysicalDeviceFeatures _supportedFeatures;
	};

	/**
	 * Creates a new Vulkan physical device which will be used as part of the device abstraction.
	 * @return A new Vulkan physical device instance or invalid handle.
	 */
	[[nodiscard]] PhysicalDeviceQueryResult QueryPhysicalDevice() const;

	/**
	 * Creates a new Vulkan logical device handle.
	 * @return A new handle to a Vulkan device.
	 */
	[[nodiscard]] VkDevice CreateLogicalDevice(const SupportedPhysicalDeviceFeatures& features) const;

	/**
	 * Find the queue family indices used for graphics, compute and presentation.
	 * @param physicalDevice A handle to the physical device.
	 * @return The queue family indices used for graphics, compute and presentation.
	 */
	[[nodiscard]] QueueFamilyIndices FindQueueFamiliesInternal(VkPhysicalDevice physicalDevice) const;

	/**
	 * Queries the swapchain details from the device based on the surface properties and capabilities.
	 * @param physicalDevice A handle to the physical device.
	 * @return The swapchain details of the surface.
	 */
	[[nodiscard]] SwapchainSupportDetails QuerySwapchainSupportInternal(VkPhysicalDevice physicalDevice) const;

	/**
	 * Check if the physical device has all extensions.
	 * @param device A Vulkan physical device handle.
	 * @return Whether or not the device contains all extensions.
	 */
	[[nodiscard]] bool DeviceHasExtensions(VkPhysicalDevice device) const;

private:
	VkPhysicalDevice _physicalDevice = VK_NULL_HANDLE;
	VkDevice _device = VK_NULL_HANDLE;

	RenderResourceRef<Instance> _instance;
	RenderResourceRef<RenderSurface> _surface;

	/// Cached physical device properties, we only need to query them once.
	VkPhysicalDeviceProperties _physicalDeviceProperties { };

	/// Cached physical device features, only need to query them once.
	VkPhysicalDeviceFeatures _physicalDeviceFeatures { };

	/// Swapchain details and surface properties. We cache them here, because we only need to query them once.
	SwapchainSupportDetails _swapchainSupportDetails { };

	/// The family indices of the available graphics and present queues.
	QueueFamilyIndices _queueFamilyIndices { };
};
