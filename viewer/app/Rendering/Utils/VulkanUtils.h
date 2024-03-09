#pragma once

#include <fstream>
#include <filesystem>
#include <iostream>
#include <span>
#include <volk.h>
#include <vulkan/vk_enum_string_helper.h>

#include <Logging/Logging.h>

#define VK_CHECK(x)                                      \
do                                                       \
{                                                        \
    VkResult err = x;                                    \
    const auto errString = string_VkResult(err);         \
    if (err != VK_SUCCESS) [[unlikely]]                  \
    {                                                    \
            LOG_ERROR("Vulkan error {}", errString);     \
            abort();                                     \
        }                                                \
    } while (0)

#if BUILD_DEVELOPMENT
	#define CHECK_VK_HANDLE(handle)                      \
		do                                               \
		{                                                \
			if ((handle) == VK_NULL_HANDLE) [[unlikely]] \
			{                                            \
				LOG_ERROR("Vulkan handle is NULL.\n");   \
				abort();                                 \
			}                                            \
		} while (0)
#else
	#define CHECK_VK_HANDLE(handle)
#endif

#if PLATFORM_MAC
	#include <vulkan/vulkan_beta.h>
#endif

	constexpr std::array VulkanDeviceExtensions {
		VK_KHR_SWAPCHAIN_EXTENSION_NAME,
		VK_EXT_EXTENDED_DYNAMIC_STATE_EXTENSION_NAME,
		VK_EXT_EXTENDED_DYNAMIC_STATE_2_EXTENSION_NAME,
		// VK_EXT_EXTENDED_DYNAMIC_STATE_3_EXTENSION_NAME,
		// VK_EXT_LINE_RASTERIZATION_EXTENSION_NAME,

		// On macOS we need to explicitly require the portability subset extension.
#if PLATFORM_MAC
		VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME
#endif
	};

struct VulkanUtils
{

static constexpr uint32_t NumRenderFramesInFlight = 2;

/**
 * Find a supported depth format from the device properties.
 *
 * @param physicalDevice A Vulkan physical device instance.
 * @param candidates All possible format candidates.
 * @param tiling The image tiling.
 * @param features The format feature flags.
 * @return A supported depth format.
 */
[[nodiscard]] static VkFormat FindSupportedDepthFormat(
	const VkPhysicalDevice physicalDevice,
	const std::span<VkFormat> candidates,
	const VkImageTiling tiling,
	const VkFormatFeatureFlags features)
{
	CHECK_VK_HANDLE(physicalDevice);

	for (const auto format : candidates)
	{
		VkFormatProperties props;
		vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &props);

		if (tiling == VK_IMAGE_TILING_LINEAR && (props.linearTilingFeatures & features) == features)
		{
			return format;
		}
		else if (tiling == VK_IMAGE_TILING_OPTIMAL && (props.optimalTilingFeatures & features) == features)
		{
			return format;
		}
	}

	return VK_FORMAT_D32_SFLOAT_S8_UINT;
}

/**
 * Check if a given format contains a stencil component
 * @param format The format.
 * @return Whether or not the format contains a stencil component.
 */
[[nodiscard]] static bool FormatHasStencilComponent(VkFormat format)
{
	return format == VK_FORMAT_D32_SFLOAT_S8_UINT || format == VK_FORMAT_D24_UNORM_S8_UINT;
}

[[nodiscard]] static uint32_t FindMemoryType(VkPhysicalDevice physicalDevice, const uint32_t typeFilter, VkMemoryPropertyFlags properties)
{
	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++)
	{
		if (typeFilter & (1u << i) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
		{
			return i;
		}
	}

	CheckFormat(true, "Could not find memory type in Vulkan.");

	return std::numeric_limits<uint32_t>::max();
}

[[nodiscard]] static uint32_t FindMemoryTypeWithFallback(VkPhysicalDevice physicalDevice, const uint32_t deviceRequirements, const uint32_t hostRequirements)
{
	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

	for (uint32_t i = 0; i < VK_MAX_MEMORY_TYPES; i++)
	{
		if (deviceRequirements & (1u << i) && (memProperties.memoryTypes[i].propertyFlags & hostRequirements) == hostRequirements)
		{
			return i;
		}
	}

	// On desktop systems, we'll need a fallback to plain device local.
	if (hostRequirements & VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT)
	{
		return FindMemoryTypeWithFallback(physicalDevice, deviceRequirements, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	}

	// If we cannot find the particular memory type we're looking for, just pick the first one available.
	if (hostRequirements != 0)
	{
		return FindMemoryTypeWithFallback(physicalDevice, deviceRequirements, 0);
	}
	else
	{
		CheckFormat(true, "Could not find memory type in Vulkan.");
	}

	return std::numeric_limits<uint32_t>::max();
}

/**
 * Begins a one-time Vulkan command to be executed immediately on the GPU.
 * @param device A Vulkan device reference.
 * @param commandPool A command pool to schedule the command on.
 * @return A one-time Vulkan command to be executed immediately on the GPU.
 */
[[nodiscard]] static VkCommandBuffer BeginOneTimeCommandBuffer(const VkDevice device, const VkCommandPool commandPool)
{
	CHECK_VK_HANDLE(device);
	CHECK_VK_HANDLE(commandPool);

	VkCommandBufferAllocateInfo allocateInfo { };
	allocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocateInfo.commandPool = commandPool;
	allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocateInfo.commandBufferCount = 1;

	VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
	VK_CHECK(vkAllocateCommandBuffers(device, &allocateInfo, &commandBuffer));

	VkCommandBufferBeginInfo beginInfo { };
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	vkBeginCommandBuffer(commandBuffer, &beginInfo);

	return commandBuffer;
}

/**
 * Ends the one-time command and submits it to the given queue immediately while waiting for it to finish. This
 * call will block the CPU waiting for it to complete.
 * @param device A Vulkan device reference.
 * @param commandPool A command pool that the command was scheduled on.
 * @param commandBuffer A command buffer to record the command on.
 * @param submitQueue A queue to submit the command on.
 */
static void EndOneTimeCommandBuffer(const VkDevice device, const VkCommandPool commandPool, const VkCommandBuffer commandBuffer, const VkQueue submitQueue)
{
	vkEndCommandBuffer(commandBuffer);

	VkSubmitInfo submitInfo { };
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &commandBuffer;

	vkQueueSubmit(submitQueue, 1, &submitInfo, VK_NULL_HANDLE);
	vkQueueWaitIdle(submitQueue);

	vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
}

/**
 * Reads a binary file into a vector of bytes.
 * @param filename The filepath.
 * @return A vectory of bytes from the file that was read in.
 */
[[nodiscard]] static std::vector<char> ReadBinaryFile(const std::string& filename)
{
	std::ifstream file(filename, std::ios::ate | std::ios::binary);

	if (!file.is_open())
	{
		LOG_ERROR("Unable to open binary file for read.");

		return { };
	}

	const auto fileSize = file.tellg();
	std::vector<char> buffer(static_cast<std::size_t>(fileSize));

	file.seekg(0);
	file.read(buffer.data(), fileSize);
	file.close();

	return buffer;
}

/**
 * Calculates the swapchain extent from the surface capabilities so that it fits in the min/max limits.
 * @param drawableSize The window drawable size.
 * @param capabilities The surface capabilities.
 * @return The swapchain extent.
 */
[[nodiscard]] static VkExtent2D CalculateSwapchainExtent(const VkExtent2D drawableSize, const VkSurfaceCapabilitiesKHR& capabilities)
{
	if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max())
	{
		return capabilities.currentExtent;
	}
	else
	{
		return VkExtent2D { std::clamp(drawableSize.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width),
							std::clamp(drawableSize.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height) };
	}
}

/**
 * Returns the optimal surface format from an array of formats or the first available one.
 * @param availableFormats An array of surface formats.
 * @return The optimal surface format.
 */
[[nodiscard]] static VkSurfaceFormatKHR GetOptimalSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats)
{
	for (const auto& format : availableFormats)
	{
		if (format.format == VK_FORMAT_B8G8R8A8_UNORM && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
		{
			return format;
		}
	}

	return availableFormats[0];
}

/**
 * Returns the optimal present mode for the application or VK_PRESENT_MODE_FIFO_KHR if none is found.
 * @param availablePresentModes All available present modes.
 * @return The optimal present mode.
 */
[[nodiscard]] static VkPresentModeKHR GetOptimalPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes)
{
	for (const auto& availablePresentMode : availablePresentModes)
	{
		if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR)
		{
			return availablePresentMode;
		}
	}

	return VK_PRESENT_MODE_FIFO_KHR;
}

}; // namespace VulkanUtils
