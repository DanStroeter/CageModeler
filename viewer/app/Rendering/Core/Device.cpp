#include <set>

#include <Rendering/Core/Device.h>
#include <Rendering/Core/Synchronization.h>
#include <Rendering/Utils/VulkanUtils.h>
#include <Rendering/Utils/VulkanValidation.h>
#include <Configuration.h>

namespace
{
	template <typename Main, typename New>
	void PushFeaturePointerToChainNext(Main* mainStruct, New* newStruct)
	{
		struct VkAnyStruct
		{
			VkStructureType sType;
			void* pNext;
		};

		VkAnyStruct* lastStruct = (VkAnyStruct *)(mainStruct);
		while (lastStruct->pNext != nullptr)
		{
			lastStruct = (VkAnyStruct *)(lastStruct->pNext);
		}

		newStruct->pNext = nullptr;
		lastStruct->pNext = newStruct;
	}
}

Device::Device(const RenderResourceRef<Instance>& instance, const RenderResourceRef<RenderSurface>& surface)
	: _instance(instance)
	, _surface(surface)
{
	const auto physicalDeviceResult = QueryPhysicalDevice();
	_physicalDevice = physicalDeviceResult._device;
	_queueFamilyIndices = FindQueueFamiliesInternal(_physicalDevice);
	_swapchainSupportDetails = QuerySwapchainSupportInternal(_physicalDevice);
	_device = CreateLogicalDevice(physicalDeviceResult._supportedFeatures);

	vkGetPhysicalDeviceProperties(_physicalDevice, &_physicalDeviceProperties);
	vkGetPhysicalDeviceFeatures(_physicalDevice, &_physicalDeviceFeatures);

#if BUILD_DEVELOPMENT
	LOG_INFO("Graphics device: {}", _physicalDeviceProperties.deviceName);
	LOG_INFO("Driver version: {}", _physicalDeviceProperties.driverVersion);
#endif
}

void Device::WaitIdle() const
{
	vkDeviceWaitIdle(_device);
}

void Device::WaitForFence(const RenderResourceRef<Fence>& fence) const
{
	const std::array<VkFence, 1> fences { fence };

	vkWaitForFences(_device, static_cast<uint32_t>(fences.size()), fences.data(), VK_TRUE, std::numeric_limits<uint64_t>::max());
}

void Device::ResetFence(const RenderResourceRef<Fence>& fence) const
{
	const std::array fences { fence->GetReference() };

	vkResetFences(_device, static_cast<uint32_t>(fences.size()), fences.data());
}

Device::PhysicalDeviceQueryResult Device::QueryPhysicalDevice() const
{
	uint32_t deviceCount = 0;
	vkEnumeratePhysicalDevices(_instance, &deviceCount, nullptr);

	if (deviceCount == 0)
	{
		return PhysicalDeviceQueryResult { };
	}

	std::vector<VkPhysicalDevice> physicalDevices(deviceCount);
	vkEnumeratePhysicalDevices(_instance, &deviceCount, physicalDevices.data());

	const auto found = std::ranges::find_if(physicalDevices, [this](const auto physicalDevice) {
		const auto queueFamilyIndices = FindQueueFamiliesInternal(physicalDevice);

		if (!queueFamilyIndices.IsComplete())
		{
			return false;
		}

		// Get the device properties.
		VkPhysicalDeviceProperties deviceProperties;
		vkGetPhysicalDeviceProperties(physicalDevice, &deviceProperties);

		// Get the device features.
		VkPhysicalDeviceFeatures deviceFeatures;
		vkGetPhysicalDeviceFeatures(physicalDevice, &deviceFeatures);

		if (!deviceFeatures.samplerAnisotropy)
		{
			return false;
		}

		// Check if the device has all extensions requested.
		const auto hasExtensions = DeviceHasExtensions(physicalDevice);
		if (!hasExtensions)
		{
			return false;
		}

		// Get the swapchain details.
		const auto swapchainDetails = QuerySwapchainSupportInternal(physicalDevice);

		return !swapchainDetails._surfaceFormats.empty() && !swapchainDetails._presentModes.empty();
	});
	Check(found != physicalDevices.end());

	const VkPhysicalDevice foundPhysicalDevice = *found;
	CHECK_VK_HANDLE(foundPhysicalDevice);

	// Require that our device supports barycentric coordinate sampling, otherwise we cannot draw wireframes.
	VkPhysicalDeviceFragmentShaderBarycentricFeaturesKHR barycentricFeatures { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FRAGMENT_SHADER_BARYCENTRIC_FEATURES_KHR };
	VkPhysicalDeviceFeatures2 physicalFeatures { };
	physicalFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
	physicalFeatures.pNext = &barycentricFeatures;

	vkGetPhysicalDeviceFeatures2(foundPhysicalDevice, &physicalFeatures);

	PhysicalDeviceQueryResult result;
	result._device = foundPhysicalDevice;
	result._supportedFeatures._barycentricCoordinates = (barycentricFeatures.fragmentShaderBarycentric == VK_TRUE);

	return result;
}

VkDevice Device::CreateLogicalDevice(const SupportedPhysicalDeviceFeatures& features) const
{
	std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
	queueCreateInfos.reserve(2);

	std::set uniqueQueueFamilies = { _queueFamilyIndices._graphics.value(), _queueFamilyIndices._present.value() };

	float queuePriority = 1.0f;
	for (const auto queueFamily : uniqueQueueFamilies)
	{
		VkDeviceQueueCreateInfo queueCreateInfo{};
		queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		queueCreateInfo.queueFamilyIndex = queueFamily;
		queueCreateInfo.queueCount = 1;
		queueCreateInfo.pQueuePriorities = &queuePriority;
		queueCreateInfos.push_back(queueCreateInfo);
	}

	std::vector<const char*> enabledExtensions;
	enabledExtensions.reserve(4);
	std::copy(VulkanDeviceExtensions.begin(), VulkanDeviceExtensions.end(), std::back_inserter(enabledExtensions));

	VkPhysicalDeviceFeatures2 deviceFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
	deviceFeatures.features.samplerAnisotropy = VK_TRUE;
	deviceFeatures.features.sampleRateShading = VK_TRUE;
	deviceFeatures.features.largePoints = VK_TRUE;

	VkPhysicalDeviceFragmentShaderBarycentricFeaturesKHR barycentricFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FRAGMENT_SHADER_BARYCENTRIC_FEATURES_KHR };
	if (features._barycentricCoordinates)
	{
		barycentricFeatures.fragmentShaderBarycentric = VK_TRUE;
		PushFeaturePointerToChainNext(&deviceFeatures, &barycentricFeatures);

		enabledExtensions.push_back(VK_KHR_FRAGMENT_SHADER_BARYCENTRIC_EXTENSION_NAME);
	}

	VkDeviceCreateInfo createInfo { };
	createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	createInfo.pNext = &deviceFeatures;
	createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
	createInfo.pQueueCreateInfos = queueCreateInfos.data();
	createInfo.enabledExtensionCount = static_cast<uint32_t>(enabledExtensions.size());
	createInfo.ppEnabledExtensionNames = enabledExtensions.data();
	createInfo.enabledLayerCount = 0;

	if constexpr (MCD::EnableVulkanValidation)
	{
		createInfo.enabledLayerCount = static_cast<uint32_t>(VulkanValidationLayers.size());
		createInfo.ppEnabledLayerNames = VulkanValidationLayers.data();
	}

	VkDevice device = VK_NULL_HANDLE;
	VK_CHECK(vkCreateDevice(_physicalDevice, &createInfo, nullptr, &device));

	return device;
}

VkSampleCountFlagBits Device::GetMaximumUsableSampleCount() const
{
	const auto counts = _physicalDeviceProperties.limits.framebufferColorSampleCounts & _physicalDeviceProperties.limits.framebufferDepthSampleCounts;
	if (counts & VK_SAMPLE_COUNT_64_BIT)
	{
		return VK_SAMPLE_COUNT_64_BIT;
	}
	if (counts & VK_SAMPLE_COUNT_32_BIT)
	{
		return VK_SAMPLE_COUNT_32_BIT;
	}
	if (counts & VK_SAMPLE_COUNT_16_BIT)
	{
		return VK_SAMPLE_COUNT_16_BIT;
	}
	if (counts & VK_SAMPLE_COUNT_8_BIT)
	{
		return VK_SAMPLE_COUNT_8_BIT;
	}
	if (counts & VK_SAMPLE_COUNT_4_BIT)
	{
		return VK_SAMPLE_COUNT_4_BIT;
	}
	if (counts & VK_SAMPLE_COUNT_2_BIT)
	{
		return VK_SAMPLE_COUNT_2_BIT;
	}

	return VK_SAMPLE_COUNT_1_BIT;
}

bool Device::DeviceHasExtensions(const VkPhysicalDevice device) const
{
	CHECK_VK_HANDLE(_surface);

	uint32_t extensionCount;
	vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);

	std::vector<VkExtensionProperties> availableExtensions(extensionCount);
	vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

	std::set<std::string_view> requiredExtensions(VulkanDeviceExtensions.begin(), VulkanDeviceExtensions.end());

	for (const auto& extension : availableExtensions)
	{
		requiredExtensions.erase(extension.extensionName);
	}

	const auto hasExtensions = requiredExtensions.empty();

	return hasExtensions;
}

QueueFamilyIndices Device::FindQueueFamiliesInternal(VkPhysicalDevice physicalDevice) const
{
	CHECK_VK_HANDLE(_surface);
	CHECK_VK_HANDLE(physicalDevice);

	uint32_t queueFamilyCount = 0;
	vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, nullptr);

	std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
	vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, queueFamilies.data());

	// Find the queue index that can be used for graphics.
	std::optional<uint32_t> graphicsFamilyIndex { };
	for (std::size_t index = 0; index < queueFamilyCount; ++index)
	{
		if (queueFamilies[index].queueFlags & VK_QUEUE_GRAPHICS_BIT)
		{
			graphicsFamilyIndex = static_cast<uint32_t>(index);

			break;
		}
	}

	// Find the queue index that can be used for graphics.
	std::optional<uint32_t> computeFamilyIndex { };
	for (std::size_t index = 0; index < queueFamilyCount; ++index)
	{
		if (queueFamilies[index].queueFlags & VK_QUEUE_COMPUTE_BIT)
		{
			computeFamilyIndex = static_cast<uint32_t>(index);

			break;
		}
	}

	// Find the queue index that can present the surface.
	std::optional<uint32_t> presentFamilyIndex { };
	for (std::size_t index = 0; index < queueFamilyCount; ++index)
	{
		VkBool32 presentSupport = false;
		vkGetPhysicalDeviceSurfaceSupportKHR(physicalDevice, static_cast<uint32_t>(index), _surface, &presentSupport);

		if (presentSupport)
		{
			presentFamilyIndex = static_cast<uint32_t>(index);

			break;
		}
	}

	return QueueFamilyIndices { graphicsFamilyIndex, computeFamilyIndex, presentFamilyIndex };
}

VkQueue Device::GetDeviceQueue(const uint32_t queueFamilyIndex, const uint32_t queueIndex) const
{
	VkQueue queue = VK_NULL_HANDLE;
	vkGetDeviceQueue(_device, queueFamilyIndex, queueIndex, &queue);

	return queue;
}

VkFormat Device::FindDepthFormat() const
{
	CHECK_VK_HANDLE(_physicalDevice);

	static std::array PreferredDepthFormats {
		VK_FORMAT_D24_UNORM_S8_UINT,
		VK_FORMAT_D32_SFLOAT_S8_UINT,
		VK_FORMAT_D32_SFLOAT
	};

	return VulkanUtils::FindSupportedDepthFormat(
		_physicalDevice,
		std::span(PreferredDepthFormats),
		VK_IMAGE_TILING_OPTIMAL,
		VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT
	);
}

SwapchainSupportDetails Device::QuerySwapchainSupportInternal(const VkPhysicalDevice physicalDevice) const
{
	CHECK_VK_HANDLE(_surface);
	CHECK_VK_HANDLE(physicalDevice);

	SwapchainSupportDetails supportDetails;

	vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, _surface, &supportDetails._surfaceCapabilities);

	uint32_t formatCount = 0;
	vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, _surface, &formatCount, nullptr);

	if (formatCount > 0)
	{
		supportDetails._surfaceFormats.resize(formatCount);
		vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, _surface, &formatCount, supportDetails._surfaceFormats.data());
	}

	uint32_t presentModeCount = 0;
	vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, _surface, &presentModeCount, nullptr);

	if (presentModeCount > 0)
	{
		supportDetails._presentModes.resize(presentModeCount);
		vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, _surface, &presentModeCount, supportDetails._presentModes.data());
	}

	return supportDetails;
}
