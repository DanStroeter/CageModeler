#include <Rendering/Core/Instance.h>
#include <Rendering/Utils/VulkanValidation.h>
#include <Rendering/Utils/VulkanUtils.h>
#include <Configuration.h>

Instance::Instance(const std::vector<const char *>& instanceExtensions)
{
	VkApplicationInfo applicationInfo { };
	applicationInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	applicationInfo.pApplicationName = "MeshCageDeformation-App";
	applicationInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
	applicationInfo.pEngineName = "MeshCageDeformation-Engine";
	applicationInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
	applicationInfo.apiVersion = VK_API_VERSION_1_3;

	VkInstanceCreateInfo createInfo { };
	createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;

#if PLATFORM_MAC
	createInfo.flags = VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
#endif

	createInfo.pApplicationInfo = &applicationInfo;
	createInfo.enabledExtensionCount = static_cast<uint32_t>(instanceExtensions.size());
	createInfo.ppEnabledExtensionNames = instanceExtensions.data();

	if constexpr (MCD::EnableVulkanValidation)
	{
		constexpr std::array enabledLayersNames { "VK_LAYER_KHRONOS_validation" };

		createInfo.enabledLayerCount = static_cast<uint32_t>(enabledLayersNames.size());
		createInfo.ppEnabledLayerNames = enabledLayersNames.data();

		const auto debugCreateInfo = VulkanValidation::CreateInfo();
		createInfo.pNext = &debugCreateInfo;
	}
	else
	{
		createInfo.enabledLayerCount = 0;
		createInfo.pNext = nullptr;
	}

	VK_CHECK(vkCreateInstance(&createInfo, nullptr, &_handle));

	LOG_DEBUG("> Instance created.");
}

Instance::~Instance()
{
	vkDestroyInstance(_handle, nullptr);
}
