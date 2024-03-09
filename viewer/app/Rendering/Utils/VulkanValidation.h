#pragma once

#include <volk.h>
#include <algorithm>

#include <Configuration.h>
#include <Logging/Logging.h>

constexpr std::array VulkanValidationLayers = {
	"VK_LAYER_KHRONOS_validation"
};

namespace VulkanValidation
{

inline std::string VulkanMessageSeverityFlagsToString(const VkDebugUtilsMessageSeverityFlagsEXT value)
{
	if (!value)
	{
		return "{}";
	}

	std::string result;
	if (value & VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT)
	{
		result += "Verbose | ";
	}
	if (value & VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT)
	{
		result += "Info | ";
	}
	if (value & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
	{
		result += "Warning | ";
	}
	if (value & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
	{
		result += "Error | ";
	}

	return "{ " + result.substr(0, result.size() - 3) + " }";
}

inline std::string VulkanMessageTypeFlagsToString(const VkDebugUtilsMessageTypeFlagsEXT value)
{
	if (!value)
	{
		return "{}";
	}

	std::string result;
	if (value & VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT)
	{
		result += "General | ";
	}
	if (value & VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT)
	{
		result += "Validation | ";
	}
	if (value & VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT)
	{
		result += "Performance | ";
	}
	if (value & VK_DEBUG_UTILS_MESSAGE_TYPE_DEVICE_ADDRESS_BINDING_BIT_EXT)
	{
		result += "DeviceAddressBinding | ";
	}

	return "{ " + result.substr(0, result.size() - 3) + " }";
}

inline VKAPI_ATTR VkBool32 VKAPI_CALL ValidationDebugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
															  VkDebugUtilsMessageTypeFlagsEXT messageType,
															  const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
															  [[maybe_unused]] void* pUserData)
{
	std::string message;

	// Format the message output to print out the data.
	message += VulkanMessageSeverityFlagsToString(messageSeverity) + ": " + VulkanMessageTypeFlagsToString(messageType) + ":\n";
	message += std::string("\t") + "messageIDName   = <" + pCallbackData->pMessageIdName + ">\n";
	message += std::string("\t") + "messageIdNumber = " + std::to_string(pCallbackData->messageIdNumber) + "\n";
	message += std::string("\t") + "message         = <" + pCallbackData->pMessage + ">\n";

	if (pCallbackData->queueLabelCount > 0)
	{
		message += std::string("\t") + "Queue Labels:\n";

		for (uint32_t i = 0; i < pCallbackData->queueLabelCount; i++)
		{
			message += std::string("\t\t") + "labelName = <" + pCallbackData->pQueueLabels[i].pLabelName + ">\n";
		}
	}
	if (pCallbackData->cmdBufLabelCount > 0)
	{
		message += std::string("\t") + "CommandBuffer Labels:\n";
		for (uint32_t i = 0; i < pCallbackData->cmdBufLabelCount; i++)
		{
			message += std::string("\t\t") + "labelName = <" + pCallbackData->pCmdBufLabels[i].pLabelName + ">\n";
		}
	}
	if (pCallbackData->objectCount > 0)
	{
		for (uint32_t i = 0; i < pCallbackData->objectCount; i++)
		{
			message += std::string("\t") + "Object " + std::to_string(i) + "\n";
			message += std::string("\t\t") + "objectHandle = " +
					   std::to_string(pCallbackData->pObjects[i].objectHandle) + "\n";

			if (pCallbackData->pObjects[i].pObjectName)
			{
				message += std::string("\t\t") + "objectName   = <" + pCallbackData->pObjects[i].pObjectName + ">\n";
			}
		}
	}

	LOG_INFO(message);

	return false;
}

inline VkResult CreateDebugUtilsMessengerEXT(VkInstance instance,
											 const VkDebugUtilsMessengerCreateInfoEXT *pCreateInfo,
											 const VkAllocationCallbacks *pAllocator,
											 VkDebugUtilsMessengerEXT* pDebugMessenger)
{
	auto func = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT"));

	if (func != nullptr)
	{
		return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
	}
	else
	{
		return VK_ERROR_EXTENSION_NOT_PRESENT;
	}
}

inline void DestroyDebugUtilsMessengerEXT(VkInstance instance,
										  VkDebugUtilsMessengerEXT debugMessenger,
										  const VkAllocationCallbacks *pAllocator)
{
	auto func = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT"));

	if (func != nullptr)
	{
		func(instance, debugMessenger, pAllocator);
	}
}

inline bool CheckInstanceValidationLayersSupport()
{
	uint32_t layerCount = 0;
	vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

	std::vector<VkLayerProperties> availableLayers(layerCount);
	vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

	return std::ranges::all_of(VulkanValidationLayers,
							   [&availableLayers](char const *name) {
								   return std::ranges::find_if(availableLayers,
															   [&name](const auto &property) {
																   return strcmp(property.layerName, name) == 0;
															   }) != availableLayers.end();
							   });
}

inline VkDebugUtilsMessengerCreateInfoEXT CreateInfo()
{
	VkDebugUtilsMessengerCreateInfoEXT createInfo { };
	createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
	createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
								 VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
	createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
							 VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
							 VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
	createInfo.pfnUserCallback = ValidationDebugCallback;
	createInfo.pUserData = nullptr;

	return createInfo;
}

} // namespace VulkanValidation