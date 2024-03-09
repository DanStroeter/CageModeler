#pragma once

#include <Rendering/Core/RenderResource.h>
#include <Rendering/Core/Instance.h>

#include <volk.h>
#include <limits>
#include <vector>

class Instance;

struct SwapchainSupportDetails
{
	/// Cached surface capabilities from the physical device.
	VkSurfaceCapabilitiesKHR _surfaceCapabilities { };

	/// Cached surface formats.
	std::vector<VkSurfaceFormatKHR> _surfaceFormats;

	/// Cached present modes.
	std::vector<VkPresentModeKHR> _presentModes;
};

class RenderSurface : public RenderResource<RenderSurface>
{
public:
	RenderSurface(VkSurfaceKHR surface, const RenderResourceRef<Instance>& instance);

	[[nodiscard]] VkSurfaceKHR GetReference() const
	{
		return _handle;
	}

	void AddRef()
	{
		// Nothing to do here.
	}

	void Release()
	{
		vkDestroySurfaceKHR(_instance->GetReference(), _handle, nullptr);
	}

private:
	VkSurfaceKHR _handle = VK_NULL_HANDLE;

	RenderResourceRef<Instance> _instance;
};
