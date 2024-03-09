#pragma once

#include <Rendering/Core/RenderResource.h>

#include <span>
#include <volk.h>

class Instance : public RenderResource<Instance>
{
public:
	explicit Instance(const std::vector<const char*>& instanceExtensions);
	~Instance();

	[[nodiscard]] VkInstance GetReference() const
	{
		return _handle;
	}

	void AddRef()
	{
		// Nothing to do here.
	}

	void Release()
	{
		vkDestroyInstance(_handle, nullptr);
	}

private:
	VkInstance _handle = VK_NULL_HANDLE;
};
