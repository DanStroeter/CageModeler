#pragma once

#include <Rendering/Core/Device.h>

#include <volk.h>

class DescriptorSetLayout : public RenderResource<DescriptorSetLayout>
{
public:
	DescriptorSetLayout(const RenderResourceRef<Device>& device,
		const std::span<VkDescriptorSetLayoutBinding> layoutBindings);

	[[nodiscard]] VkDescriptorSetLayout GetReference() const
	{
		return _handle;
	}

	void AddRef()
	{
		// Nothing to do here.
	}

	void Release()
	{
		vkDestroyDescriptorSetLayout(_device, _handle, nullptr);
	}

private:
	RenderResourceRef<Device> _device;
	VkDescriptorSetLayout _handle = VK_NULL_HANDLE;
};
