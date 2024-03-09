#include <Rendering/Core/DescriptorSetLayout.h>

DescriptorSetLayout::DescriptorSetLayout(const RenderResourceRef<Device>& device,
	const std::span<VkDescriptorSetLayoutBinding> layoutBindings)
	: _device(device)
{
	VkDescriptorSetLayoutCreateInfo layoutInfo { };
	layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	layoutInfo.bindingCount = static_cast<uint32_t>(layoutBindings.size());
	layoutInfo.pBindings = layoutBindings.data();

	VK_CHECK(vkCreateDescriptorSetLayout(_device, &layoutInfo, nullptr, &_handle));
}
