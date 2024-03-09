#include <Rendering/Core/DescriptorPool.h>
#include <Rendering/Utils/VulkanUtils.h>

DescriptorPool::DescriptorPool(const RenderResourceRef<Device>& device)
	: _device(device)
{
	std::array<VkDescriptorPoolSize, 5> poolSizes { };

	poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	poolSizes[0].descriptorCount = 10 * VulkanUtils::NumRenderFramesInFlight;

	poolSizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	poolSizes[1].descriptorCount = 10 * VulkanUtils::NumRenderFramesInFlight;

	poolSizes[2].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	poolSizes[2].descriptorCount = 10 * VulkanUtils::NumRenderFramesInFlight;

	poolSizes[3].type = VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
	poolSizes[3].descriptorCount = 10 * VulkanUtils::NumRenderFramesInFlight;

	poolSizes[4].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
	poolSizes[4].descriptorCount = 1 * VulkanUtils::NumRenderFramesInFlight;

	VkDescriptorPoolCreateInfo poolInfo { };
	poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	poolInfo.maxSets = 10 * VulkanUtils::NumRenderFramesInFlight * poolSizes.size();
	poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
	poolInfo.pPoolSizes = poolSizes.data();

	VK_CHECK(vkCreateDescriptorPool(_device, &poolInfo, nullptr, &_descriptorPool));
}

void DescriptorPool::Release()
{
	vkDestroyDescriptorPool(_device, _descriptorPool, nullptr);
}

std::vector<VkDescriptorSet> DescriptorPool::AllocateDescriptorSets(const std::span<VkDescriptorSetLayout> descriptorSetLayouts) const
{
	VkDescriptorSetAllocateInfo allocateInfo { };
	allocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocateInfo.descriptorPool = _descriptorPool;
	allocateInfo.descriptorSetCount = static_cast<uint32_t>(descriptorSetLayouts.size());
	allocateInfo.pSetLayouts = descriptorSetLayouts.data();

	std::vector<VkDescriptorSet> descriptorSets(descriptorSetLayouts.size(), VK_NULL_HANDLE);
	VK_CHECK(vkAllocateDescriptorSets(_device, &allocateInfo, descriptorSets.data()));

	return descriptorSets;
}

RenderResourceRef<DescriptorSetLayout> DescriptorPool::CreateDescriptorSetLayout(const std::span<VkDescriptorSetLayoutBinding> layoutBindings) const
{
	return CreateRenderResource<DescriptorSetLayout>(_device, layoutBindings);
}
