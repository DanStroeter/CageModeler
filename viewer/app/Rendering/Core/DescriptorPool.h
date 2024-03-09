#pragma once

#include <Rendering/Core/Device.h>
#include <Rendering/Core/DescriptorSetLayout.h>

#include <span>
#include <volk.h>

/// A wrapper around the VkDescriptorPool handle that exposes useful functionality and abstracts away some of the complexity.
class DescriptorPool : public RenderResource<DescriptorPool>
{
public:
	DescriptorPool(const RenderResourceRef<Device>& device);

	void AddRef()
	{
		// Nothing to do here.
	}

	void Release();

	/**
	 * Allocates a vector of descriptor sets from the pool. The result will contain an amount of descriptor sets
	 * that is equal to the size of the input set layouts.
	 * @param descriptorSetLayouts The descriptor set layots that will be used for the sets.
	 * @return A vector of descriptor sets.
	 */
	[[nodiscard]] std::vector<VkDescriptorSet> AllocateDescriptorSets(const std::span<VkDescriptorSetLayout> descriptorSetLayouts) const;

	/**
	 * Creates a new descriptor set layout.
	 * @return A new descriptor set layout.
	 */
	[[nodiscard]] RenderResourceRef<DescriptorSetLayout> CreateDescriptorSetLayout(const std::span<VkDescriptorSetLayoutBinding> layoutBindings) const;

	/**
	 * Returns the VkDescriptorPool handle.
	 * @return A reference to the VkDescriptorPool handle.
	 */
	[[nodiscard]] VkDescriptorPool GetReference() const
	{
		return _descriptorPool;
	}

private:
	RenderResourceRef<Device> _device;

	/// A VkDescriptorPool object handle.
	VkDescriptorPool _descriptorPool = VK_NULL_HANDLE;
};
