#include <Rendering/Core/RenderResourceManager.h>

RenderResourceManager::RenderResourceManager(const RenderResourceRef<Device>& device)
	: _device(device)
{
}

RenderResourceManager::~RenderResourceManager()
{
}

Image RenderResourceManager::CreateImage(const uint32_t width,
								   const uint32_t height,
								   const VkFormat format,
								   const VkImageTiling tiling,
								   const VkSampleCountFlagBits sampleCountFlags,
								   const VkImageUsageFlags bufferUsage) const
{
	CHECK_VK_HANDLE(_device);

	VkImageCreateInfo createInfo { };
	createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	createInfo.imageType = VK_IMAGE_TYPE_2D;
	createInfo.format = format;
	createInfo.extent.width = width;
	createInfo.extent.height = height;
	createInfo.extent.depth = 1;
	createInfo.mipLevels = 1;
	createInfo.arrayLayers = 1;
	createInfo.samples = sampleCountFlags;
	createInfo.tiling = tiling;
	createInfo.usage = bufferUsage;
	createInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	createInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	VkImage image = VK_NULL_HANDLE;
	VK_CHECK(vkCreateImage(_device, &createInfo, nullptr, &image));

	VkMemoryRequirements memoryRequirements;
	vkGetImageMemoryRequirements(_device, image, &memoryRequirements);

	VkMemoryAllocateInfo allocateInfo { };
	allocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocateInfo.allocationSize = memoryRequirements.size;

	if (bufferUsage & VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT)
	{
		// If an image is transient, try to find lazily allocated memory.
		allocateInfo.memoryTypeIndex = VulkanUtils::FindMemoryTypeWithFallback(
			_device->GetPhysicalDeviceHandle(),
			memoryRequirements.memoryTypeBits,
			VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT | VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	}
	else
	{
		// If a device local memory type exists, we should use that.
		allocateInfo.memoryTypeIndex = VulkanUtils::FindMemoryTypeWithFallback(
			_device->GetPhysicalDeviceHandle(),
			memoryRequirements.memoryTypeBits,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	}

	VkDeviceMemory imageMemory = VK_NULL_HANDLE;
	VK_CHECK(vkAllocateMemory(_device, &allocateInfo, nullptr, &imageMemory));
	VK_CHECK(vkBindImageMemory(_device, image, imageMemory, 0));

	return Image(image, imageMemory);
}

Buffer RenderResourceManager::AllocateDeviceBuffer(const VkDeviceSize deviceSize,
											 const VkBufferUsageFlags bufferUsage,
											 const VkMemoryPropertyFlags properties) const
{
	CHECK_VK_HANDLE(_device);

	VkBufferCreateInfo bufferInfo { };
	bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferInfo.size = deviceSize;
	bufferInfo.usage = bufferUsage;
	bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

	VkBuffer buffer = VK_NULL_HANDLE;
	VK_CHECK(vkCreateBuffer(_device, &bufferInfo, nullptr, &buffer));

	VkMemoryRequirements memRequirements;
	vkGetBufferMemoryRequirements(_device, buffer, &memRequirements);

	VkMemoryAllocateInfo allocateInfo { };
	allocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocateInfo.allocationSize = memRequirements.size;
	allocateInfo.memoryTypeIndex = VulkanUtils::FindMemoryType(_device->GetPhysicalDeviceHandle(),
	                                                           memRequirements.memoryTypeBits,
	                                                           properties);

	VkDeviceMemory deviceMemory = VK_NULL_HANDLE;
	VK_CHECK(vkAllocateMemory(_device, &allocateInfo, nullptr, &deviceMemory));

	// Bind the memory to the buffer object.
	vkBindBufferMemory(_device, buffer, deviceMemory, 0);

	return Buffer(buffer, deviceMemory, deviceSize);
}

VkImageView RenderResourceManager::CreateImageView(const Image& image, const VkFormat format, const VkImageAspectFlags aspectFlags) const
{
	CHECK_VK_HANDLE(_device);

	VkImageViewCreateInfo createInfo { };
	createInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	createInfo.image = image._deviceBuffer;
	createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
	createInfo.format = format;
	createInfo.subresourceRange.aspectMask = aspectFlags;
	createInfo.subresourceRange.baseMipLevel = 0;
	createInfo.subresourceRange.levelCount = 1;
	createInfo.subresourceRange.baseArrayLayer = 0;
	createInfo.subresourceRange.layerCount = 1;

	VkImageView imageView = VK_NULL_HANDLE;
	VK_CHECK(vkCreateImageView(_device, &createInfo, nullptr, &imageView));

	return imageView;
}

VkSampler RenderResourceManager::CreateTextureSampler() const
{
	CHECK_VK_HANDLE(_device);

	VkSamplerCreateInfo createInfo { };
	createInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
	createInfo.magFilter = VK_FILTER_LINEAR;
	createInfo.minFilter = VK_FILTER_LINEAR;
	createInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
	createInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	createInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	createInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	createInfo.mipLodBias = 0.0f;

	createInfo.anisotropyEnable = VK_TRUE;
	createInfo.maxAnisotropy = _device->GetPhysicalDeviceProperties().limits.maxSamplerAnisotropy;
	createInfo.compareEnable = VK_FALSE;
	createInfo.compareOp = VK_COMPARE_OP_ALWAYS;
	createInfo.minLod = 0.0f;
	createInfo.maxLod = 0.0f;
	createInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
	createInfo.unnormalizedCoordinates = VK_FALSE;

	VkSampler sampler = VK_NULL_HANDLE;
	VK_CHECK(vkCreateSampler(_device, &createInfo, nullptr, &sampler));

	return sampler;
}
