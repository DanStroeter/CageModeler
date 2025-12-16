#include <Rendering/Cubemaps/CubemapRenderer.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/ScreenPass.h>
#include <Editor/Light.h>
#include <cstddef>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../../external/stb_image_write.h"

CubemapRenderer::CubemapRenderer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
	const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance) : _renderPipelineManager(renderPipelineManager),
	_resourceManager(resourceManager), _device(device), _instance(instance)
{
	//TODO alignment??
}

void CubemapRenderer::Initialize()
{
	_descriptorPool = CreateRenderResource<DescriptorPool>(_device);
	CreateImageViews(512, VK_FORMAT_R32G32B32A32_SFLOAT);
	CreateRenderPass(VK_FORMAT_R32G32B32A32_SFLOAT);
	CreateDescriptorSetLayouts();
	CreateCubemapRenderPipeline();

	CreateDepthImage(512);
	CreateFramebuffer(512);

	uint32_t graphicsQueueFamilyIndex = _device->GetQueueFamilies()._graphics.value();
	CreateCommandPool(graphicsQueueFamilyIndex);
	VkDeviceSize uboSize = sizeof(CubemapMatricesUBO);
	CreateUniformBuffer(uboSize);

	AllocateMatricesDescriptorSet();
	UpdateMatricesDescriptorSet();

	CreateVertexBufferFromMesh();
	CreateIndexBufferFromMesh();
	CreateCommandBuffer();

	CreateSyncObjects();

	CreateComputeCommandPool(graphicsQueueFamilyIndex);
	CreateCubemapImageViews();
	CreateComputeDescriptorSetLayout();
	CreateComputeBuffers();
	AllocateComputeDescriptorSet();
	CreateComputePipeline();
	CreateComputeCommandBuffer();
	CreateSampler();
	//UpdateComputeDescriptorSet();
	SphereWeightInitialization(512);
}

void CubemapRenderer::CreateImageViews(uint32_t size, VkFormat format) {
	//Create image
	VkImageCreateInfo imageInfo{};
	imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	imageInfo.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
	imageInfo.imageType = VK_IMAGE_TYPE_2D;
	imageInfo.format = format;
	imageInfo.extent = { size, size, 1 };
	imageInfo.mipLevels = 1;
	imageInfo.arrayLayers = 6;
	imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	imageInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT 
		| VK_IMAGE_USAGE_TRANSFER_SRC_BIT; //only added for debugging prints for image remove after done
	imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	VkImage cubemapImage = VK_NULL_HANDLE;

	if (vkCreateImage(_device, &imageInfo, nullptr, &cubemapImage) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create Cubemap image!");
	}
	_cubemapImages.push_back(cubemapImage);

	// Allocate and bind memory
	VkMemoryRequirements memRequirements;
	vkGetImageMemoryRequirements(_device, cubemapImage, &memRequirements);

	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.allocationSize = memRequirements.size;

	// Find a suitable memory type
	VkPhysicalDeviceMemoryProperties memProperties;
	VkPhysicalDevice physicalDevice = _device->GetPhysicalDeviceHandle();
	vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

	uint32_t memoryTypeIndex = UINT32_MAX;
	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
		if ((memRequirements.memoryTypeBits & (1 << i)) &&
			(memProperties.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) == VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) {
			memoryTypeIndex = i;
			break;
		}
	}
	if (memoryTypeIndex == UINT32_MAX) {
		throw std::runtime_error("Failed to find suitable memory type for Cubemap image!");
	}
	allocInfo.memoryTypeIndex = memoryTypeIndex;

	VkDeviceMemory cubemapMemory = VK_NULL_HANDLE;
	if (vkAllocateMemory(_device, &allocInfo, nullptr, &cubemapMemory) != VK_SUCCESS) {
		throw std::runtime_error("Failed to allocate memory for Cubemap image!");
	}

	if (vkBindImageMemory(_device, cubemapImage, cubemapMemory, 0) != VK_SUCCESS) {
		throw std::runtime_error("Failed to bind memory to Cubemap image!");
	}
	_cubemapImageMemory.push_back(cubemapMemory);

	std::array<VkImageView, 6> faceImageViews = {};
	// Create 6 image views

	for (uint32_t face = 0; face < 6; ++face) {
		VkImageViewCreateInfo viewInfo{};
		viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		viewInfo.image = cubemapImage;
		viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D; 
		viewInfo.format = format;
		viewInfo.components = {
			VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY,
			VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY
		};
		viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		viewInfo.subresourceRange.baseMipLevel = 0;
		viewInfo.subresourceRange.levelCount = 1;
		viewInfo.subresourceRange.baseArrayLayer = face; 
		viewInfo.subresourceRange.layerCount = 1;       

		VkImageView faceImageView = VK_NULL_HANDLE;
		if (vkCreateImageView(_device, &viewInfo, nullptr, &faceImageView) != VK_SUCCESS) {
			throw std::runtime_error("Failed to create Cubemap face image view!");
		}

		faceImageViews[face] = faceImageView;
	}
	_faceImageViews.push_back(faceImageViews);

	// Create a single VkImageView for the whole Cubemap
	VkImageViewCreateInfo cubeViewInfo{};
	cubeViewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	cubeViewInfo.image = cubemapImage;
	cubeViewInfo.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
	cubeViewInfo.format = format;
	cubeViewInfo.components = {
		VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY,
		VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY
	};
	cubeViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	cubeViewInfo.subresourceRange.baseMipLevel = 0;
	cubeViewInfo.subresourceRange.levelCount = 1;
	cubeViewInfo.subresourceRange.baseArrayLayer = 0;
	cubeViewInfo.subresourceRange.layerCount = 6;

	VkImageView cubemapView = VK_NULL_HANDLE;
	if (vkCreateImageView(_device, &cubeViewInfo, nullptr, &cubemapView) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create Cubemap image view!");
	}
	_cubemapViews.push_back(cubemapView);
}

void CubemapRenderer::CreateRenderPass(VkFormat format) {
	VkAttachmentDescription colorAttachment{};
	colorAttachment.format = format;
	colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT; 
	colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	colorAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	VkAttachmentDescription depthAttachment{};
	depthAttachment.format = _device->FindDepthFormat();
	depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
	depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	VkAttachmentReference colorAttachmentRef{};
	colorAttachmentRef.attachment = 0;
	colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	VkAttachmentReference depthAttachmentRef{};
	depthAttachmentRef.attachment = 1;
	depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	VkSubpassDescription subpass{};
	subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpass.colorAttachmentCount = 1;
	subpass.pColorAttachments = &colorAttachmentRef;
	subpass.pDepthStencilAttachment = &depthAttachmentRef;

	VkSubpassDependency dependency{};
	dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
	dependency.dstSubpass = 0;
	dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
	dependency.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
	dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
	dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

	std::array<VkAttachmentDescription, 2> attachments = { colorAttachment, depthAttachment };
	VkRenderPassCreateInfo renderPassInfo{};
	renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
	renderPassInfo.pAttachments = attachments.data();
	renderPassInfo.subpassCount = 1;
	renderPassInfo.pSubpasses = &subpass;
	renderPassInfo.dependencyCount = 1;
	renderPassInfo.pDependencies = &dependency;

	if (vkCreateRenderPass(_device, &renderPassInfo, nullptr, &_renderPass) != VK_SUCCESS) {
		throw std::runtime_error("failed to create Cubemap render pass!");
	}
}

void CubemapRenderer::CreateDescriptorSetLayouts() {
	VkDescriptorSetLayoutBinding layoutBinding{ };
	layoutBinding.binding = 0;
	layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	layoutBinding.descriptorCount = 1;
	layoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

	std::array<VkDescriptorSetLayoutBinding, 1> layoutBindings{ layoutBinding };

	_matricesLayout = _descriptorPool->CreateDescriptorSetLayout(layoutBindings);
}

void CubemapRenderer::CreateCubemapRenderPipeline()
{
	VkVertexInputBindingDescription bindingDesc{};
	bindingDesc.binding = 0;
	bindingDesc.stride = sizeof(CubemapVertex);
	bindingDesc.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

	std::array<VkVertexInputAttributeDescription, 3> attributeDescs{};
	attributeDescs[0].binding = 0;
	attributeDescs[0].location = 0; //position
	attributeDescs[0].format = VK_FORMAT_R32G32B32_SFLOAT;
	attributeDescs[0].offset = offsetof(CubemapVertex, _position);

	attributeDescs[1].binding = 0;
	attributeDescs[1].location = 1; // triangle ID
	attributeDescs[1].format = VK_FORMAT_R32_UINT;
	attributeDescs[1].offset = offsetof(CubemapVertex, _triangleID);

	attributeDescs[2].binding = 0;
	attributeDescs[2].location = 2; // vertex index
	attributeDescs[2].format = VK_FORMAT_R32_UINT;
	attributeDescs[2].offset = offsetof(CubemapVertex, _vertexIndex);

	// Vertex input state
	VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
	vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
	vertexInputInfo.vertexBindingDescriptionCount = 1;
	vertexInputInfo.pVertexBindingDescriptions = &bindingDesc;
	vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescs.size());
	vertexInputInfo.pVertexAttributeDescriptions = attributeDescs.data();

	// Blend state (disable blending, write all channels)
	VkPipelineColorBlendAttachmentState colorBlendAttachment{};
	colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT |
		VK_COLOR_COMPONENT_G_BIT |
		VK_COLOR_COMPONENT_B_BIT |
		VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachment.blendEnable = VK_FALSE;

	// Depth/stencil state
	VkPipelineDepthStencilStateCreateInfo depthStencil{};
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_TRUE;
	depthStencil.depthWriteEnable = VK_TRUE;
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
	depthStencil.depthBoundsTestEnable = VK_FALSE;
	depthStencil.stencilTestEnable = VK_FALSE;

	// Multisample state 
	VkPipelineMultisampleStateCreateInfo msaa{};
	msaa.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	msaa.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT; 

	// Only the matrices layout is needed for cubemap rendering
	std::array descriptorSetLayouts{ _matricesLayout->GetReference() };

	VkViewport viewport{};
	viewport.x = 0.0f;
	viewport.y = 0.0f;
	viewport.width = (float)512;
	viewport.height = (float)512;
	viewport.minDepth = 0.0f;
	viewport.maxDepth = 1.0f;

	VkRect2D scissor{};
	scissor.offset = { 0, 0 };
	scissor.extent = { 512, 512 };

	// Build the pipeline
	_cubemapPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetColorBlendAttachments(std::span(&colorBlendAttachment, 1))
		.SetDepthStencilState(depthStencil)
		.SetDescriptorSetLayouts(std::span(descriptorSetLayouts))
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Cubemap.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/Cubemap.frag.spv")
		.SetMultisampleState(msaa)
		.SetViewportAndScissor(viewport, scissor)
		.SetVertexInputBindingDescriptions(std::span(&bindingDesc, 1))
		.SetVertexInputAttributeDescriptions(std::span(attributeDescs))
		.Build(); 
}

void CubemapRenderer::CreateDepthImage(uint32_t size) {
	VkFormat depthFormat = _device->FindDepthFormat();

	VkImageCreateInfo depthImageInfo{};
	depthImageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	depthImageInfo.imageType = VK_IMAGE_TYPE_2D;
	depthImageInfo.format = depthFormat;
	depthImageInfo.extent = { size, size, 1 };
	depthImageInfo.mipLevels = 1;
	depthImageInfo.arrayLayers = 6;
	depthImageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	depthImageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	depthImageInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
	depthImageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	depthImageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	if (vkCreateImage(_device, &depthImageInfo, nullptr, &_depthImage) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create depth image!");
	}

	VkMemoryRequirements memRequirements;
	vkGetImageMemoryRequirements(_device, _depthImage, &memRequirements);

	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.allocationSize = memRequirements.size;
	VkPhysicalDeviceMemoryProperties memProperties;
	VkPhysicalDevice physicalDevice = _device->GetPhysicalDeviceHandle();
	vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

	uint32_t memoryTypeIndex = UINT32_MAX;
	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
		if ((memRequirements.memoryTypeBits & (1 << i)) &&
			(memProperties.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) == VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) {
			memoryTypeIndex = i;
			break;
		}
	}
	if (memoryTypeIndex == UINT32_MAX) {
		throw std::runtime_error("Failed to find suitable memory type for Cubemap image!");
	}
	allocInfo.memoryTypeIndex = memoryTypeIndex;

	if (vkAllocateMemory(_device, &allocInfo, nullptr, &_depthImageMemory) != VK_SUCCESS) {
		throw std::runtime_error("Failed to allocate depth image memory!");
	}
	vkBindImageMemory(_device, _depthImage, _depthImageMemory, 0);

	for (uint32_t face = 0; face < 6; ++face) {
		VkImageViewCreateInfo viewInfo{};
		viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		viewInfo.image = _depthImage;
		viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		viewInfo.format = depthFormat;
		viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
		viewInfo.subresourceRange.baseMipLevel = 0;
		viewInfo.subresourceRange.levelCount = 1;
		viewInfo.subresourceRange.baseArrayLayer = face;
		viewInfo.subresourceRange.layerCount = 1;

		VkImageView depthView;
		if (vkCreateImageView(_device, &viewInfo, nullptr, &depthView) != VK_SUCCESS) {
			throw std::runtime_error("Failed to create depth image view!");
		}
		_depthImageViews.push_back(depthView);
	}
}

void CubemapRenderer::CreateFramebuffer(uint32_t size) {
	_faceFramebuffers.resize(6);
	for (uint32_t i = 0; i < 6; ++i)
	{
		VkImageView attachments[2] = {
			_faceImageViews[0][i],
			_depthImageViews[i]     
		};

		VkFramebufferCreateInfo fbInfo{};
		fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		fbInfo.renderPass = _renderPass;
		fbInfo.attachmentCount = 2;
		fbInfo.pAttachments = attachments;
		fbInfo.width = size;
		fbInfo.height = size;
		fbInfo.layers = 1;

		if (vkCreateFramebuffer(_device, &fbInfo, nullptr, &_faceFramebuffers[i]) != VK_SUCCESS) {
			throw std::runtime_error("failed to create cubemap face framebuffer!");
		}
	}
}

void CubemapRenderer::CreateCommandPool(uint32_t queueFamilyIndex) {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_graphicCommandPool) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create command pool!");
	}
}

void CubemapRenderer::CreateComputeCommandPool(uint32_t queueFamilyIndex)
{
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_computeCommandPool) != VK_SUCCESS)
		throw std::runtime_error("Failed to create compute command pool!");
}

void CubemapRenderer::CreateUniformBuffer(VkDeviceSize bufferSize)
{
	std::span<std::byte> sizeSpan(static_cast<std::byte*>(nullptr), bufferSize);

	_matricesUniformBuffer =
		_resourceManager->CreateBufferAndMapMemory(
			sizeSpan,
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		);
}

void CubemapRenderer::CreateVertexBufferFromMesh()
{
	const EigenMesh& geom = _cageMesh;
	const auto& positions = geom._vertices;
	const auto& faces = geom._faces;

	std::vector<CubemapVertex> vertexData;
	vertexData.reserve(faces.rows() * 3);

	for (int tri = 0; tri < faces.rows(); ++tri)
	{
		for (int v = 0; v < 3; ++v)
		{
			int idx = faces(tri, v);

			glm::vec3 pos(
				static_cast<float>(positions(idx, 0)),
				static_cast<float>(positions(idx, 1)),
				static_cast<float>(positions(idx, 2))
			);

			vertexData.push_back({
				pos,
				static_cast<uint32_t>(tri), // triangle ID
				static_cast<uint32_t>(v)    // local vertex ID (0,1,2)
				});
		}
	}

	_vertexBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>((std::byte*)nullptr,
			vertexData.size() * sizeof(CubemapVertex)),
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	memcpy(_vertexBuffer._mappedData, vertexData.data(),
		vertexData.size() * sizeof(CubemapVertex));
}

void CubemapRenderer::CreateIndexBufferFromMesh()
{
	const EigenMesh& geom = _cageMesh;
	const auto& faces = geom._faces;

	// Each triangle has 3 unique vertices in the vertex buffer
	const size_t vertexCount = faces.rows() * 3;

	std::vector<uint32_t> indices(vertexCount);
	for (uint32_t i = 0; i < vertexCount; ++i)
		indices[i] = i;

	_indexBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span(indices),
		VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	memcpy(_indexBuffer._mappedData, indices.data(),
		indices.size() * sizeof(uint32_t));
}

void CubemapRenderer::AllocateMatricesDescriptorSet()
{
	VkDescriptorSetLayout layout = _matricesLayout->GetReference();

	VkDescriptorSetAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocInfo.descriptorPool = _descriptorPool;
	allocInfo.descriptorSetCount = 1;
	allocInfo.pSetLayouts = &layout;

	if (vkAllocateDescriptorSets(_device, &allocInfo, &_matricesDescriptorSet) != VK_SUCCESS)
		throw std::runtime_error("Failed to allocate cubemap matrices descriptor set");
}

void CubemapRenderer::UpdateMatricesDescriptorSet()
{
	VkDescriptorBufferInfo bufferInfo{};
	bufferInfo.buffer = _matricesUniformBuffer._deviceBuffer;
	bufferInfo.offset = 0;
	bufferInfo.range = _matricesUniformBuffer._allocatedSize;

	VkWriteDescriptorSet write{};
	write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
	write.dstSet = _matricesDescriptorSet;
	write.dstBinding = 0;
	write.dstArrayElement = 0;
	write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	write.descriptorCount = 1;
	write.pBufferInfo = &bufferInfo;

	vkUpdateDescriptorSets(_device, 1, &write, 0, nullptr);
}

void CubemapRenderer::CreateCommandBuffer()
{
	// Allocate one primary command buffer per cubemap face
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.commandPool = _graphicCommandPool;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandBufferCount = 6;

	_commandBuffers.resize(6);
	if (vkAllocateCommandBuffers(_device, &allocInfo, _commandBuffers.data()) != VK_SUCCESS) {
		throw std::runtime_error("Failed to allocate cubemap command buffers!");
	}
}

void CubemapRenderer::CreateSyncObjects()
{
	VkFenceCreateInfo fenceInfo{};
	fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceInfo.flags = 0; 

	if (vkCreateFence(_device, &fenceInfo, nullptr, &_renderFence) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create cubemap render fence!");
	}
}

void CubemapRenderer::RenderCubemaps()
{
	LOG_DEBUG("Rendering started!");
	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	VkClearValue clearValues[2];
	clearValues[0].color = { {0.0f, 0.0f, 0.0f, 1.0f} };
	clearValues[1].depthStencil = { 1.0f, 0 };

	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

	const uint32_t numTriangles =
		static_cast<uint32_t>(_cageMesh._faces.size() / 3);

	const float invNumTriangles =
		1.0f / static_cast<float>(numTriangles);


	std::vector<glm::vec3> vertices;
	for (int i = 0; i < _deformableMesh._vertices.rows(); ++i)
	{
		const auto& v = _deformableMesh._vertices.row(i);
		vertices.push_back(glm::vec3(
			static_cast<float>(v(0)),
			static_cast<float>(v(1)),
			static_cast<float>(v(2))
		));
	}

	for (size_t vertexIndex = 0; vertexIndex < vertices.size(); ++vertexIndex)
	{
		const glm::vec3& camPos = vertices[vertexIndex];

		float nearPlane = 0.1f;   
		float farPlane = 1000.0f; //temporary
		//float nearPlane = ComputeNearPlane(camPos, vertices);
		//float farPlane = ComputeFarPlane(camPos, vertices);
		std::string w =
			"Computing cubemap " + std::to_string(vertexIndex + 1) +
			" / " + std::to_string(vertices.size()) +
			" (" + std::to_string(100.0f * (vertexIndex + 1) / vertices.size()) + "%)";

		LOG_DEBUG(w);
		// Render each cubemap face sequentially
		for (int face = 0; face < 6; ++face)
		{
			CubemapMatricesUBO faceUBO{};
			faceUBO.proj = glm::perspective(glm::radians(90.0f), 1.0f, nearPlane, farPlane);
			faceUBO.proj[1][1] *= -1.0f;
			faceUBO.invNumTriangles = invNumTriangles;

			faceUBO.view = ComputeCubemapViewMatrix(face, camPos); // only one view per face
			memcpy(_matricesUniformBuffer._mappedData, &faceUBO, sizeof(faceUBO));

			VkCommandBuffer cmdBuffer = _commandBuffers[face];

			vkResetCommandBuffer(cmdBuffer, 0);
			vkBeginCommandBuffer(cmdBuffer, &beginInfo);

			VkRenderPassBeginInfo renderPassInfo{};
			renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
			renderPassInfo.renderPass = _renderPass;
			renderPassInfo.framebuffer = _faceFramebuffers[face];
			renderPassInfo.renderArea.offset = { 0, 0 };
			renderPassInfo.renderArea.extent = { 512, 512 };
			renderPassInfo.clearValueCount = 2;
			renderPassInfo.pClearValues = clearValues;

			vkCmdBeginRenderPass(cmdBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

			const PipelineObject& pipelineObj = _renderPipelineManager->GetPipelineObject(_cubemapPipelineHandle);
			VkPipeline pipelineHandle = pipelineObj._handle;
			VkPipelineLayout pipelineLayout = pipelineObj._pipelineLayout;

			vkCmdBindPipeline(cmdBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineHandle);

			VkBuffer vertexBuffers[] = { _vertexBuffer._deviceBuffer };
			VkDeviceSize offsets[] = { 0 };
			vkCmdBindVertexBuffers(cmdBuffer, 0, 1, vertexBuffers, offsets);
			vkCmdBindIndexBuffer(cmdBuffer, _indexBuffer._deviceBuffer, 0, VK_INDEX_TYPE_UINT32);

			vkCmdBindDescriptorSets(
				cmdBuffer,
				VK_PIPELINE_BIND_POINT_GRAPHICS,
				pipelineLayout,
				0,
				1,
				&_matricesDescriptorSet,
				0, nullptr
			);

			vkCmdDrawIndexed(cmdBuffer, static_cast<uint32_t>(_cageMesh._faces.size()), 1, 0, 0, 0);

			vkCmdEndRenderPass(cmdBuffer);
			vkEndCommandBuffer(cmdBuffer);

			submitInfo.commandBufferCount = 1;
			submitInfo.pCommandBuffers = &cmdBuffer;
			uint32_t graphicsFamily = _device->GetQueueFamilies()._graphics.value();
			VkQueue graphicsQueue;
			vkGetDeviceQueue(_device, graphicsFamily, 0, &graphicsQueue);
			vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
			vkQueueWaitIdle(graphicsQueue);
			//vkQueueSubmit(graphicsQueue, 1, &submitInfo, _renderFence);
			//vkWaitForFences(_device, 1, &_renderFence, VK_TRUE, UINT64_MAX);
			//vkResetFences(_device, 1, &_renderFence);
		}
		std::string filename = "debug_cubemap_" + std::to_string(vertexIndex) + ".png";
		//ExportCubemapAsVerticalStrip("F:/Cubemaps/" + filename);
		ComputeCoordinates(vertexIndex);
		ReadbackCompute(vertexIndex);
	}
	WriteWeightsToFile("DebugWeights");
}

glm::mat4 CubemapRenderer::ComputeCubemapViewMatrix(uint32_t faceIndex, const glm::vec3& pos)
{
	switch (faceIndex)
	{
	case 0: return glm::lookAt(pos, pos + glm::vec3(1, 0, 0), glm::vec3(0, -1, 0)); // +X
	case 1: return glm::lookAt(pos, pos + glm::vec3(-1, 0, 0), glm::vec3(0, -1, 0)); // -X
	case 2: return glm::lookAt(pos, pos + glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));  // +Y
	case 3: return glm::lookAt(pos, pos + glm::vec3(0, -1, 0), glm::vec3(0, 0, -1)); // -Y
	case 4: return glm::lookAt(pos, pos + glm::vec3(0, 0, 1), glm::vec3(0, -1, 0)); // +Z
	case 5: return glm::lookAt(pos, pos + glm::vec3(0, 0, -1), glm::vec3(0, -1, 0)); // -Z
	default: return glm::mat4(1.0f);
	}
}

float CubemapRenderer::ComputeNearPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices)
{
	float minDist = std::numeric_limits<float>::max();
	for (const auto& v : vertices) {
		float dist = glm::length(v - camPos);
		if (dist < minDist) minDist = dist;
	}
	return minDist * 0.95f;
}

float CubemapRenderer::ComputeFarPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices)
{
	float maxDist = 0.0f;
	for (const auto& v : vertices) {
		float dist = glm::length(v - camPos);
		if (dist > maxDist) maxDist = dist;
	}
	return maxDist * 1.05f;
}

std::vector<CubemapVertex> CubemapRenderer::CreateCubemapVertexBuffer(const PolygonMesh& mesh)
{
	const auto& geom = mesh.GetGeometry();
	const auto& positions = geom._positions;  
	const auto& indices = geom._indices;

	std::vector<CubemapVertex> vertexBuffer;
	vertexBuffer.reserve(indices.size());

	for (size_t tri = 0; tri < indices.size() / 3; ++tri)
	{
		uint32_t i0 = indices[tri * 3 + 0];
		uint32_t i1 = indices[tri * 3 + 1];
		uint32_t i2 = indices[tri * 3 + 2];

		vertexBuffer.push_back({ positions[i0], static_cast<uint32_t>(tri), 0 });
		vertexBuffer.push_back({ positions[i1], static_cast<uint32_t>(tri), 1 });
		vertexBuffer.push_back({ positions[i2], static_cast<uint32_t>(tri), 2 });
	}

	return vertexBuffer;
}

//--------------------------------Debug functions--------------------------------//
uint32_t CubemapRenderer::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties)
{
	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(_device->GetPhysicalDeviceHandle(), &memProperties);

	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
		if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
			return i;
	}
	throw std::runtime_error("Failed to find suitable memory type!");
}

void CubemapRenderer::ExportCubemapAsVerticalStrip(const std::string& filename)
{
	const uint32_t faceSize = 512;
	const uint32_t numFaces = 6;
	const uint32_t channels = 4;
	const VkDeviceSize srcBytesPerTexel = sizeof(float) * channels; 
	const VkDeviceSize imageSizePerFace = faceSize * faceSize * srcBytesPerTexel;
	const VkDeviceSize totalSize = imageSizePerFace * numFaces;

	// --- 1. Create staging buffer ---
	VkBuffer stagingBuffer;
	VkDeviceMemory stagingMemory;

	VkBufferCreateInfo bufferInfo{};
	bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferInfo.size = totalSize;
	bufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
	bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	vkCreateBuffer(_device, &bufferInfo, nullptr, &stagingBuffer);

	VkMemoryRequirements memRequirements;
	vkGetBufferMemoryRequirements(_device, stagingBuffer, &memRequirements);

	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.allocationSize = memRequirements.size;
	allocInfo.memoryTypeIndex = FindMemoryType(
		memRequirements.memoryTypeBits,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	vkAllocateMemory(_device, &allocInfo, nullptr, &stagingMemory);
	vkBindBufferMemory(_device, stagingBuffer, stagingMemory, 0);

	VkCommandBuffer cmdBuffer = BeginOneTimeCommands();

	TransitionImageToTransferSrc(cmdBuffer, _cubemapImages[0]);

	std::vector<VkBufferImageCopy> regions(numFaces);
	for (uint32_t face = 0; face < numFaces; ++face) {
		regions[face].bufferOffset = imageSizePerFace * face;
		regions[face].bufferRowLength = 0;
		regions[face].bufferImageHeight = 0;
		regions[face].imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		regions[face].imageSubresource.mipLevel = 0;
		regions[face].imageSubresource.baseArrayLayer = face;
		regions[face].imageSubresource.layerCount = 1;
		regions[face].imageOffset = { 0, 0, 0 };
		regions[face].imageExtent = { faceSize, faceSize, 1 };
	}

	vkCmdCopyImageToBuffer(
		cmdBuffer,
		_cubemapImages[0],
		VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
		stagingBuffer,
		static_cast<uint32_t>(regions.size()),
		regions.data()
	);

	EndOneTimeCommands(cmdBuffer);  

	// --- 3. Map staging buffer and convert float4 -> 8-bit RGBA
	void* raw = nullptr;
	vkMapMemory(_device, stagingMemory, 0, totalSize, 0, &raw);
	float* src = reinterpret_cast<float*>(raw);

	// Output: 4 bytes/px for PNG
	const size_t outBytesPerTexel = 4;
	std::vector<uint8_t> strip(faceSize * faceSize * numFaces * outBytesPerTexel);

	const size_t texelCountPerFace = faceSize * faceSize;

	for (uint32_t face = 0; face < numFaces; ++face) {
		for (size_t i = 0; i < texelCountPerFace; ++i) {
			size_t srcIndex = (face * texelCountPerFace + i) * channels;
			size_t dstIndex = (face * texelCountPerFace + i) * outBytesPerTexel;

			for (int c = 0; c < 4; ++c) {
				float f = src[srcIndex + c];
				// simple clamp 0..1 → 0..255
				f = std::max(0.0f, std::min(1.0f, f));
				strip[dstIndex + c] = static_cast<uint8_t>(f * 255.0f);
			}
		}
	}

	vkUnmapMemory(_device, stagingMemory);

	stbi_write_png(
		filename.c_str(),
		faceSize,
		faceSize * numFaces,
		4,
		strip.data(),
		faceSize * 4
	);

	// --- 4. Cleanup ---
	vkDestroyBuffer(_device, stagingBuffer, nullptr);
	vkFreeMemory(_device, stagingMemory, nullptr);

	LOG_DEBUG("Cubemap exported to " + filename);
}

VkCommandBuffer CubemapRenderer::BeginOneTimeCommands() {
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandPool = _graphicCommandPool;
	allocInfo.commandBufferCount = 1;

	VkCommandBuffer cmd;
	vkAllocateCommandBuffers(_device, &allocInfo, &cmd);

	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	vkBeginCommandBuffer(cmd, &beginInfo);
	return cmd;
}

void CubemapRenderer::TransitionImageToTransferSrc(VkCommandBuffer cmd, VkImage image)
{
	VkImageMemoryBarrier barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;

	barrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

	barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.image = image;
	barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	barrier.subresourceRange.baseMipLevel = 0;
	barrier.subresourceRange.levelCount = 1;
	barrier.subresourceRange.baseArrayLayer = 0;
	barrier.subresourceRange.layerCount = 6;

	barrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT; 
	barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

	vkCmdPipelineBarrier(
		cmd,
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 
		VK_PIPELINE_STAGE_TRANSFER_BIT,       
		0,
		0, nullptr,
		0, nullptr,
		1, &barrier
	);
}

void CubemapRenderer::EndOneTimeCommands(VkCommandBuffer cmd) {
	vkEndCommandBuffer(cmd);
	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &cmd;

	VkQueue graphicsQueue;
	vkGetDeviceQueue(_device, _device->GetQueueFamilies()._graphics.value(), 0, &graphicsQueue);
	vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
	vkQueueWaitIdle(graphicsQueue);
	vkFreeCommandBuffers(_device, _graphicCommandPool, 1, &cmd);
}

//--------------------------------ComputeShader functions--------------------------------//

void CubemapRenderer::CartesianToSpherical(float x, float y, float z, float& theta, float& phi)
{
	theta = acosf(z);         // polar angle [0, pi]
	phi = atan2f(y, x);     // azimuth [-pi, pi]
}

// Compute pixel solid angle weight for one cube map face
/*float CubemapRenderer::ComputeSphereWeight(int px, int py, int faceSize)
{
	// Step 1: pixel coordinates in [-1,1]
	float invSize = 1.0f / faceSize;
	float x0 = 2.0f * px * invSize - 1.0f;
	float y0 = 2.0f * py * invSize - 1.0f;
	float x1 = 2.0f * (px + 1) * invSize - 1.0f;
	float y1 = 2.0f * (py + 1) * invSize - 1.0f;

	// Step 2: map corners to unit sphere
	auto MapToSphere = [](float x, float y) -> std::array<float, 3> {
		float z = 1.0f; // +Z face
		float len = sqrtf(x * x + y * y + z * z);
		return { x / len, y / len, z / len };
	};

	auto c00 = MapToSphere(x0, y0);
	auto c01 = MapToSphere(x0, y1);
	auto c10 = MapToSphere(x1, y0);
	auto c11 = MapToSphere(x1, y1);

	// Step 3: convert to spherical coordinates
	float theta00, phi00, theta01, phi01, theta10, phi10, theta11, phi11;
	CartesianToSpherical(c00[0], c00[1], c00[2], theta00, phi00);
	CartesianToSpherical(c01[0], c01[1], c01[2], theta01, phi01);
	CartesianToSpherical(c10[0], c10[1], c10[2], theta10, phi10);
	CartesianToSpherical(c11[0], c11[1], c11[2], theta11, phi11);

	// Step 4-5: Integrate solid angle over pixel
	float dPhi = std::max({ phi00, phi01, phi10, phi11 }) - std::min({ phi00, phi01, phi10, phi11 });
	float dTheta = std::max({ theta00, theta01, theta10, theta11 }) - std::min({ theta00, theta01, theta10, theta11 });

	// Inner integral: sin(theta) dTheta
	float thetaMin = std::min({ theta00, theta01, theta10, theta11 });
	float thetaMax = std::max({ theta00, theta01, theta10, theta11 });
	float innerIntegral = cosf(thetaMin) - cosf(thetaMax);

	// Outer integral: dPhi * innerIntegral
	float weight = dPhi * innerIntegral;

	// Step 6: normalize by pixel area in cube map (optional)
	// float cubePixelArea = (2.0f / faceSize) * (2.0f / faceSize);
	// weight /= cubePixelArea;

	return weight;
}
*/
/*float CubemapRenderer::ComputeSphereWeight(
	int px,
	int py,
	int faceSize,
	int face)
{
	// Step 1: pixel coordinates in [-1,1]
	float invSize = 1.0f / faceSize;

	float x0 = 2.0f * (px + 0) * invSize - 1.0f;
	float y0 = 2.0f * (py + 0) * invSize - 1.0f;
	float x1 = 2.0f * (px + 1) * invSize - 1.0f;
	float y1 = 2.0f * (py + 1) * invSize - 1.0f;

	// Step 2: map cube face to sphere (face-aware)
	auto MapToSphere = [&](float x, float y) -> std::array<float, 3>
	{
		float vx, vy, vz;

		switch (face) {
		case 0: vx = 1; vy = -y; vz = -x; break; // +X
		case 1: vx = -1; vy = -y; vz = x; break; // -X
		case 2: vx = x; vy = 1; vz = y; break; // +Y
		case 3: vx = x; vy = -1; vz = -y; break; // -Y
		case 4: vx = x; vy = -y; vz = 1; break; // +Z
		case 5: vx = -x; vy = -y; vz = -1; break; // -Z
		default: vx = vy = vz = 0; break;
		}

		float len = std::sqrt(vx * vx + vy * vy + vz * vz);
		return { vx / len, vy / len, vz / len };
	};

	auto c00 = MapToSphere(x0, y0);
	auto c01 = MapToSphere(x0, y1);
	auto c10 = MapToSphere(x1, y0);
	auto c11 = MapToSphere(x1, y1);

	// Step 3: convert to spherical coordinates
	auto Theta = [](const std::array<float, 3>& v) {
		return std::acos(std::clamp(v[2], -1.0f, 1.0f));
	};
	auto Phi = [](const std::array<float, 3>& v) {
		return std::atan2(v[1], v[0]);
	};

	float theta00 = Theta(c00), phi00 = Phi(c00);
	float theta01 = Theta(c01), phi01 = Phi(c01);
	float theta10 = Theta(c10), phi10 = Phi(c10);
	float theta11 = Theta(c11), phi11 = Phi(c11);

	// Step 4–5: integrate solid angle
	float thetaMin = std::min({ theta00, theta01, theta10, theta11 });
	float thetaMax = std::max({ theta00, theta01, theta10, theta11 });

	float phiMin = std::min({ phi00, phi01, phi10, phi11 });
	float phiMax = std::max({ phi00, phi01, phi10, phi11 });

	float innerIntegral = std::cos(thetaMin) - std::cos(thetaMax);
	float weight = (phiMax - phiMin) * innerIntegral;

	return weight;
}*/

static inline float SolidAngle(float x, float y)
{
	return std::atan2(x * y, std::sqrt(x * x + y * y + 1.0f));
}

float CubemapRenderer::ComputeSphereWeight(
	int px,
	int py, int faceSize)
{
	float inv = 2.0f / faceSize;

	float x0 = -1.0f + px * inv;
	float y0 = -1.0f + py * inv;
	float x1 = x0 + inv;
	float y1 = y0 + inv;

	float w =
		SolidAngle(x1, y1) -
		SolidAngle(x0, y1) -
		SolidAngle(x1, y0) +
		SolidAngle(x0, y0);

	return w;
}

void CubemapRenderer::CreateComputeDescriptorSetLayout() {
	std::vector<VkDescriptorSetLayoutBinding> bindings{
		// Image
		{0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT},
		// Vertex index list
		{1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT},
		// lambda output
		{2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT},
		// wsum output
		{3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT},
		// 
		{ 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT }
	};

	_computeLayout = _descriptorPool->CreateDescriptorSetLayout(bindings);
}

void CubemapRenderer::CreateComputeBuffers()
{
	// -------------------------------
	// Lambda / wsum buffers (ONE cubemap per dispatch)
	// -------------------------------
	const size_t numVertices =
		static_cast<size_t>(_cageMesh._vertices.rows());

	const size_t lambdaBytes = numVertices * sizeof(float);
	const size_t wsumBytes = sizeof(float);

	_lambdaStagingBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>((std::byte*)nullptr, lambdaBytes),
		VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_wsumStagingBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>((std::byte*)nullptr, wsumBytes),
		VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_lambdaBuffer = _resourceManager->AllocateDeviceBuffer(
		lambdaBytes,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	_wsumBuffer = _resourceManager->AllocateDeviceBuffer(
		wsumBytes,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// -------------------------------
	// Vertex index list (flatten Eigen faces)
	// -------------------------------
	const int triCount = _cageMesh._faces.rows();
	if (_cageMesh._faces.cols() != 3) {
		throw std::runtime_error("EigenMesh faces must be T x 3.");
	}

	std::vector<uint32_t> vertexList(static_cast<size_t>(triCount) * 3);

	for (int t = 0; t < triCount; ++t) {
		vertexList[t * 3 + 0] =
			static_cast<uint32_t>(_cageMesh._faces(t, 0));
		vertexList[t * 3 + 1] =
			static_cast<uint32_t>(_cageMesh._faces(t, 1));
		vertexList[t * 3 + 2] =
			static_cast<uint32_t>(_cageMesh._faces(t, 2));
	}

	const size_t vertexListBytes =
		vertexList.size() * sizeof(uint32_t);

	// -------------------------------
	// Upload vertex list (EXPLICIT memcpy)
	// -------------------------------
	auto vertexListStaging = _resourceManager->CreateBufferAndCopy(
		std::span<const uint32_t>(vertexList.data(), vertexList.size()),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);


	std::memcpy(
		vertexListStaging._mappedData,
		vertexList.data(),
		vertexListBytes
	);

	_vertexListBuffer = _resourceManager->AllocateDeviceBuffer(
		vertexListBytes,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(
		vertexListStaging._deviceBuffer,
		_vertexListBuffer._deviceBuffer,
		vertexListBytes
	);
}

void CubemapRenderer::AllocateComputeDescriptorSet() {
	VkDescriptorSetLayout layout = _computeLayout->GetReference();

	VkDescriptorSetAllocateInfo alloc{};
	alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	alloc.descriptorPool = _descriptorPool;
	alloc.descriptorSetCount = 1;
	alloc.pSetLayouts = &layout;

	vkAllocateDescriptorSets(_device, &alloc, &_computeDescriptorSet);
}

void CubemapRenderer::UpdateComputeDescriptorSet() {

	VkDescriptorImageInfo imageInfo{};
	imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
	imageInfo.imageView = _baryTexImageView; 
	imageInfo.sampler = _sampler;

	VkDescriptorBufferInfo vertexListInfo{};
	vertexListInfo.buffer = _vertexListBuffer._deviceBuffer;
	vertexListInfo.range = VK_WHOLE_SIZE;

	VkDescriptorBufferInfo lambdaInfo{};
	lambdaInfo.buffer = _lambdaBuffer._deviceBuffer;
	lambdaInfo.range = VK_WHOLE_SIZE;

	VkDescriptorBufferInfo wsumInfo{};
	wsumInfo.buffer = _wsumBuffer._deviceBuffer;
	wsumInfo.range = VK_WHOLE_SIZE;

	VkDescriptorImageInfo weightInfo{}; 
	weightInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; 
	weightInfo.imageView = _solidAngleArrayView;
	weightInfo.sampler = _solidAngleSampler;

	std::array<VkWriteDescriptorSet, 5> writes{};

	writes[0] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 0, 0, 1,
				  VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &imageInfo, nullptr, nullptr };
	writes[1] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 1, 0, 1,
				  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &vertexListInfo, nullptr };
	writes[2] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 2, 0, 1,
				  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &lambdaInfo, nullptr };
	writes[3] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 3, 0, 1,
				  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &wsumInfo, nullptr };
	writes[4] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 4, 0,	1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &weightInfo, nullptr, nullptr };

	vkUpdateDescriptorSets(_device, writes.size(), writes.data(), 0, nullptr);
}

void CubemapRenderer::CreateComputePipeline() {
	ComputePipelineObjectProxy proxy;
	proxy._renderPipelineManager = _renderPipelineManager;
	proxy._shaderModule = "assets/shaders/PMVCCompute.comp.spv";
	proxy._descriptorSetLayouts = { _computeLayout };

	VkPushConstantRange range{};
	range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	range.offset = 0;
	range.size = sizeof(ComputePushConstants);
	proxy._pushConstantRanges = { range }; 

	_computePipelineHandle = proxy.Build();
}

void CubemapRenderer::CreateComputeCommandBuffer() {
	VkCommandBufferAllocateInfo alloc{};
	alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	alloc.commandPool = _computeCommandPool;   
	alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	alloc.commandBufferCount = 1;

	vkAllocateCommandBuffers(_device, &alloc, &_computeCommandBuffer);
}

void CubemapRenderer::ComputeCoordinates(uint32_t cubeIndex)
{
	// We only have one cubemap image & view, reused for each vertex.
	// So we always use index 0 here.
	//(void)cubeIndex; // cubeIndex only used to index CPU-side result arrays, not GPU resources

	uint32_t numTriangles = static_cast<int>(_cageMesh._faces.rows());
	LOG_DEBUG(numTriangles);
	uint32_t numCageVertices = static_cast<int>(_cageMesh._vertices.rows());
	LOG_DEBUG(numCageVertices);

	_baryTexImageView = _baryTexImageViews[0];
	UpdateComputeDescriptorSet();

	vkResetCommandBuffer(_computeCommandBuffer, 0);

	VkCommandBufferBeginInfo begin{};
	begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

	vkBeginCommandBuffer(_computeCommandBuffer, &begin);

	VkImageSubresourceRange subresourceRange{};
	subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	subresourceRange.baseMipLevel = 0;
	subresourceRange.levelCount = 1;
	subresourceRange.baseArrayLayer = 0;
	subresourceRange.layerCount = 6;

	// Transition the *single* cubemap image we are reusing:
	InsertImageMemoryBarrierToGeneral(
		_computeCommandBuffer,
		_cubemapImages[0],     
		subresourceRange
	);

	const PipelineObject& obj =
		_renderPipelineManager->GetPipelineObject(_computePipelineHandle);

	vkCmdBindPipeline(_computeCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, obj._handle);

	vkCmdBindDescriptorSets(
		_computeCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
		obj._pipelineLayout, 0, 1,
		&_computeDescriptorSet, 0, nullptr);

	// --- Push constants: we are only processing ONE cubemap on this dispatch ---
	ComputePushConstants pc{};
	pc.uNumCubemaps = 1;                           
	pc.uNumCageVertices = static_cast<int>(_cageMesh._vertices.rows());
	pc.uFaceSize = glm::ivec2(512, 512);
	pc.uFacesPerCubemap = 6;
	pc.uNumTriangles = _cageMesh._faces.rows();

	//LOG_DEBUG(static_cast<int>(_cageMesh._faces.size()) + _cageMesh._vertices.rows() + " triangles in cage mesh.\n");

	vkCmdPushConstants(
		_computeCommandBuffer,
		obj._pipelineLayout,
		VK_SHADER_STAGE_COMPUTE_BIT,
		0,
		sizeof(ComputePushConstants),
		&pc
	);

	//uint32_t groupsX = (512 + 7) / 8;
	//uint32_t groupsY = (512 + 7) / 8;

	//vkCmdDispatch(_computeCommandBuffer, groupsX, groupsY, 1);  // one cubemap
		int indexCount = _cageMesh._faces.size();
		LOG_DEBUG("index count : " + std::to_string(indexCount));
		int triCount = _cageMesh._faces.rows(); //(and assert indexCount % 3 == 0)
		LOG_DEBUG("triangle count : " + std::to_string(triCount));
		LOG_DEBUG("lambda gpu size: " + std::to_string(_lambdaBuffer._allocatedSize));
		LOG_DEBUG("wsum gpu size: " + std::to_string(_wsumBuffer._allocatedSize));

	vkCmdDispatch(_computeCommandBuffer, 1, 1, 1);

	vkEndCommandBuffer(_computeCommandBuffer);

	VkSubmitInfo submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit.commandBufferCount = 1;
	submit.pCommandBuffers = &_computeCommandBuffer;

	VkQueue queue;
	vkGetDeviceQueue(_device, _device->GetQueueFamilies()._graphics.value(), 0, &queue);

	vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE);
	vkQueueWaitIdle(queue);
}

void CubemapRenderer::ReadbackCompute(uint32_t cubeIndex)
{
	VkCommandBuffer cmd = BeginOneTimeCommands();

	const VkDeviceSize lambdaBytes = static_cast<VkDeviceSize>(_cageMesh._vertices.rows()) * sizeof(float);
	const VkDeviceSize wsumBytes = sizeof(float);

	VkBufferCopy lambdaCopy{ 0, 0, lambdaBytes };
	vkCmdCopyBuffer(cmd,
		_lambdaBuffer._deviceBuffer,
		_lambdaStagingBuffer._deviceBuffer,
		1, &lambdaCopy);

	VkBufferCopy wsumCopy{ 0, 0, wsumBytes };
	vkCmdCopyBuffer(cmd,
		_wsumBuffer._deviceBuffer,
		_wsumStagingBuffer._deviceBuffer,
		1, &wsumCopy);

	EndOneTimeCommands(cmd);

	float* lambdaCPU = reinterpret_cast<float*>(_lambdaStagingBuffer._mappedData);
	float* wsumCPU = reinterpret_cast<float*>(_wsumStagingBuffer._mappedData);

	storeLambdaForVertex(cubeIndex, lambdaCPU);

	// shader writes only wsum[0]
	storeWsumForVertex(cubeIndex, wsumCPU);
}

// store lambda results read back from the GPU into CPU-side container
void CubemapRenderer::storeLambdaForVertex(uint32_t cubeIndex, const float* lambdaCPU)
{
	const size_t numCageVerts = static_cast<size_t>(_cageMesh._vertices.rows());
	if (numCageVerts == 0) return;

	const size_t expectedSize = numCageVerts; 

	if (_lambdaResults.size() <= cubeIndex)
		_lambdaResults.resize(cubeIndex + 1);

	if (_lambdaResults[cubeIndex].size() != expectedSize)
		_lambdaResults[cubeIndex].assign(expectedSize, 0.0f);

	float* dst = _lambdaResults[cubeIndex].data();
	for (size_t i = 0; i < expectedSize; i++)
	{
		dst[i] += lambdaCPU[i];
	}
}

void CubemapRenderer::storeWsumForVertex(uint32_t cubeIndex, const float* wsumCPU)
{
	if (!wsumCPU) return;

	if (_wsumResults.size() <= cubeIndex)
		_wsumResults.resize(cubeIndex + 1, 0.0f);

	_wsumResults[cubeIndex] = wsumCPU[0];
}

void CubemapRenderer::InsertImageMemoryBarrierToGeneral(
	VkCommandBuffer cmd,
	VkImage image,
	VkImageSubresourceRange subresourceRange)
{
	VkImageMemoryBarrier barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;

	// Compute shader will read the image (sampled)
	barrier.srcAccessMask =
		VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |      
		VK_ACCESS_TRANSFER_WRITE_BIT |
		VK_ACCESS_SHADER_WRITE_BIT;                 

	barrier.dstAccessMask =
		VK_ACCESS_SHADER_READ_BIT;                  

	barrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;   
	barrier.image = image;
	barrier.subresourceRange = subresourceRange;

	vkCmdPipelineBarrier(
		cmd,
		VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT |
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
		VK_PIPELINE_STAGE_TRANSFER_BIT,
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
		0,
		0, nullptr,
		0, nullptr,
		1, &barrier
	);
}

void CubemapRenderer::CreateSampler() {
		VkSamplerCreateInfo s{};
		s.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
		s.pNext = nullptr;
		s.flags = 0;

		s.magFilter = VK_FILTER_NEAREST;
		s.minFilter = VK_FILTER_NEAREST;
		s.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;

		s.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		s.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		s.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;

		s.mipLodBias = 0.0f;
		s.minLod = 0.0f;
		s.maxLod = 0.0f;

		s.anisotropyEnable = VK_FALSE;
		s.maxAnisotropy = 1.0f;

		s.compareEnable = VK_FALSE;
		s.compareOp = VK_COMPARE_OP_ALWAYS;

		s.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
		s.unnormalizedCoordinates = VK_FALSE;

		if (vkCreateSampler(_device, &s, nullptr, &_sampler) != VK_SUCCESS) {
			throw std::runtime_error("Failed to create sampler for compute image!");
		}
}

void CubemapRenderer::CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size)
{
	// Allocate a temporary one-time command buffer
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandPool = _graphicCommandPool;
	allocInfo.commandBufferCount = 1;

	VkCommandBuffer cmd;
	vkAllocateCommandBuffers(_device, &allocInfo, &cmd);

	// Begin recording
	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	vkBeginCommandBuffer(cmd, &beginInfo);

	// Copy command
	VkBufferCopy copyRegion{};
	copyRegion.srcOffset = 0;
	copyRegion.dstOffset = 0;
	copyRegion.size = size;
	vkCmdCopyBuffer(cmd, src, dst, 1, &copyRegion);

	vkEndCommandBuffer(cmd);

	// Submit and wait
	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &cmd;

	VkQueue graphicsQueue;
	vkGetDeviceQueue(_device, _device->GetQueueFamilies()._graphics.value(), 0, &graphicsQueue);

	vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
	vkQueueWaitIdle(graphicsQueue);

	// Clean up
	vkFreeCommandBuffers(_device, _graphicCommandPool, 1, &cmd);
}

void CubemapRenderer::CreateCubemapImageViews() {
	_baryTexImageViews.resize(_cubemapImages.size());

	for (size_t i = 0; i < _cubemapImages.size(); i++) {

		VkImageViewCreateInfo viewInfo{};
		viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
		viewInfo.format = VK_FORMAT_R32G32B32A32_SFLOAT;
		viewInfo.image = _cubemapImages[i];

		viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		viewInfo.subresourceRange.baseMipLevel = 0;     
		viewInfo.subresourceRange.levelCount = 1;

		viewInfo.subresourceRange.baseArrayLayer = 0;   
		viewInfo.subresourceRange.layerCount = 6;      

		if (vkCreateImageView(_device, &viewInfo, nullptr, &_baryTexImageViews[i]) != VK_SUCCESS) {
			throw std::runtime_error("Failed to create cubemap bary image view!");
		}
	}
}

void CubemapRenderer::WriteWeightsToFile(const std::string& filename)
{
	std::ofstream file(filename);
	if (!file.is_open()) throw std::runtime_error("Failed to open weight write file!");

	file << "===== LAMBDA BUFFER (cubemaps=" << _lambdaResults.size()
		<< ", verts=" << (_lambdaResults.empty() ? 0 : _lambdaResults[0].size()) << ") =====\n";

	for (size_t cub = 0; cub < _lambdaResults.size(); ++cub) {
		for (size_t v = 0; v < _lambdaResults[cub].size(); ++v) {
			file << "lambda[" << cub << "][" << v << "] = " << _lambdaResults[cub][v] << "\n";
		}
	}

	file << "\n===== WSUM BUFFER (" << _wsumResults.size() << " floats) =====\n";
	for (size_t cub = 0; cub < _wsumResults.size(); ++cub) {
		file << "wsum[" << cub << "] = " << _wsumResults[cub] << "\n";
	}

	file.close();
	LOG_INFO("Weights written to " + filename);
}

glm::vec3 CubemapRenderer::CubeFaceDir(int face, float x, float y)
{
	switch (face) {
	case 0: return normalize(glm::vec3(1, -y, -x)); // +X
	case 1: return normalize(glm::vec3(-1, -y, x)); // -X
	case 2: return normalize(glm::vec3(x, 1, y)); // +Y
	case 3: return normalize(glm::vec3(x, -1, -y)); // -Y
	case 4: return normalize(glm::vec3(x, -y, 1)); // +Z
	case 5: return normalize(glm::vec3(-x, -y, -1)); // -Z
	}
}

void CubemapRenderer::SphereWeightInitialization(uint32_t size) {
		const uint32_t faceSize = size;
		const uint32_t faceCount = 6;

		// ------------------------------------------------------------
		// 1) CPU: precompute solid-angle weights (ONCE)
		// ------------------------------------------------------------
		std::vector<float> weights(faceCount * faceSize * faceSize);

		for (uint32_t face = 0; face < faceCount; ++face) {
			for (uint32_t y = 0; y < faceSize; ++y) {
				for (uint32_t x = 0; x < faceSize; ++x) {
					weights[
						face * faceSize * faceSize +
							y * faceSize + x
					] = ComputeSphereWeight(x, y, faceSize);
				}
			}
		}

		// ------------------------------------------------------------
		// 2) Create GPU image (R32_SFLOAT, 6 layers)
		// ------------------------------------------------------------
		VkImageCreateInfo img{};
		img.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		img.imageType = VK_IMAGE_TYPE_2D;
		img.format = VK_FORMAT_R32_SFLOAT;
		img.extent = { faceSize, faceSize, 1 };
		img.mipLevels = 1;
		img.arrayLayers = 6;
		img.samples = VK_SAMPLE_COUNT_1_BIT;
		img.tiling = VK_IMAGE_TILING_OPTIMAL;
		img.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
		img.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
		img.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

		vkCreateImage(_device, &img, nullptr, &_solidAngleImage);

		VkMemoryRequirements memReq{};
		vkGetImageMemoryRequirements(_device, _solidAngleImage, &memReq);

		VkMemoryAllocateInfo alloc{};
		alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		alloc.allocationSize = memReq.size;
		alloc.memoryTypeIndex =
			FindMemoryType(memReq.memoryTypeBits,
				VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

		vkAllocateMemory(_device, &alloc, nullptr, &_solidAngleMemory);
		vkBindImageMemory(_device, _solidAngleImage, _solidAngleMemory, 0);

		// ------------------------------------------------------------
		// 3) Upload via staging buffer
		// ------------------------------------------------------------
		const VkDeviceSize uploadBytes = weights.size() * sizeof(float);

		auto staging = _resourceManager->CreateBufferAndCopy(
			std::span<const float>(weights.data(), weights.size()),
			VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		);

		VkCommandBuffer cmd = BeginOneTimeCommands();

		VkImageSubresourceRange range{};
		range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		range.baseMipLevel = 0;
		range.levelCount = 1;
		range.baseArrayLayer = 0;
		range.layerCount = 6;

		VkImageMemoryBarrier barrier1{};
		barrier1.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		barrier1.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		barrier1.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		barrier1.srcAccessMask = 0;
		barrier1.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		barrier1.image = _solidAngleImage;
		barrier1.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		barrier1.subresourceRange.baseMipLevel = 0;
		barrier1.subresourceRange.levelCount = 1;
		barrier1.subresourceRange.baseArrayLayer = 0;
		barrier1.subresourceRange.layerCount = 6;

		vkCmdPipelineBarrier(
			cmd,
			VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
			VK_PIPELINE_STAGE_TRANSFER_BIT,
			0,
			0, nullptr,
			0, nullptr,
			1, &barrier1
		);

		VkBufferImageCopy copy{};
		copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		copy.imageSubresource.mipLevel = 0;
		copy.imageSubresource.baseArrayLayer = 0;
		copy.imageSubresource.layerCount = 6;
		copy.imageExtent = { faceSize, faceSize, 1 };

		vkCmdCopyBufferToImage(
			cmd,
			staging._deviceBuffer,
			_solidAngleImage,
			VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
			1,
			&copy
		);

		VkImageMemoryBarrier barrier2{};
		barrier2.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		barrier2.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		barrier2.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		barrier2.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		barrier2.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
		barrier2.image = _solidAngleImage;
		barrier2.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		barrier2.subresourceRange.baseMipLevel = 0;
		barrier2.subresourceRange.levelCount = 1;
		barrier2.subresourceRange.baseArrayLayer = 0;
		barrier2.subresourceRange.layerCount = 6;

		vkCmdPipelineBarrier(
			cmd,
			VK_PIPELINE_STAGE_TRANSFER_BIT,
			VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
			0,
			0, nullptr,
			0, nullptr,
			1, &barrier2
		);

		EndOneTimeCommands(cmd);

		// ------------------------------------------------------------
		// 4) Create 2D-array image view (for compute)
		// ------------------------------------------------------------
		VkImageViewCreateInfo view{};
		view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		view.image = _solidAngleImage;
		view.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
		view.format = VK_FORMAT_R32_SFLOAT;
		view.subresourceRange = range;

		vkCreateImageView(_device, &view, nullptr, &_solidAngleArrayView);

		// ------------------------------------------------------------
		// 5) Create sampler (NEAREST)
		// ------------------------------------------------------------
		VkSamplerCreateInfo samp{};
		samp.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
		samp.magFilter = VK_FILTER_NEAREST;
		samp.minFilter = VK_FILTER_NEAREST;
		samp.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
		samp.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samp.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samp.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;

		vkCreateSampler(_device, &samp, nullptr, &_solidAngleSampler);
		double sum = 0.0;
		for (float w : weights) sum += w;
		LOG_DEBUG("Total solid angle = " + std::to_string(sum));
	}
	/*
	}
	/*on upload
	VkBufferImageCopy region{};
region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
region.imageSubresource.mipLevel = 0;
region.imageSubresource.baseArrayLayer = 0;
region.imageSubresource.layerCount = 6;
region.imageExtent = { faceSize, faceSize, 1 };

binding
// binding = 4 (example)
{ 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT }
shader
// binding = 4 (example)
{ 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT }
*/

