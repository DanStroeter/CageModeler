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

void ComputeCoordinates() {
	// Placeholder function to compute cubemap coordinates
	std::cout << "Computed PMVC coordinates beep boop!" << "\n";
}

CubemapRenderer::CubemapRenderer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
	const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance) : _renderPipelineManager(renderPipelineManager),
	_resourceManager(resourceManager), _device(device), _instance(instance)
{
	//TODO alignment??
	// Calculate required alignment based on minimum device offset alignment
	//_objectsBufferDynamicAlignment = _device->GetMinimumMemoryAlignment<ModelInfo>();
	//_objectsBufferData.Resize(TotalNumSceneObjects);
}

void CubemapRenderer::Initialize()
{
	_descriptorPool = CreateRenderResource<DescriptorPool>(_device);
	CreateImageViews(512, VK_FORMAT_R8G8B8A8_UNORM);
	CreateRenderPass(VK_FORMAT_R8G8B8A8_UNORM);
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
	const EigenMesh geom = _cageMesh;
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
				static_cast<uint32_t>(tri),
				static_cast<uint32_t>(v)  
				});
		}
	}

	_vertexBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span(vertexData),
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	memcpy(_vertexBuffer._mappedData, vertexData.data(), vertexData.size() * sizeof(CubemapVertex));
}

void CubemapRenderer::CreateIndexBufferFromMesh()
{
	const EigenMesh& geom = _cageMesh;

	std::vector<uint32_t> indices;
	indices.reserve(geom._faces.size());

	for (int i = 0; i < geom._faces.rows(); ++i)
		for (int j = 0; j < geom._faces.cols(); ++j)
			indices.push_back(static_cast<uint32_t>(geom._faces(i, j)));

	_indexBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span(indices),
		VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	memcpy(_indexBuffer._mappedData, indices.data(), indices.size() * sizeof(uint32_t));
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

	std::vector<glm::vec3> vertices;
	for (int i = 0; i < _cageMesh._vertices.rows(); ++i)
	{
		const auto& v = _cageMesh._vertices.row(i);
		vertices.push_back(glm::vec3(
			static_cast<float>(v(0)),
			static_cast<float>(v(1)),
			static_cast<float>(v(2))
		));
	}

	for (size_t vertexIndex = 0; vertexIndex < vertices.size(); ++vertexIndex)
	{
		const glm::vec3& camPos = vertices[vertexIndex];

		float nearPlane = ComputeNearPlane(camPos, vertices);
		float farPlane = ComputeFarPlane(camPos, vertices);

		// Update UBO
		/*CubemapMatricesUBO ubo{};
		ubo.proj = glm::perspective(glm::radians(90.0f), 1.0f, nearPlane, farPlane);
		ubo.proj[1][1] *= -1.0f;

		for (int i = 0; i < 6; ++i)
			ubo.views[i] = ComputeCubemapViewMatrix(i, camPos);*/

		//memcpy(_matricesUniformBuffer._mappedData, &ubo, sizeof(ubo));

		// Render each cubemap face sequentially
		for (int face = 0; face < 6; ++face)
		{
			CubemapMatricesUBO faceUBO{};
			faceUBO.proj = glm::perspective(glm::radians(90.0f), 1.0f, nearPlane, farPlane);
			faceUBO.proj[1][1] *= -1.0f;

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
			vkQueueSubmit(graphicsQueue, 1, &submitInfo, _renderFence);
			vkWaitForFences(_device, 1, &_renderFence, VK_TRUE, UINT64_MAX);
			vkResetFences(_device, 1, &_renderFence);
		}
		std::string filename = "debug_cubemap_" + std::to_string(vertexIndex) + ".png";
		ExportCubemapAsVerticalStrip("F:/Cubemaps/" + filename);
	}
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
/*void CubemapRenderer::CreateComputePipeline()
{
	// Create the compute pipeline.
	std::array descriptorSetLayouts{ _matricesLayout->GetReference(),
		_objectDataLayout->GetReference() };
	_computePipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetDescriptorSetLayouts(std::span(descriptorSetLayouts))
		.SetShaderModule(ShaderModuleType::Compute, "assets/shaders/CubemapCompute.comp.spv")
		.Build();
}*/
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
	const uint32_t faceSize = 512; // cubemap resolution
	const uint32_t numFaces = 6;
	const VkDeviceSize imageSizePerFace = faceSize * faceSize * 4; // RGBA8
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

	// --- 2. Record command buffer to copy cubemap to staging buffer ---
	VkCommandBuffer cmdBuffer = BeginOneTimeCommands();

	// Transition cubemap image to transfer src
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

	// --- 3. Submit command buffer with a fence ---
	vkEndCommandBuffer(cmdBuffer);

	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &cmdBuffer;

	VkQueue graphicsQueue;
	vkGetDeviceQueue(_device, _device->GetQueueFamilies()._graphics.value(), 0, &graphicsQueue);

	VkFenceCreateInfo fenceInfo{};
	fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceInfo.flags = 0;

	VkFence fence;
	vkCreateFence(_device, &fenceInfo, nullptr, &fence);

	vkQueueSubmit(graphicsQueue, 1, &submitInfo, fence);
	vkWaitForFences(_device, 1, &fence, VK_TRUE, UINT64_MAX);

	vkDestroyFence(_device, fence, nullptr);
	vkFreeCommandBuffers(_device, _graphicCommandPool, 1, &cmdBuffer);

	// --- 4. Map staging buffer and write vertical strip ---
	uint8_t* data;
	vkMapMemory(_device, stagingMemory, 0, totalSize, 0, reinterpret_cast<void**>(&data));

	std::vector<uint8_t> strip(faceSize * faceSize * 4 * numFaces);

	for (uint32_t face = 0; face < numFaces; ++face) {
		std::memcpy(
			strip.data() + face * faceSize * faceSize * 4,
			data + face * faceSize * faceSize * 4,
			faceSize * faceSize * 4
		);
	}

	stbi_write_png(filename.c_str(), faceSize, faceSize * numFaces, 4, strip.data(), faceSize * 4);

	vkUnmapMemory(_device, stagingMemory);

	// --- 5. Cleanup ---
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
	barrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

	vkCmdPipelineBarrier(
		cmd,
		VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
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



