#include <Rendering/Cubemaps/CubemapRenderer.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/ScreenPass.h>
#include <Editor/Light.h>

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
	CreateVertexBuffer(_cageMesh->GetGeometry()._positions);
	CreateIndexBuffer(_cageMesh->GetGeometry()._indices);
}

void CubemapRenderer::CreateImageViews(uint32_t size, VkFormat format) {
	// Create Cubemap image
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
	imageInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
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
	VkCommandPoolCreateInfo poolInfo{ };
	poolInfo.queueFamilyIndex = queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	vkCreateCommandPool(_device, &poolInfo, nullptr, &_graphicCommandPool);
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

void CubemapRenderer::CreateVertexBuffer(const std::vector < glm::vec3>& vertices)
{
	if (!_cageMesh) {
		throw std::runtime_error("Cage mesh not initialized!");
	}

	_vertexBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span(vertices),
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	memcpy(_vertexBuffer._mappedData, vertices.data(), vertices.size() * sizeof(Vertex));
}

void CubemapRenderer::CreateIndexBuffer(const std::vector<uint32_t>& indices)
{
	if (!_cageMesh) {
		throw std::runtime_error("Cage mesh not initialized!");
	}

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
	if (!_cageMesh) return;

	const auto& vertices = _cageMesh->GetGeometry()._positions;

	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	VkClearValue clearValues[2];
	clearValues[0].color = { {0.0f, 0.0f, 0.0f, 1.0f} };
	clearValues[1].depthStencil = { 1.0f, 0 };

	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

	for (size_t vertexIndex = 0; vertexIndex < vertices.size(); ++vertexIndex)
	{
		const glm::vec3& camPos = vertices[vertexIndex];

		float nearPlane = ComputeNearPlane(camPos, vertices);
		float farPlane = ComputeFarPlane(camPos, vertices);

		// Update UBO
		CubemapMatricesUBO ubo{};
		ubo.proj = glm::perspective(glm::radians(90.0f), 1.0f, nearPlane, farPlane);

		for (int i = 0; i < 6; ++i)
			ubo.views[i] = ComputeCubemapViewMatrix(i, camPos);

		memcpy(_matricesUniformBuffer._mappedData, &ubo, sizeof(ubo));

		// Render each cubemap face sequentially
		for (int face = 0; face < 6; ++face)
		{
			VkCommandBuffer cmdBuffer = _commandBuffers[face];

			vkResetCommandBuffer(cmdBuffer, 0);
			vkBeginCommandBuffer(cmdBuffer, &beginInfo);

			VkRenderPassBeginInfo renderPassInfo{};
			renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
			renderPassInfo.renderPass = _renderPass;
			renderPassInfo.framebuffer = _faceFramebuffers[face];
			renderPassInfo.renderArea.offset = { 0, 0 };
			renderPassInfo.renderArea.extent = { 512, 512 }; // cubemap size
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
				0, 1,
				&_matricesDescriptorSet,
				0, nullptr
			);

			vkCmdDrawIndexed(cmdBuffer, static_cast<uint32_t>(_cageMesh->GetGeometry()._indices.size()), 1, 0, 0, 0);

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

