#include <Rendering/Cubemaps/CubemapRenderer.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/ScreenPass.h>
#include <Editor/Light.h>



CubemapRenderer::CubemapRenderer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
	const std::shared_ptr<RenderResourceManager>& resourceManager)
	: _renderPipelineManager(renderPipelineManager)
	, _resourceManager(resourceManager)
{
	// Calculate required alignment based on minimum device offset alignment
	//_objectsBufferDynamicAlignment = _device->GetMinimumMemoryAlignment<ModelInfo>();
	//_objectsBufferData.Resize(TotalNumSceneObjects);
}

void CubemapRenderer::Initialize(const SubsystemsCollection& collection)
{
	// Creates all descriptor set layouts.
	//CreateDescriptorSetLayouts();

	// Creates, maps and updates the uniform buffers.
	//CreateUniformBuffers();

	// Allocates and updates the descriptor sets.
	//AllocateDescriptorSets();

	// Creates all graphics pipelines.
	//CreateCubemapRenderPipeline();
	//CreateComputePipeline();

	// Creates the grid and the gradient background.
	//CreateScreenPasses();

	// Get the editor's RenderSubsystem
	_renderSubsystem = GetDependencySubsystem<RenderSubsystem>(collection);
	// Reuse Vulkan instance and device
	_device = _renderSubsystem->getDevice();
	VkInstance instance = _renderSubsystem->getInstance();

	/*VkPipelineLayoutCreateInfo layoutInfo{};
	layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	layoutInfo.setLayoutCount = 0; // if you use descriptors, set them here
	layoutInfo.pushConstantRangeCount = 0;
	if (vkCreatePipelineLayout(_device, &layoutInfo, nullptr, _CubemapPipelineLayout) != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create Cubemap pipeline layout!");
	}*/
	CreateImageViews(512, VK_FORMAT_R8G8B8A8_UNORM);
}

void CubemapRenderer::CreateImageViews(uint32_t size, VkFormat format) {
	// Create Cubemap image
	VkImageCreateInfo imageInfo{};
	imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	imageInfo.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
	imageInfo.imageType = VK_IMAGE_TYPE_2D;
	imageInfo.format = format;
	imageInfo.extent = { size, size, 1 };
	imageInfo.mipLevels = 0;
	imageInfo.arrayLayers = 6;
	imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	imageInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
	imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	RenderResourceRef<VkImage> cubemapImage;

	if (vkCreateImage(_device, &imageInfo, nullptr, cubemapImage) != VK_SUCCESS) {
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

	RenderResourceRef<VkDeviceMemory> cubemapMemory;
	if (vkAllocateMemory(_device, &allocInfo, nullptr, cubemapMemory) != VK_SUCCESS) {
		throw std::runtime_error("Failed to allocate memory for Cubemap image!");
	}

	if (vkBindImageMemory(_device, cubemapImage, cubemapMemory, 0) != VK_SUCCESS) {
		throw std::runtime_error("Failed to bind memory to Cubemap image!");
	}
	_cubemapImageMemory.push_back(cubemapMemory);

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
		viewInfo.subresourceRange.baseArrayLayer = 0;
		viewInfo.subresourceRange.layerCount = 6;

		RenderResourceRef<VkImageView> faceImageView;

		if (vkCreateImageView(_device, &viewInfo, nullptr, faceImageView) != VK_SUCCESS) {
			throw std::runtime_error("Failed to create Cubemap face image view!");
		}
		_faceImageViews.push_back(faceImageView);
	}

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

	RenderResourceRef<VkImageView> cubemapView;
	if (vkCreateImageView(_device, &cubeViewInfo, nullptr, cubemapView) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create Cubemap image view!");
	}
	_cubemapViews.push_back(cubemapView);
}

/*void CubemapRenderer::CreateScreenPasses()
{
	RenderArrayType<std::vector<VkDescriptorSet>> descriptorSets{};
	for (std::size_t i = 0; i < descriptorSets.size(); ++i)
	{
		descriptorSets[i] = std::vector{ _matricesDescriptorSets[i] };
	}

	const auto backgroundScreenPass = std::make_shared<ScreenPass>(MeshProxySolidPipeline{
		_backgroundPipelineHandle,
		_staticMeshInfluenceMapPipelineHandle,
		descriptorSets });
	_screenPasses.push_back(backgroundScreenPass);
	backgroundScreenPass->CollectRenderProxy(_renderProxyCollector, _device, _renderCommandScheduler);

	const auto gridScreenPass = std::make_shared<ScreenPass>(MeshProxySolidPipeline{
		_gridPipelineHandle,
		_staticMeshInfluenceMapPipelineHandle,
		descriptorSets });
	_screenPasses.push_back(gridScreenPass);
	gridScreenPass->CollectRenderProxy(_renderProxyCollector, _device, _renderCommandScheduler);
	;
}*/

/*std::shared_ptr<PolygonMesh> CubemapRenderer::AddCage(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& indices)
{
	RenderArrayType<std::vector<VkDescriptorSet>> descriptorSets{};
	for (std::size_t i = 0; i < descriptorSets.size(); ++i)
	{
		descriptorSets[i] = std::vector{ _matricesDescriptorSets[i], _objectDataDescriptorSets[i] };
	}

	const auto mesh = std::make_shared<PolygonMesh>(vertices,
		indices,
		MeshProxySolidPipeline{
			_cagePipelineHandle,
			_staticMeshInfluenceMapPipelineHandle,
			descriptorSets },
			MeshProxyWireframePipeline{ _pointsPipelineHandle, _edgesPipelineHandle, _polysPipelineHandle, descriptorSets },
			true,
			WireframeRenderMode::Points | WireframeRenderMode::Edges | WireframeRenderMode::Polygons);
	mesh->CollectRenderProxy(_renderProxyCollector, _device, _renderCommandScheduler);

	return mesh;
}

std::shared_ptr<PolygonMesh> CubemapRenderer::AddMesh(const Eigen::MatrixXd& vertices,
	const Eigen::MatrixXi& indices)
{
	RenderArrayType<std::vector<VkDescriptorSet>> solidDescriptorSets{ };
	for (std::size_t i = 0; i < solidDescriptorSets.size(); ++i)
	{
		solidDescriptorSets[i] = std::vector{ _matricesDescriptorSets[i], _lightsDescriptorSets[i], _objectDataDescriptorSets[i] };
	}

	RenderArrayType<std::vector<VkDescriptorSet>> wireframeDescriptorSets{ };
	for (std::size_t i = 0; i < wireframeDescriptorSets.size(); ++i)
	{
		wireframeDescriptorSets[i] = std::vector{ _matricesDescriptorSets[i], _objectDataDescriptorSets[i] };
	}

	const auto mesh = std::make_shared<PolygonMesh>(vertices,
		indices,
		MeshProxySolidPipeline{
			_staticMeshPipelineHandle,
			_staticMeshInfluenceMapPipelineHandle,
			solidDescriptorSets },
			MeshProxyWireframePipeline{ _pointsPipelineHandle, _edgesPipelineHandle, _polysPipelineHandle, wireframeDescriptorSets },
			false,
			WireframeRenderMode::None);
	mesh->CollectRenderProxy(_renderProxyCollector, _device, _renderCommandScheduler);

	return mesh;
}

void CubemapRenderer::RemoveMesh(const std::shared_ptr<PolygonMesh>& mesh)
{
	mesh->DestroyRenderProxy(_renderProxyCollector);
}

/*void CubemapRenderer::CreateViewportGridPipeline()
{
	// Blend the fragments.
	VkPipelineColorBlendAttachmentState colorBlendAttachment{ };
	colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachment.blendEnable = VK_TRUE;
	colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
	colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
	colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
	colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
	colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
	colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

	// Create the depth and stencil descriptions.
	VkPipelineDepthStencilStateCreateInfo depthStencil{ };
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_TRUE;
	depthStencil.depthWriteEnable = VK_TRUE;
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
	depthStencil.depthBoundsTestEnable = VK_FALSE;
	depthStencil.minDepthBounds = 0.0f;
	depthStencil.maxDepthBounds = 1.0f;
	depthStencil.stencilTestEnable = VK_FALSE;
	depthStencil.front = { };
	depthStencil.back = { };

	std::array descriptorSetLayouts{ _matricesLayout->GetReference() };

	_gridPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetColorBlendAttachments(std::span(&colorBlendAttachment, 1))
		.SetDepthStencilState(depthStencil)
		.SetDescriptorSetLayouts(std::span(descriptorSetLayouts))
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Grid.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/Grid.frag.spv")
		.Build();
}*/

/*void CubemapRenderer::CreateUniformBuffers()
{
	CHECK_VK_HANDLE(_device);

	_matricesUniformBuffers.resize(VulkanUtils::NumRenderFramesInFlight);
	_lightsUniformBuffers.resize(VulkanUtils::NumRenderFramesInFlight);
	_objectsDynamicUniformBuffers.resize(VulkanUtils::NumRenderFramesInFlight);

	for (std::size_t index = 0; index < VulkanUtils::NumRenderFramesInFlight; ++index)
	{
		FrameInfo frameInfo{ };
		_matricesUniformBuffers[index] = _resourceManager->CreateBufferAndMapMemory(std::span(&frameInfo, 1),
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);

		if (!_lightSources.empty())
		{
			_lightsUniformBuffers[index] = _resourceManager->CreateBufferAndMapMemory(std::span(_lightSources),
				VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
		}

		_objectsDynamicUniformBuffers[index] = _resourceManager->CreateBufferAndMapMemoryAligned(std::span(_objectsBufferData.Data(), TotalNumSceneObjects),
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
	}
	;
}*/

/*void CubemapRenderer::AllocateDescriptorSets()
{
	CHECK_VK_HANDLE(_device);

	RenderArrayType<VkDescriptorSetLayout> matricesDescriptorSetLayouts{ };
	std::ranges::fill(matricesDescriptorSetLayouts, _matricesLayout);
	_matricesDescriptorSets = _descriptorPool->AllocateDescriptorSets(matricesDescriptorSetLayouts);

	RenderArrayType<VkDescriptorSetLayout> objectDataDescriptorSetLayouts{ };
	std::ranges::fill(objectDataDescriptorSetLayouts, _objectDataLayout);
	_objectDataDescriptorSets = _descriptorPool->AllocateDescriptorSets(objectDataDescriptorSetLayouts);

	RenderArrayType<VkDescriptorSetLayout> lightsDescriptorSetLayouts{ };
	std::ranges::fill(lightsDescriptorSetLayouts, _lightsLayout);
	_lightsDescriptorSets = _descriptorPool->AllocateDescriptorSets(lightsDescriptorSetLayouts);

	for (std::size_t index = 0; index < VulkanUtils::NumRenderFramesInFlight; ++index)
	{
		std::array<VkWriteDescriptorSet, 3> descriptorWrites{ };

		VkDescriptorBufferInfo matricesBufferInfo{ };
		matricesBufferInfo.buffer = _matricesUniformBuffers[index]._deviceBuffer;
		matricesBufferInfo.offset = 0;
		matricesBufferInfo.range = sizeof(FrameInfo);

		descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[0].dstSet = _matricesDescriptorSets[index];
		descriptorWrites[0].dstBinding = 0;
		descriptorWrites[0].dstArrayElement = 0;
		descriptorWrites[0].descriptorCount = 1;
		descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		descriptorWrites[0].pBufferInfo = &matricesBufferInfo;

		VkDescriptorBufferInfo lightsBufferInfo{ };
		lightsBufferInfo.buffer = _lightsUniformBuffers[index]._deviceBuffer;
		lightsBufferInfo.offset = 0;
		lightsBufferInfo.range = sizeof(PointLightGPU) * _lightSources.size();

		descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[1].dstSet = _lightsDescriptorSets[index];
		descriptorWrites[1].dstBinding = 1;
		descriptorWrites[1].dstArrayElement = 0;
		descriptorWrites[1].descriptorCount = 1;
		descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		descriptorWrites[1].pBufferInfo = &lightsBufferInfo;

		VkDescriptorBufferInfo objectBufferInfo{ };
		objectBufferInfo.buffer = _objectsDynamicUniformBuffers[index]._deviceBuffer;
		objectBufferInfo.offset = 0;
		objectBufferInfo.range = _objectsBufferDynamicAlignment;

		descriptorWrites[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[2].dstSet = _objectDataDescriptorSets[index];
		descriptorWrites[2].dstBinding = 2;
		descriptorWrites[2].dstArrayElement = 0;
		descriptorWrites[2].descriptorCount = 1;
		descriptorWrites[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
		descriptorWrites[2].pBufferInfo = &objectBufferInfo;

		vkUpdateDescriptorSets(_device, static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
		
	}*/

/*void CubemapRenderer::CreateRenderPass(VkFormat format) {
	VkAttachmentDescription colorAttachment{};
	colorAttachment.format = format; // Use your Cubemap VkFormat
	//colorAttachment.samples = CubemapMsaaSamples; // Use your Cubemap's sample count (usually VK_SAMPLE_COUNT_1_BIT)
	colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	colorAttachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	VkAttachmentDescription depthAttachment{};
	depthAttachment.format = _device->FindDepthFormat(); // Use your depth format 
	//depthAttachment.samples = CubemapMsaaSamples;
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
}*/

/*void CubemapRenderer::CreateDescriptorSetLayout() {
	VkDescriptorSetLayoutBinding uboLayoutBinding{ };
	uboLayoutBinding.binding = 0;
	uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	uboLayoutBinding.descriptorCount = 1;
	uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

	VkDescriptorSetLayoutCreateInfo layoutInfo{ };
	layoutInfo.bindingCount = 1;
	layoutInfo.pBindings = &uboLayoutBinding;

	vkCreateDescriptorSetLayout(_device, &layoutInfo, nullptr, &_descriptorSetLayout);
}*/

/*void CubemapRenderer::CreateCubemapRenderPipeline()
{
	// Blend the fragments.
	VkPipelineColorBlendAttachmentState colorBlendAttachment{ };
	colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachment.blendEnable = VK_FALSE;
	// Create the depth and stencil descriptions.
	VkPipelineDepthStencilStateCreateInfo depthStencil{ };
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_TRUE;
	depthStencil.depthWriteEnable = VK_TRUE;
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
	depthStencil.depthBoundsTestEnable = VK_FALSE;
	depthStencil.minDepthBounds = 0.0f;
	depthStencil.maxDepthBounds = 1.0f;
	depthStencil.stencilTestEnable = VK_FALSE;
	depthStencil.front = { };
	depthStencil.back = { };
	std::array descriptorSetLayouts{ _matricesLayout->GetReference(),
		_objectDataLayout->GetReference() };
	_CubemapPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetColorBlendAttachments(std::span(&colorBlendAttachment, 1))
		.SetDepthStencilState(depthStencil)
		.SetDescriptorSetLayouts(std::span(descriptorSetLayouts))
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Cubemap.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/Cubemap.frag.spv")
		.Build();
}*/

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

/*void CubemapRenderer::CreateFramebuffer(uint32_t size) {
	for (uint32_t face = 0; face < 6; ++face) {
		VkImageView attachments[] = { _faceImageViews[face] };
		VkFramebufferCreateInfo framebufferInfo{ };
		framebufferInfo.renderPass = _renderPass;
		framebufferInfo.attachmentCount = 1;
		framebufferInfo.pAttachments = attachments;
		framebufferInfo.width = size;
		framebufferInfo.height = size;
		framebufferInfo.layers = 1;

		vkCreateFramebuffer(_device, &framebufferInfo, nullptr, &_faceFramebuffers[face]);
	}
}*/

/*void CubemapRenderer::CreateCommandPool(uint32_t queueFamilyIndex) {
	VkCommandPoolCreateInfo poolInfo{ };
	poolInfo.queueFamilyIndex = queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	vkCreateCommandPool(_device, &poolInfo, nullptr, &_commandPool);
}*/

/*void CubemapRenderer::CreateVertexBuffer(const std::vector<Vertex>& vertices) {
	// Create VkBuffer, allocate memory, copy data
	// Use VK_BUFFER_USAGE_VERTEX_BUFFER_BIT
}*/

/*void CubemapRenderer::CreateIndexBuffer(const std::vector<uint32_t>& indices) {
	// Create VkBuffer, allocate memory, copy data
	// Use VK_BUFFER_USAGE_INDEX_BUFFER_BIT
}*/

/*void CubemapRenderer::CreateUniformBuffer(VkDeviceSize bufferSize) {
	_uniformBuffer = _resourceManager->AllocateDeviceBuffer(
		bufferSize,
		VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);
	// Optionally map and store pointer for updates
}*/

/*void CubemapRenderer::CreateDescriptorPool() {
	VkDescriptorPoolSize poolSize{};
	poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	poolSize.descriptorCount = 6; // one per Cubemap face

	VkDescriptorPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	poolInfo.poolSizeCount = 1;
	poolInfo.pPoolSizes = &poolSize;
	poolInfo.maxSets = 6;

	vkCreateDescriptorPool(_device, &poolInfo, nullptr, &_descriptorPoolHandle);
}*/

/*void CubemapRenderer::CreateDescriptorPoolSets() {
	std::vector<VkDescriptorSetLayout> layouts(6, _descriptorSetLayout);
	VkDescriptorSetAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocInfo.descriptorPool = _descriptorPoolHandle;
	allocInfo.descriptorSetCount = 6;
	allocInfo.pSetLayouts = layouts.data();

	_descriptorSets.resize(6);
	vkAllocateDescriptorSets(_device, &allocInfo, _descriptorSets.data());

	for (size_t i = 0; i < 6; ++i) {
		VkDescriptorBufferInfo bufferInfo{};
		bufferInfo.buffer = _uniformBuffer._handle;
		bufferInfo.offset = 0;
		bufferInfo.range = sizeof(YourUniformStruct);

		VkWriteDescriptorSet descriptorWrite{};
		descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrite.dstSet = _descriptorSets[i];
		descriptorWrite.dstBinding = 0;
		descriptorWrite.dstArrayElement = 0;
		descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		descriptorWrite.descriptorCount = 1;
		descriptorWrite.pBufferInfo = &bufferInfo;

		vkUpdateDescriptorSets(_device, 1, &descriptorWrite, 0, nullptr);
	}
}*/

/*void CubemapRenderer::CreateCommandBuffer() {
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.commandPool = _commandPool;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandBufferCount = 6; // one per face

	_commandBuffers.resize(6);
	vkAllocateCommandBuffers(_device, &allocInfo, _commandBuffers.data());
}*/

/*void CubemapRenderer::CreateSyncObjects() {
	VkSemaphoreCreateInfo semaphoreInfo{};
	semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

	VkFenceCreateInfo fenceInfo{};
	fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

	for (int i = 0; i < 6; ++i) {
		vkCreateSemaphore(_device, &semaphoreInfo, nullptr, &_renderFinishedSemaphores[i]);
		vkCreateFence(_device, &fenceInfo, nullptr, &_inFlightFences[i]);
	}
}*/

