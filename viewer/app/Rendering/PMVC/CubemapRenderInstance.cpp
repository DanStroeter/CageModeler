#include <Rendering/PMVC/CubemapRenderInstance.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/ScreenPass.h>
#include <Editor/Light.h>
#include <cstddef>
#include <Rendering/PMVC/CpuComputeStrategy.h>
#include <Rendering/PMVC/GpuAtomicComputeStrategy.h>
#include <Rendering/PMVC/CubemapManager.h>
#include <Rendering/PMVC/DebugCubemapComputeStrategy.h>
#include <Rendering/PMVC/ScopedCmdBuffer.h>
#include <Rendering/PMVC/GpuSerialComputeStrategy.h>

CubemapRenderInstance::~CubemapRenderInstance() {
	//TODO: cleanup
}

CubemapRenderInstance::CubemapRenderInstance(CubemapManager& cubemapManager)
	: _cubemapManager(cubemapManager)
{
	_cubemapSize = 32;
	_format = VK_FORMAT_R32G32B32A32_SFLOAT;
	_computeType = ComputeType::CPU;
	Initalize();
}

CubemapRenderInstance::CubemapRenderInstance(CubemapManager& cubemapManager, int cubemapSize, VkFormat format, ComputeType computeType)
	: _cubemapManager(cubemapManager),
	_cubemapSize(cubemapSize),
	_format(format),
	_computeType(computeType)
{
	Initalize();
}

void CubemapRenderInstance::Initalize() {
	switch (_computeType)
	{
	case ComputeType::CPU:
		_computeStage = std::make_unique<CpuComputeStrategy>();
		break;
	case ComputeType::GPUATOMIC:
		_computeStage = std::make_unique<GpuAtomicComputeStrategy>(
			_cubemapManager._device,
			_cubemapManager._device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_cubemapManager._descriptorPool,
			_cubemapManager._resourceManager,
			_cubemapManager._renderPipelineManager,
			_cubemapManager._cageMesh,
			_cubemapManager._deformableMesh
		);
		break;
	case ComputeType::DEBUGCUBEMAPS:
		VkQueue transferQueue;
		vkGetDeviceQueue(
			_cubemapManager._device,
			_cubemapManager._device->GetQueueFamilies()._graphics.value(), 
			0,
			&transferQueue);
		_computeStage = std::make_unique<DebugCubemapComputeStrategy>(
			_cubemapManager._device,
			_cubemapManager._device->GetPhysicalDeviceHandle(),
			transferQueue,
			_cubemapManager._device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format);
		break;
	case ComputeType::GPUSERIAL:
		_computeStage = std::make_unique<GpuSerialComputeStrategy>(
			_cubemapManager._device,
			_cubemapManager._device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_cubemapManager._descriptorPool,
			_cubemapManager._resourceManager,
			_cubemapManager._renderPipelineManager,
			_cubemapManager._cageMesh,
			_cubemapManager._deformableMesh
		);
		break;
	}

	uint32_t graphicsQueueFamilyIndex = _cubemapManager._device->GetQueueFamilies()._graphics.value();
	CreateCommandPool(graphicsQueueFamilyIndex);

	_cubemapRenderUnit = CreateCubemapRenderUnit();

	const uint32_t targetCount = _computeStage->RequiredRenderTargetCount();
	_computeStage->Initialize();

	_cubemapRenderUnit.targets.reserve(targetCount);
	for (uint32_t i = 0; i < targetCount; ++i)
	{
		_cubemapRenderUnit.targets.push_back(CreateCubemapRenderTarget());
	}
	UpdateMatricesDescriptorSet();
	CreateSyncObjects();
	//_sphereWeightCalculator = SphereWeightCalculator();
	//_sphereWeightCalculator.SphereWeightInitialization(_cubemapSize, _cubemapManager._device, _cubemapManager._resourceManager, _graphicCommandPool);
}

CubemapRenderTarget CubemapRenderInstance::CreateCubemapRenderTarget() const
{
	CubemapRenderTarget target{};
	//target.currentLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	// ---------------------------------------------------------------------
	// Create cubemap color image
	// ---------------------------------------------------------------------
	VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
	imageInfo.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
	imageInfo.imageType = VK_IMAGE_TYPE_2D;
	imageInfo.format = _format;
	imageInfo.extent = { _cubemapSize, _cubemapSize, 1 };
	imageInfo.mipLevels = 1;
	imageInfo.arrayLayers = 6;
	imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	imageInfo.usage =
		VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
		VK_IMAGE_USAGE_SAMPLED_BIT |
		VK_IMAGE_USAGE_STORAGE_BIT |
		VK_IMAGE_LAYOUT_GENERAL |
		VK_IMAGE_USAGE_TRANSFER_SRC_BIT; //TODO: only added for debugging prints for image remove after done(needed for CPU compute?)
	imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	VK_CHECK(vkCreateImage(_cubemapManager._device, &imageInfo, nullptr, &target.cubemapImage));

	VkMemoryRequirements memReq{};
	vkGetImageMemoryRequirements(_cubemapManager._device, target.cubemapImage, &memReq);

	VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
	allocInfo.allocationSize = memReq.size;
	allocInfo.memoryTypeIndex = _cubemapManager.FindMemoryType(memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	VK_CHECK(vkAllocateMemory(_cubemapManager._device, &allocInfo, nullptr, &target.cubemapMemory));
	VK_CHECK(vkBindImageMemory(_cubemapManager._device, target.cubemapImage, target.cubemapMemory, 0));


	VkCommandBufferAllocateInfo allocInfoCB{
	.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
	.commandPool = _cubemapManager._graphicCommandPool, // or graphics pool
	.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
	.commandBufferCount = 1
	};

	VkCommandBuffer cmd;
	VK_CHECK(vkAllocateCommandBuffers(_cubemapManager._device, &allocInfoCB, &cmd));

	VkCommandBufferBeginInfo beginInfo{
	.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
	.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
	};

	VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

	// ---------------------------------------------------------------------
	// Create per-face color views
	// ---------------------------------------------------------------------
	for (uint32_t face = 0; face < 6; ++face)
	{
		VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
		viewInfo.image = target.cubemapImage;
		viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		viewInfo.format = _format;
		viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		viewInfo.subresourceRange.levelCount = 1;
		viewInfo.subresourceRange.baseArrayLayer = face;
		viewInfo.subresourceRange.layerCount = 1;

		VK_CHECK(vkCreateImageView(_cubemapManager._device, &viewInfo, nullptr, &target.faceViews[face]));
	}

	// ---------------------------------------------------------------------
	// Create cubemap view (for sampling)
	// ---------------------------------------------------------------------
	/*VkImageViewCreateInfo cubeViewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
	cubeViewInfo.image = target.cubemapImage;
	cubeViewInfo.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
	cubeViewInfo.format = _format;
	cubeViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	cubeViewInfo.subresourceRange.levelCount = 1;
	cubeViewInfo.subresourceRange.layerCount = 6;

	VK_CHECK(vkCreateImageView(_cubemapManager._device, &cubeViewInfo, nullptr, &target.cubemapView));*/
	VkImageViewCreateInfo cubeViewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
	cubeViewInfo.image = target.cubemapImage;
	cubeViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
	cubeViewInfo.format = _format;
	cubeViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	cubeViewInfo.subresourceRange.baseMipLevel = 0;
	cubeViewInfo.subresourceRange.levelCount = 1;
	cubeViewInfo.subresourceRange.baseArrayLayer = 0;
	cubeViewInfo.subresourceRange.layerCount = 6;

	VK_CHECK(vkCreateImageView(_cubemapManager._device, &cubeViewInfo, nullptr, &target.cubemapView));

	// ---------------------------------------------------------------------
	// Create depth image
	// ---------------------------------------------------------------------
	VkFormat depthFormat = _cubemapManager._device->FindDepthFormat();

	VkImageCreateInfo depthInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
	depthInfo.imageType = VK_IMAGE_TYPE_2D;
	depthInfo.format = depthFormat;
	depthInfo.extent = { _cubemapSize, _cubemapSize, 1 };
	depthInfo.mipLevels = 1;
	depthInfo.arrayLayers = 6;
	depthInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	depthInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	depthInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
	depthInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	VK_CHECK(vkCreateImage(_cubemapManager._device, &depthInfo, nullptr, &target.depthImage));

	vkGetImageMemoryRequirements(_cubemapManager._device, target.depthImage, &memReq);

	allocInfo.allocationSize = memReq.size;
	allocInfo.memoryTypeIndex =
		_cubemapManager.FindMemoryType(memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	VK_CHECK(vkAllocateMemory(_cubemapManager._device, &allocInfo, nullptr, &target.depthMemory));
	VK_CHECK(vkBindImageMemory(_cubemapManager._device, target.depthImage, target.depthMemory, 0));

	// ---------------------------------------------------------------------
	// Create per-face depth views
	// ---------------------------------------------------------------------
	for (uint32_t face = 0; face < 6; ++face)
	{
		VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
		viewInfo.image = target.depthImage;
		viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		viewInfo.format = depthFormat;
		viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
		viewInfo.subresourceRange.levelCount = 1;
		viewInfo.subresourceRange.baseArrayLayer = face;
		viewInfo.subresourceRange.layerCount = 1;

		VK_CHECK(vkCreateImageView(_cubemapManager._device, &viewInfo, nullptr, &target.depthViews[face]));
	}

	// ---------------------------------------------------------------------
	// Create framebuffers
	// ---------------------------------------------------------------------
	for (uint32_t face = 0; face < 6; ++face)
	{
		VkImageView attachments[2] = {
			target.faceViews[face],
			target.depthViews[face]
		};

		VkFramebufferCreateInfo fbInfo{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
		fbInfo.renderPass = _cubemapManager._renderPass;
		fbInfo.attachmentCount = 2;
		fbInfo.pAttachments = attachments;
		fbInfo.width = _cubemapSize;
		fbInfo.height = _cubemapSize;
		fbInfo.layers = 1;

		VK_CHECK(vkCreateFramebuffer(_cubemapManager._device, &fbInfo, nullptr, &target.framebuffers[face]));
	}

	return target;
}

CubemapRenderUnit CubemapRenderInstance::CreateCubemapRenderUnit() const
{
	VkDeviceSize matricesUBOSize = sizeof(CubemapMatricesUBO);
	CubemapRenderUnit unit{};

	// ------------------------------------------------------------
	// Create uniform buffer
	// ------------------------------------------------------------
	std::span<std::byte> sizeSpan(
		static_cast<std::byte*>(nullptr),
		matricesUBOSize
	);

	unit.matricesUBO =
		_cubemapManager._resourceManager->CreateBufferAndMapMemory(
			sizeSpan,
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		);

	// ------------------------------------------------------------
	// Allocate descriptor set
	// ------------------------------------------------------------
	VkDescriptorSetLayout layout = _cubemapManager._matricesLayout->GetReference();

	VkDescriptorSetAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	allocInfo.descriptorPool = _cubemapManager._descriptorPool;
	allocInfo.descriptorSetCount = 1;
	allocInfo.pSetLayouts = &layout;

	VK_CHECK(vkAllocateDescriptorSets(
		_cubemapManager._device,
		&allocInfo,
		&unit.matricesDescriptorSet
	));

	// ------------------------------------------------------------
	// Allocate command buffers (one per face)
	// ------------------------------------------------------------
	std::array<VkCommandBuffer, 6> buffers{};

	VkCommandBufferAllocateInfo alloc{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		.commandPool = _graphicCommandPool,
		.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
		.commandBufferCount = (uint32_t)buffers.size()
	};

	VK_CHECK(vkAllocateCommandBuffers(
		_cubemapManager._device,
		&alloc,
		buffers.data()));

	//unit.beginCmd = buffers[0];

	for (uint32_t i = 0; i < 6; ++i)
		unit.graphicsCmd[i] = buffers[i];

	//unit.endCmd = buffers[7];


	/*VkCommandBufferAllocateInfo cmdAllocInfo{
		VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO
	};
	cmdAllocInfo.commandPool = _graphicCommandPool;
	cmdAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	cmdAllocInfo.commandBufferCount = 6;

	VK_CHECK(vkAllocateCommandBuffers(
		_cubemapManager._device,
		&cmdAllocInfo,
		unit.graphicsCmd.data()
	));*/

	return unit;
}

void CubemapRenderInstance::CreateCommandPool(uint32_t queueFamilyIndex) {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_cubemapManager._device, &poolInfo, nullptr, &_graphicCommandPool) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create command pool!");
	}
}

void CubemapRenderInstance::UpdateMatricesDescriptorSet()
{
	VkDescriptorBufferInfo bufferInfo{};
	bufferInfo.buffer = _cubemapRenderUnit.matricesUBO._deviceBuffer;
	bufferInfo.offset = 0;
	bufferInfo.range = _cubemapRenderUnit.matricesUBO._allocatedSize;

	VkWriteDescriptorSet write{};
	write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
	write.dstSet = _cubemapRenderUnit.matricesDescriptorSet;
	write.dstBinding = 0;
	write.dstArrayElement = 0;
	write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	write.descriptorCount = 1;
	write.pBufferInfo = &bufferInfo;

	vkUpdateDescriptorSets(_cubemapManager._device, 1, &write, 0, nullptr);
}

void CubemapRenderInstance::RecordAndSubmitCubemapRender(
	uint32_t cubemapIdx,
	const glm::vec3& camPos,
	CubemapRenderTarget& target,
	VkSemaphore timeline,
	uint64_t signalValue)
{
	VkClearValue clearValues[2]{};
	clearValues[0].color = { {0.f, 0.f, 0.f, 1.f} };
	clearValues[1].depthStencil = { 1.f, 0 };

	VkCommandBufferBeginInfo beginInfo{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
		.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
	};

	const PipelineObject& pipelineObj =
		_cubemapManager._renderPipelineManager->GetPipelineObject(
			_cubemapManager._cubemapPipelineHandle);

	// ---------------------------------------------------------------------
	// Update shared UBO (outside render loop)
	// ---------------------------------------------------------------------
	const uint32_t numTriangles =
		static_cast<uint32_t>(_cubemapManager._cageMesh._faces.size() / 3);

	CubemapMatricesUBO ubo{};
	ubo.invNumTriangles = 1.0f / float(numTriangles);
	std::memcpy(
		_cubemapRenderUnit.matricesUBO._mappedData,
		&ubo,
		sizeof(ubo));

	// =====================================================================
	// GRAPHICS CMDS — one per face
	// =====================================================================
	for (uint32_t face = 0; face < 6; ++face)
	{
		VkCommandBuffer cmd = _cubemapRenderUnit.graphicsCmd[face];
		VK_CHECK(vkResetCommandBuffer(cmd, 0));
		VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

		CubemapPushConstants push{};
		push.proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 1000.0f);
		push.proj[1][1] *= -1.0f;
		push.view = _cubemapManager.ComputeCubemapViewMatrix(face, camPos);

		vkCmdPushConstants(
			cmd,
			pipelineObj._pipelineLayout,
			VK_SHADER_STAGE_VERTEX_BIT,
			0,
			sizeof(push),
			&push);

		VkRenderPassBeginInfo rpInfo{
			.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO,
			.renderPass = _cubemapManager._renderPass,
			.framebuffer = target.framebuffers[face],
			.renderArea = {{0, 0}, {_cubemapSize, _cubemapSize}},
			.clearValueCount = 2,
			.pClearValues = clearValues
		};

		vkCmdBeginRenderPass(cmd, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);
		vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineObj._handle);

		VkBuffer vb[] = { _cubemapManager._vertexBuffer._deviceBuffer };
		VkDeviceSize offs[] = { 0 };
		vkCmdBindVertexBuffers(cmd, 0, 1, vb, offs);
		vkCmdBindIndexBuffer(
			cmd,
			_cubemapManager._indexBuffer._deviceBuffer,
			0,
			VK_INDEX_TYPE_UINT32);

		vkCmdBindDescriptorSets(
			cmd,
			VK_PIPELINE_BIND_POINT_GRAPHICS,
			pipelineObj._pipelineLayout,
			0, 1,
			&_cubemapRenderUnit.matricesDescriptorSet,
			0, nullptr);

		vkCmdDrawIndexed(
			cmd,
			static_cast<uint32_t>(_cubemapManager._cageMesh._faces.size()),
			1, 0, 0, 0);

		vkCmdEndRenderPass(cmd);
		VK_CHECK(vkEndCommandBuffer(cmd));
	}

	// =====================================================================
	// SUBMIT
	// =====================================================================
	std::array<VkCommandBufferSubmitInfo, 6> cmdInfos{};

	for (uint32_t i = 0; i < 6; ++i) {
		cmdInfos[i] = {
			VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
			nullptr,
			_cubemapRenderUnit.graphicsCmd[i]
		};
	}

	VkSemaphoreSubmitInfo signalInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = timeline,
		.value = signalValue,
		.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT
	};

	VkSubmitInfo2 submit{
		.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2,
		.commandBufferInfoCount = uint32_t(cmdInfos.size()),
		.pCommandBufferInfos = cmdInfos.data(),
		.signalSemaphoreInfoCount = 1,
		.pSignalSemaphoreInfos = &signalInfo
	};

	VkQueue graphicsQueue;
	vkGetDeviceQueue(
		_cubemapManager._device,
		_cubemapManager._device->GetQueueFamilies()._graphics.value(),
		0,
		&graphicsQueue);

	VK_CHECK(vkQueueSubmit2(graphicsQueue, 1, &submit, VK_NULL_HANDLE));
}

void CubemapRenderInstance::CreateSyncObjects()
{
	const uint32_t slotCount =
		static_cast<uint32_t>(_cubemapRenderUnit.targets.size());

	_timelines.resize(slotCount);
	_slotDoneValue.assign(slotCount, 0);

	for (uint32_t i = 0; i < slotCount; ++i)
	{
		VkSemaphoreTypeCreateInfo typeInfo{
			.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO,
			.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE,
			.initialValue = 0
		};

		VkSemaphoreCreateInfo semInfo{
			.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO,
			.pNext = &typeInfo
		};

		VK_CHECK(vkCreateSemaphore(
			_cubemapManager._device,
			&semInfo,
			nullptr,
			&_timelines[i]
		));
	}
}

std::vector<glm::vec3> CubemapRenderInstance::BuildDeformableVertexPositions() const
{
	std::vector<glm::vec3> vertices;
	vertices.reserve(_cubemapManager._deformableMesh._vertices.rows());

	for (int i = 0; i < _cubemapManager._deformableMesh._vertices.rows(); ++i)
	{
		const auto& v = _cubemapManager._deformableMesh._vertices.row(i);

		vertices.emplace_back(
			static_cast<float>(v(0)),
			static_cast<float>(v(1)),
			static_cast<float>(v(2))
		);
	}

	return vertices;
}

void SphereWeightCalculator::SphereWeightInitialization(uint32_t size, RenderResourceRef<Device> device, std::shared_ptr<RenderResourceManager> resourceManager, VkCommandPool commandPool) {
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
				] = ComputeSphereWeight(x, y, faceSize, face);
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

	vkCreateImage(device, &img, nullptr, &_solidAngleImage);

	VkMemoryRequirements memReq{};
	vkGetImageMemoryRequirements(device, _solidAngleImage, &memReq);

	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(device->GetPhysicalDeviceHandle(), &memProperties);

	std::optional<uint32_t> memtype;

	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
		if (memReq.memoryTypeBits & (1 << i)) {
			if (memProperties.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) {
				memtype = i;
				break;
			}
		}
	}

	// Fallback: accept any compatible memory
	if (!memtype.has_value()) {
		for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
			if (memReq.memoryTypeBits & (1 << i)) {
				memtype = i;
				break;
			}
		}
	}

	if (!memtype.has_value()) {
		throw std::runtime_error("Failed to find ANY compatible memory type for solid angle image!");
	}


	VkMemoryAllocateInfo alloc{};
	alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	alloc.allocationSize = memReq.size;
	alloc.memoryTypeIndex = memtype.value();

	vkAllocateMemory(device, &alloc, nullptr, &_solidAngleMemory);
	vkBindImageMemory(device, _solidAngleImage, _solidAngleMemory, 0);

	// ------------------------------------------------------------
	// 3) Upload via staging buffer
	// ------------------------------------------------------------
	const VkDeviceSize uploadBytes = weights.size() * sizeof(float);

	auto staging = resourceManager->CreateBufferAndCopy(
		std::span<const float>(weights.data(), weights.size()),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);


	ScopedCmdBuffer scoped(device, commandPool);
	VkCommandBuffer cmd = scoped.Get();

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

	VkQueue transferQueue;
	vkGetDeviceQueue(
		device,
		device->GetQueueFamilies()._graphics.value(),
		0,
		&transferQueue);
	scoped.SubmitAndWait(transferQueue);

	// ------------------------------------------------------------
	// 4) Create 2D-array image view (for compute)
	// ------------------------------------------------------------
	VkImageViewCreateInfo view{};
	view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	view.image = _solidAngleImage;
	view.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
	view.format = VK_FORMAT_R32_SFLOAT;
	view.subresourceRange = range;

	vkCreateImageView(device, &view, nullptr, &_solidAngleArrayView);

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

	vkCreateSampler(device, &samp, nullptr, &_solidAngleSampler);
	double sum = 0.0;
	for (float w : weights) sum += w;
	//LOG_DEBUG("Total solid angle = " + std::to_string(sum));
}

float SphereWeightCalculator::ComputeSphereWeight(
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
}

void CubemapRenderInstance::ComputeCoordinatesGPUSerial(
	const CubemapWorkRange& range, Eigen::MatrixXd& weights)
{
	LOG_DEBUG("ComputeCoordinatesGPUSerial: range.first={}, range.count={}",
		range.first, range.count);

	auto* computeStage =
		static_cast<GpuSerialComputeStrategy*>(_computeStage.get());

	const auto vertices = BuildDeformableVertexPositions();

	const uint32_t end = std::min<uint32_t>(
		range.first + range.count,
		static_cast<uint32_t>(vertices.size())
	);

	// ---- single slot ----
	constexpr uint32_t slot = 0;
	CubemapRenderTarget& target =
		_cubemapRenderUnit.targets[slot];

	VkSemaphore timeline = _timelines[slot];
	uint64_t& timelineValue = _slotDoneValue[slot];

	for (uint32_t cubemapIdx = range.first; cubemapIdx < end; ++cubemapIdx)
	{
		LOG_DEBUG("[Serial] cubemapIdx={}", cubemapIdx);

		// ------------------------------------------------------------
		// 1) Render
		// ------------------------------------------------------------
		uint64_t renderDone = ++timelineValue;

		LOG_DEBUG("  [Render] signal={}", renderDone);

		RecordAndSubmitCubemapRender(
			cubemapIdx,
			vertices[cubemapIdx],
			target,
			timeline,
			renderDone
		);

		// ------------------------------------------------------------
		// 2) Compute (waits on renderDone)
		// ------------------------------------------------------------
		uint64_t computeDone = ++timelineValue;

		LOG_DEBUG("  [Compute] wait={}, signal={}",
			renderDone, computeDone);

		computeStage->DispatchAfterRender(
			cubemapIdx,
			slot,
			timeline,
			renderDone,
			computeDone,
			target
		);

		// ------------------------------------------------------------
		// 3) Copy (waits on computeDone)
		// ------------------------------------------------------------
		uint64_t copyDone = ++timelineValue;

		LOG_DEBUG("  [Copy] wait={}, signal={}",
			computeDone, copyDone);

		computeStage->SubmitReadbackCopy(
			slot,
			timeline,
			computeDone,
			copyDone
		);
		
		computeStage->ConsumeSlot(
			cubemapIdx,
			slot,
			timeline,
			copyDone
		);

		// ------------------------------------------------------------
		// 4) CPU wait (serialization point)
		// ------------------------------------------------------------
		VkSemaphoreWaitInfo waitInfo{};
		waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		waitInfo.semaphoreCount = 1;
		waitInfo.pSemaphores = &timeline;
		waitInfo.pValues = &computeDone;

		vkWaitSemaphores(
			_cubemapManager._device,
			&waitInfo,
			UINT64_MAX
		);
	}
	weights = computeStage->Readback();
	
	LOG_DEBUG("ComputeCoordinatesGPUSerial: done");
}
