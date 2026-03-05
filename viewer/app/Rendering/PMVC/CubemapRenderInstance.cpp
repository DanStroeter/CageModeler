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
#include <Rendering/PMVC/GpuMassivelyParallelComputeStrategy.h>
#include <Mesh/Operations/MeshWeightsParams.h>

CubemapRenderInstance::~CubemapRenderInstance() {
	//TODO: cleanup
}

CubemapRenderInstance::CubemapRenderInstance()
{
	_cubemapSize = 32;
	_format = VK_FORMAT_R32G32B32A32_SFLOAT;
	_deformationType = DeformationType::PMVCSerialNoOffset;
	Initialize();
}

CubemapRenderInstance::CubemapRenderInstance(
	CubemapManager& cubemapManager,
	int cubemapSize,
	VkFormat format,
	DeformationType deformationType,

	RenderResourceRef<Device> device,
	RenderResourceRef<DescriptorPool> descriptorPool,
	std::shared_ptr<RenderResourceManager> resourceManager,
	std::shared_ptr<RenderPipelineManager> renderPipelineManager,

	EigenMesh cageMesh,
	EigenMesh deformableMesh,

	VkCommandPool graphicsCommandPool,
	VkRenderPass renderPass,
	VkRenderPass renderPassCpu,
	PipelineHandle cubemapPipelineHandle,
	PipelineHandle cubemapPipelineHandleCpu,

	RenderResourceRef<DescriptorSetLayout> matricesLayout,

	MemoryMappedBuffer indexBuffer,
	MemoryMappedBuffer vertexBuffer
): _cubemapSize(cubemapSize)
, _format(format)
, _deformationType(deformationType)
, _device(std::move(device))
, _descriptorPool(std::move(descriptorPool))
, _resourceManager(std::move(resourceManager))
, _renderPipelineManager(std::move(renderPipelineManager))
, _cageMesh(std::move(cageMesh))
, _deformableMesh(std::move(deformableMesh))
, _graphicsCommandPool(graphicsCommandPool)
, _renderPass(renderPass)
, _renderPassCpu(renderPassCpu)
, _cubemapPipelineHandle(cubemapPipelineHandle)
, _matricesLayout(std::move(matricesLayout))
, _indexBuffer(std::move(indexBuffer))
, _vertexBuffer(std::move(vertexBuffer))
, _cubemapPipelineHandleCpu(cubemapPipelineHandleCpu)
{
	Initialize();
}

void CubemapRenderInstance::Initialize() {
	_offset = DeformationTypeHelpers::PMVCOffset(_deformationType);

	if (DeformationType::PMVCSerialOffset == _deformationType || DeformationType::PMVCSerialNoOffset == _deformationType) {
		_computeStage = std::make_unique<GpuSerialComputeStrategy>(
			_device,
			_device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_descriptorPool,
			_resourceManager,
			_renderPipelineManager,
			_cageMesh,
			_deformableMesh, 
			_offset
		);
	}
	else if (DeformationType::PMVCRingOffset == _deformationType || DeformationType::PMVCRingNoOffset == _deformationType) {
		_computeStage = std::make_unique<GpuAtomicComputeStrategy>(
			_device,
			_device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_descriptorPool,
			_resourceManager,
			_renderPipelineManager,
			_cageMesh,
			_deformableMesh,
			_offset
		);
	}
	else if (DeformationType::PMVCAllOffset == _deformationType || DeformationType::PMVCAllNoOffset == _deformationType) {
		_computeStage = std::make_unique<GpuMPComputeStrategy>(
			_device,
			_device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_descriptorPool,
			_resourceManager,
			_renderPipelineManager,
			_cageMesh,
			_deformableMesh,
			_offset
		);
	}
	else if (DeformationType::PMVCCpuOffset == _deformationType || DeformationType::PMVCCpuNoOffset == _deformationType) {
		_computeStage = std::make_unique<CpuComputeStrategy>(
			_device,
			_device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_descriptorPool,
			_resourceManager,
			_renderPipelineManager,
			_cageMesh,
			_deformableMesh,
			_offset
		);
	}

	uint32_t graphicsQueueFamilyIndex = _device->GetQueueFamilies()._graphics.value();
	CreateCommandPool(graphicsQueueFamilyIndex);

	const uint32_t targetCount = _computeStage->RequiredRenderTargetCount();
	_computeStage->Initialize();

	_cubemapRenderUnit.targets.reserve(targetCount);
	for (uint32_t i = 0; i < targetCount; ++i)
	{
		_cubemapRenderUnit.targets.push_back(CreateCubemapRenderTarget());
	}

	_cubemapRenderUnit = CreateCubemapRenderUnit();
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
		VK_IMAGE_USAGE_TRANSFER_SRC_BIT; //TODO: only added for debugging prints for image remove after done(needed for CPU compute?)
	imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	VK_CHECK(vkCreateImage(_device, &imageInfo, nullptr, &target.cubemapImage));

	VkMemoryRequirements memReq{};
	vkGetImageMemoryRequirements(_device, target.cubemapImage, &memReq);

	VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
	allocInfo.allocationSize = memReq.size;
	allocInfo.memoryTypeIndex = FindMemoryType(memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	VK_CHECK(vkAllocateMemory(_device, &allocInfo, nullptr, &target.cubemapMemory));
	VK_CHECK(vkBindImageMemory(_device, target.cubemapImage, target.cubemapMemory, 0));


	VkCommandBufferAllocateInfo allocInfoCB{
	.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
	.commandPool = _graphicCommandPool, // or graphics pool
	.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
	.commandBufferCount = 1
	};

	VkCommandBuffer cmd;
	VK_CHECK(vkAllocateCommandBuffers(_device, &allocInfoCB, &cmd));

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

		VK_CHECK(vkCreateImageView(_device, &viewInfo, nullptr, &target.faceViews[face]));
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

	VK_CHECK(vkCreateImageView(_device, &cubeViewInfo, nullptr, &target.cubemapView));

	// ---------------------------------------------------------------------
	// Create depth image
	// ---------------------------------------------------------------------
	VkFormat depthFormat = _device->FindDepthFormat();

	VkImageCreateInfo depthInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
	depthInfo.imageType = VK_IMAGE_TYPE_2D;
	depthInfo.format = depthFormat;
	depthInfo.extent = { _cubemapSize, _cubemapSize, 1 };
	depthInfo.mipLevels = 1;
	depthInfo.arrayLayers = 6;
	depthInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	depthInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	depthInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT |
		VK_IMAGE_USAGE_SAMPLED_BIT;
	depthInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	VK_CHECK(vkCreateImage(_device, &depthInfo, nullptr, &target.depthImage));

	vkGetImageMemoryRequirements(_device, target.depthImage, &memReq);

	allocInfo.allocationSize = memReq.size;
	allocInfo.memoryTypeIndex =
		FindMemoryType(memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	VK_CHECK(vkAllocateMemory(_device, &allocInfo, nullptr, &target.depthMemory));
	VK_CHECK(vkBindImageMemory(_device, target.depthImage, target.depthMemory, 0));

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

		VK_CHECK(vkCreateImageView(_device, &viewInfo, nullptr, &target.depthViews[face]));
	}

	VkImageViewCreateInfo depthViewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
	depthViewInfo.image = target.depthImage;
	depthViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
	depthViewInfo.format = depthFormat;
	depthViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
	depthViewInfo.subresourceRange.baseMipLevel = 0;
	depthViewInfo.subresourceRange.levelCount = 1;
	depthViewInfo.subresourceRange.baseArrayLayer = 0;
	depthViewInfo.subresourceRange.layerCount = 6;

	VK_CHECK(vkCreateImageView(_device, &depthViewInfo, nullptr, &target.depthView));

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
		if(_deformationType == DeformationType::PMVCCpuNoOffset || _deformationType ==  DeformationType::PMVCCpuOffset)
			fbInfo.renderPass = _renderPassCpu;
		else
			fbInfo.renderPass = _renderPass;
		fbInfo.attachmentCount = 2;
		fbInfo.pAttachments = attachments;
		fbInfo.width = _cubemapSize;
		fbInfo.height = _cubemapSize;
		fbInfo.layers = 1;

		VK_CHECK(vkCreateFramebuffer(_device, &fbInfo, nullptr, &target.framebuffers[face]));
	}

	return target;
}

CubemapRenderUnit CubemapRenderInstance::CreateCubemapRenderUnit() const
{
	VkDeviceSize matricesUBOSize = sizeof(CubemapMatricesUBO);
	CubemapRenderUnit unit{};
	unit.targets = _cubemapRenderUnit.targets;
	// ------------------------------------------------------------
	// Create uniform buffer
	// ------------------------------------------------------------
	std::span<std::byte> sizeSpan(
		static_cast<std::byte*>(nullptr),
		matricesUBOSize
	);

	unit.matricesUBO =
		_resourceManager->CreateBufferAndMapMemory(
			sizeSpan,
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		);

	// ------------------------------------------------------------
	// Allocate descriptor set
	// ------------------------------------------------------------
	VkDescriptorSetLayout layout = _matricesLayout->GetReference();

	VkDescriptorSetAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	allocInfo.descriptorPool = _descriptorPool;
	allocInfo.descriptorSetCount = 1;
	allocInfo.pSetLayouts = &layout;

	VK_CHECK(vkAllocateDescriptorSets(
		_device,
		&allocInfo,
		&unit.matricesDescriptorSet
	));

	// ------------------------------------------------------------
	// Allocate command buffers (one per face)
	// ------------------------------------------------------------
	/*std::array<VkCommandBuffer, 6> buffers{};

	VkCommandBufferAllocateInfo alloc{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		.commandPool = _graphicCommandPool,
		.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
		.commandBufferCount = (uint32_t)buffers.size()
	};

	VK_CHECK(vkAllocateCommandBuffers(
		_device,
		&alloc,
		buffers.data()));

	//unit.beginCmd = buffers[0];*/
	const uint32_t targetCount =
		static_cast<uint32_t>(unit.targets.size());
	unit.graphicsCmdPerTarget.resize(targetCount);

	/*for (uint32_t i = 0; i < 6; ++i)
		unit.graphicsCmd[i] = buffers[i];
		*/
	//unit.endCmd = buffers[7];

	if (targetCount > 0)
	{
		std::vector<VkCommandBuffer> flatBuffers(targetCount * 6);

		VkCommandBufferAllocateInfo alloc{
			.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
			.commandPool = _graphicCommandPool,
			.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
			.commandBufferCount = static_cast<uint32_t>(flatBuffers.size())
		};
		VK_CHECK(vkAllocateCommandBuffers(
			_device,
			&alloc,
			flatBuffers.data()));
		for (uint32_t target = 0; target < targetCount; ++target)
		{
			for (uint32_t face = 0; face < 6; ++face)
			{
				unit.graphicsCmdPerTarget[target][face] =
					flatBuffers[target * 6 + face];
			}
		}
	}
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

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_graphicCommandPool) != VK_SUCCESS) {
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

	vkUpdateDescriptorSets(_device, 1, &write, 0, nullptr);
}

void CubemapRenderInstance::RecordAndSubmitCubemapRender(
	uint32_t cubemapIdx,
	uint32_t targetIndex,
	const glm::vec3& camPos,
	CubemapRenderTarget& target,
	VkSemaphore timeline,
	uint64_t signalValue)
{
	assert(targetIndex < _cubemapRenderUnit.graphicsCmdPerTarget.size());

	VkClearValue clearValues[2]{};
	clearValues[0].color = { {0.f, 0.f, 0.f, 1.f} };
	clearValues[1].depthStencil = { 1.f, 0 };

	VkCommandBufferBeginInfo beginInfo{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
		.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
	};
	
	PipelineHandle pipelineHandle = _cubemapPipelineHandle;
	if (_deformationType == DeformationType::PMVCCpuNoOffset || _deformationType == DeformationType::PMVCCpuOffset)
		pipelineHandle = _cubemapPipelineHandleCpu;

	const PipelineObject& pipelineObj =
		_renderPipelineManager->GetPipelineObject(
			pipelineHandle);

	// ---------------------------------------------------------------------
	// Update shared UBO (outside render loop)
	// ---------------------------------------------------------------------
	const uint32_t numTriangles =
		static_cast<uint32_t>(_cageMesh._faces.size() / 3);

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
		VkCommandBuffer cmd = _cubemapRenderUnit.graphicsCmdPerTarget[targetIndex][face];
		VK_CHECK(vkResetCommandBuffer(cmd, 0));
		VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

		CubemapPushConstants push{};
		push.proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.1f, 1000.0f);
		push.proj[1][1] *= -1.0f;
		push.view = ComputeCubemapViewMatrix(face, camPos);

		vkCmdPushConstants(
			cmd,
			pipelineObj._pipelineLayout,
			VK_SHADER_STAGE_VERTEX_BIT,
			0,
			sizeof(push),
			&push);

		VkRenderPass renderPass = _renderPass;
		if (_deformationType == DeformationType::PMVCCpuNoOffset || _deformationType == DeformationType::PMVCCpuOffset)
			renderPass = _renderPassCpu;

		VkRenderPassBeginInfo rpInfo{
			.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO,
			.renderPass = renderPass,
			.framebuffer = target.framebuffers[face],
			.renderArea = {{0, 0}, {_cubemapSize, _cubemapSize}},
			.clearValueCount = 2,
			.pClearValues = clearValues
		};

		vkCmdBeginRenderPass(cmd, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);
		vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineObj._handle);

		VkBuffer vb[] = { _vertexBuffer._deviceBuffer };
		VkDeviceSize offs[] = { 0 };
		vkCmdBindVertexBuffers(cmd, 0, 1, vb, offs);
		vkCmdBindIndexBuffer(
			cmd,
			_indexBuffer._deviceBuffer,
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
			static_cast<uint32_t>(_cageMesh._faces.size()),
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
			_cubemapRenderUnit.graphicsCmdPerTarget[targetIndex][i]
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
		_device,
		_device->GetQueueFamilies()._graphics.value(),
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
			_device,
			&semInfo,
			nullptr,
			&_timelines[i]
		));
	}
}

std::vector<glm::vec3> CubemapRenderInstance::BuildDeformableVertexPositions() const
{
	std::vector<glm::vec3> vertices;
	vertices.reserve(_deformableMesh._vertices.rows());

	for (int i = 0; i < _deformableMesh._vertices.rows(); ++i)
	{
		const auto& v = _deformableMesh._vertices.row(i);

		vertices.emplace_back(
			static_cast<float>(v(0)),
			static_cast<float>(v(1)),
			static_cast<float>(v(2))
		);
	}

	return vertices;
}

static double SecondsSince(
	const std::chrono::high_resolution_clock::time_point& start)
{
	using namespace std::chrono;
	return duration<double>(high_resolution_clock::now() - start).count();
}

glm::mat4 CubemapRenderInstance::ComputeCubemapViewMatrix(uint32_t faceIndex, const glm::vec3& pos)
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

uint32_t CubemapRenderInstance::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const
{
	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(_device->GetPhysicalDeviceHandle(), &memProperties);

	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
		if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
			return i;
	}
	throw std::runtime_error("Failed to find suitable memory type!");
}

void CubemapRenderInstance::ComputeCoordinatesGPUMP(
	const CubemapWorkRange& range,
	Eigen::MatrixXd& weights)
{
	auto waitStart = std::chrono::high_resolution_clock::now();
	auto waitTemp = std::chrono::high_resolution_clock::now();
	LOG_DEBUG("ComputeCoordinatesGPUAtomic (batched, no slots)");

	auto* computeStage =
		static_cast<GpuMPComputeStrategy*>(_computeStage.get());

	const auto vertices = BuildDeformableVertexPositions();

	const uint32_t end = std::min<uint32_t>(
		range.first + range.count,
		static_cast<uint32_t>(vertices.size())
	);

	const uint32_t cubemapCount = end - range.first;

	VkSemaphore timeline = _timelines[0];
	uint64_t& timelineValue = _slotDoneValue[0];

	if (timelineValue > 0)
	{
		VkSemaphoreWaitInfo waitInfo{};
		waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		waitInfo.semaphoreCount = 1;
		waitInfo.pSemaphores = &timeline;
		waitInfo.pValues = &timelineValue;
		VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
	}

	if (timelineValue > 0)
	{
		VkSemaphoreWaitInfo waitInfo{};
		waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		waitInfo.semaphoreCount = 1;
		waitInfo.pSemaphores = &timeline;
		waitInfo.pValues = &timelineValue;
		VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
	}
	// ============================================================
	// Phase 1: Render ALL cubemaps
	// ============================================================
	uint64_t renderDone = timelineValue;

	double waitSeconds = SecondsSince(waitTemp);
	LOG_DEBUG("Init complete");
	LOG_DEBUG(
		"Total time={:.6f} ms",
		waitSeconds * 1000.0);

	waitTemp = std::chrono::high_resolution_clock::now();
	for (uint32_t i = 0; i < cubemapCount; ++i)
	{
		const uint32_t cubemapIdx = range.first + i;
		CubemapRenderTarget& target =
			_cubemapRenderUnit.targets[i];
		renderDone = ++timelineValue;
		RecordAndSubmitCubemapRender(
			cubemapIdx,
			i,
			vertices[cubemapIdx],
			target,
			timeline,
			renderDone
		);
	}
	LOG_DEBUG("Cubemap Renders submitted");
	waitSeconds = SecondsSince(waitTemp);
	LOG_DEBUG(
		"Total time={:.6f} ms",
		waitSeconds * 1000.0);
	// ============================================================
	// Phase 2: Record ALL compute command buffers
	// ============================================================
	waitTemp = std::chrono::high_resolution_clock::now();
	for (uint32_t i = 0; i < cubemapCount; ++i)
	{
		computeStage->RecordCompute(
			i,
			_cubemapRenderUnit.targets[i],
			range.first + i
		);
	}
	LOG_DEBUG("Cubemap Renders recorded");
	waitSeconds = SecondsSince(waitTemp);
	LOG_DEBUG(
		"Total time={:.6f} ms",
		waitSeconds * 1000.0);

	uint64_t computeDone = ++timelineValue;
	waitTemp = std::chrono::high_resolution_clock::now();
	computeStage->SubmitAllComputes(
		timeline,
		renderDone,
		timeline,
		computeDone
	);
	LOG_DEBUG("Computes submitted");
	waitSeconds = SecondsSince(waitTemp);
	LOG_DEBUG(
		"Total time={:.6f} ms",
		waitSeconds * 1000.0);
	// ============================================================
	// Phase 3: Submit ALL readback copies
	// ============================================================
	uint64_t copyDone = ++timelineValue;
	waitTemp = std::chrono::high_resolution_clock::now();
	computeStage->SubmitAllReadbackCopies(
		timeline,
		computeDone,
		timeline,
		copyDone
	);
	LOG_DEBUG("Readback submitted");
	waitSeconds = SecondsSince(waitTemp);
	LOG_DEBUG(
		"Total time={:.6f} ms",
		waitSeconds * 1000.0);
	// ============================================================
	// Phase 4: Wait ONCE (everything complete)
	// ============================================================
	VkSemaphoreWaitInfo wait{};
	wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
	wait.semaphoreCount = 1;
	wait.pSemaphores = &timeline;
	wait.pValues = &copyDone;

	vkWaitSemaphores(_device, &wait, UINT64_MAX);
	//timelineValue = copyDone;
	timelineValue = copyDone;
	// ============================================================
	// Phase 5: Consume ALL slots (CPU-side)
	// ============================================================
	waitTemp = std::chrono::high_resolution_clock::now();
	computeStage->ConsumeAllSlots();
	LOG_DEBUG("Slots consumed");
	waitSeconds = SecondsSince(waitTemp);
	LOG_DEBUG(
		"Total time={:.6f} ms",
		waitSeconds * 1000.0);
	waitTemp = std::chrono::high_resolution_clock::now();
	weights = computeStage->Readback();
	LOG_DEBUG("Readback complete");
	waitSeconds = SecondsSince(waitTemp);
	LOG_DEBUG(
		"Total time={:.6f} ms",
		waitSeconds * 1000.0);
	LOG_DEBUG("ComputeCoordinatesGPUAtomic (batched): done");
	waitSeconds = SecondsSince(waitStart);
	LOG_DEBUG(
		"Total time={:.6f} ms",
		waitSeconds * 1000.0);
}

void CubemapRenderInstance::ComputeCoordinatesGPUAtomic(
	const CubemapWorkRange& range, Eigen::MatrixXd& weights)
{
	LOG_DEBUG("ComputeCoordinatesGPUSerial (pipelined): range.first={}, range.count={}",
		range.first, range.count);

	auto* computeStage =
		static_cast<GpuAtomicComputeStrategy*>(_computeStage.get());

	const auto vertices = BuildDeformableVertexPositions();

	const uint32_t end = std::min<uint32_t>(
		range.first + range.count,
		static_cast<uint32_t>(vertices.size())
	);

	constexpr uint32_t SlotCount = 3; // ⭐ sweet spot
	static_assert(SlotCount > 0);

	// ------------------------------------------------------------
	// Main submission loop (NO per-iteration CPU wait)
	// ------------------------------------------------------------
	double totalSlotWaitMs = 0.0;
	for (uint32_t cubemapIdx = range.first; cubemapIdx < end; ++cubemapIdx)
	{
		const uint32_t slot = cubemapIdx % SlotCount;

		CubemapRenderTarget& target =
			_cubemapRenderUnit.targets[slot];

		VkSemaphore timeline = _timelines[slot];
		uint64_t& timelineValue = _slotDoneValue[slot];

		LOG_DEBUG("[Pipelined] cubemapIdx={}, slot={}", cubemapIdx, slot);

		// --------------------------------------------------------
		// 0) Wait ONLY if slot is being reused
		// --------------------------------------------------------
		if (timelineValue > 0)
		{
			VkSemaphoreWaitInfo waitInfo{};
			waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
			waitInfo.semaphoreCount = 1;
			waitInfo.pSemaphores = &timeline;
			waitInfo.pValues = &timelineValue;

			auto waitStart = std::chrono::high_resolution_clock::now();

			vkWaitSemaphores(
				_device,
				&waitInfo,
				UINT64_MAX
			);

			double waitSeconds = SecondsSince(waitStart);

			LOG_DEBUG(
				"  [SlotWait] slot={}, value={}, wait={:.6f} ms",
				slot,
				timelineValue,
				waitSeconds * 1000.0
			);
			totalSlotWaitMs += waitSeconds * 1000.0;
		}

		// --------------------------------------------------------
		// 1) Render
		// --------------------------------------------------------
		uint64_t renderDone = ++timelineValue;

		LOG_DEBUG("  [Render] signal={}", renderDone);

		RecordAndSubmitCubemapRender(
			cubemapIdx,
			slot,
			vertices[cubemapIdx],
			target,
			timeline,
			renderDone
		);

		// --------------------------------------------------------
		// 2) Compute (waits on renderDone)
		// --------------------------------------------------------
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

		// --------------------------------------------------------
		// 3) Copy (waits on computeDone)
		// --------------------------------------------------------
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

		// timelineValue now represents the last in-flight op for this slot
	}

	// ------------------------------------------------------------
	// Final synchronization (wait for ALL slots)
	// ------------------------------------------------------------
	for (uint32_t slot = 0; slot < SlotCount; ++slot)
	{
		VkSemaphore timeline = _timelines[slot];
		uint64_t value = _slotDoneValue[slot];

		if (value == 0)
			continue;

		VkSemaphoreWaitInfo waitInfo{};
		waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		waitInfo.semaphoreCount = 1;
		waitInfo.pSemaphores = &timeline;
		waitInfo.pValues = &value;

		auto waitStart = std::chrono::high_resolution_clock::now();

		vkWaitSemaphores(_device, &waitInfo, UINT64_MAX);

		double waitSeconds = SecondsSince(waitStart);

		LOG_DEBUG(
			"  [FinalWait] slot={}, value={}, wait={:.6f} ms",
			slot,
			value,
			waitSeconds * 1000.0);
		totalSlotWaitMs += waitSeconds * 1000.0;
	}
	LOG_DEBUG(
		"GPUAtomic pipelining stats: slotWait={:.3f} ms",
		totalSlotWaitMs
	);
	// ------------------------------------------------------------
	// Readback (now guaranteed complete)
	// ------------------------------------------------------------
	weights = computeStage->Readback();

	LOG_DEBUG("ComputeCoordinatesGPUSerial (pipelined): done");
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
			slot,
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
			_device,
			&waitInfo,
			UINT64_MAX
		);
	}
	weights = computeStage->Readback();

	LOG_DEBUG("ComputeCoordinatesGPUSerial: done");
}

void CubemapRenderInstance::ComputeCoordinatesCpu(
	const CubemapWorkRange& range, Eigen::MatrixXd& weights)
{
	auto* computeStage = static_cast<CpuComputeStrategy*>(_computeStage.get());
	const auto vertices = BuildDeformableVertexPositions();

	const uint32_t end = std::min<uint32_t>(
		range.first + range.count,
		static_cast<uint32_t>(vertices.size())
	);
	const uint32_t cubemapCount = end - range.first;

	VkSemaphore timeline = _timelines[0];
	uint64_t& timelineValue = _slotDoneValue[0];

	if (timelineValue > 0)
	{
		VkSemaphoreWaitInfo waitInfo{};
		waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		waitInfo.semaphoreCount = 1;
		waitInfo.pSemaphores = &timeline;
		waitInfo.pValues = &timelineValue;
		VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
	}

	uint64_t renderDone = timelineValue;
	for (uint32_t i = 0; i < cubemapCount; ++i)
	{
		const uint32_t cubemapIdx = range.first + i;
		CubemapRenderTarget& target = _cubemapRenderUnit.targets[i];

		renderDone = ++timelineValue;
		RecordAndSubmitCubemapRender(
			cubemapIdx,
			i,
			vertices[cubemapIdx],
			target,
			timeline,
			renderDone
		);

		computeStage->RecordReadback(i, target, cubemapIdx);
	}

	const uint64_t readbackDone = ++timelineValue;
	computeStage->SubmitAllReadbacks(
		timeline,
		renderDone,
		timeline,
		readbackDone
	);

	VkSemaphoreWaitInfo wait{};
	wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
	wait.semaphoreCount = 1;
	wait.pSemaphores = &timeline;
	wait.pValues = &readbackDone;
	VK_CHECK(vkWaitSemaphores(_device, &wait, UINT64_MAX));
	timelineValue = readbackDone;

	computeStage->ConsumeAllSlots();
	weights = computeStage->Readback();
}

void CubemapRenderInstance::ComputeCoordinates(
	const CubemapWorkRange& range, Eigen::MatrixXd& weights) {
	if (DeformationType::PMVCSerialOffset == _deformationType || DeformationType::PMVCSerialNoOffset == _deformationType) {
		ComputeCoordinatesGPUSerial(range, weights);
	}
	else if (DeformationType::PMVCRingOffset == _deformationType || DeformationType::PMVCRingNoOffset == _deformationType) {
		ComputeCoordinatesGPUAtomic(range, weights);
	}
	else if (DeformationType::PMVCAllOffset == _deformationType || DeformationType::PMVCAllNoOffset == _deformationType) {
		ComputeCoordinatesGPUMP(range, weights);
	}
	else if (DeformationType::PMVCCpuOffset == _deformationType || DeformationType::PMVCCpuNoOffset == _deformationType) {
		ComputeCoordinatesCpu(range, weights);
	}
}
