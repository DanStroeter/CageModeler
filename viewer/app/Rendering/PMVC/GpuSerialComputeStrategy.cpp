#include <Rendering/PMVC/GpuSerialComputeStrategy.h>
#include <Rendering/PMVC/ScopedCmdBuffer.h>
#include <Rendering/PMVC/CubemapRenderInstance.h>

uint32_t GpuSerialComputeStrategy::RequiredRenderTargetCount() const
{
	return _targetCount;
}

void GpuSerialComputeStrategy::Initialize()
{
	//_slotSync.resize(_targetCount);
	// Prepare slots
	//_slotDoneValue.resize(_targetCount, 0ull);
	_lambdaResults.resize(_deformableMesh._vertices.rows());
	for (int i = 0; i < _lambdaResults.size(); i++) {
		_lambdaResults[i].resize(_cageMesh._vertices.rows(), 0.0f);
	}
	_wsumResults.resize(_deformableMesh._vertices.rows(), 0.0f);

	_computeCommandBuffers.resize(1);
	_copyCommandBuffers.resize(1);
	_slots.resize(1);
	// Pipeline + layouts
	CreatePipelineAndLayouts();

	// Allocate all buffers / descriptor sets
	AllocateResources();

	// Create per-target command buffers
	VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	allocInfo.commandPool = _computeCommandPool;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandBufferCount = static_cast<uint32_t>(_computeCommandBuffers.size());

	if (vkAllocateCommandBuffers(_device, &allocInfo, _computeCommandBuffers.data()) != VK_SUCCESS)
		throw std::runtime_error("Failed to allocate compute command buffers!");

	VkCommandBufferAllocateInfo allocInfo2{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	allocInfo.commandPool = _computeCommandPool;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandBufferCount = static_cast<uint32_t>(_copyCommandBuffers.size());

	if (vkAllocateCommandBuffers(_device, &allocInfo, _copyCommandBuffers.data()) != VK_SUCCESS)
		throw std::runtime_error("Failed to allocate compute command buffers!");

	_sphereWeightCalculator = SphereWeightCalculator();
	_sphereWeightCalculator.SphereWeightInitialization(
		_faceSize,
		_device,
		_resourceManager,
		_computeCommandPool
	);
	CreateSampler();
	CreateDepthSampler();
}

//TODO maybe needs to be checked
/*void GpuSerialComputeStrategy::WaitForTargetReuse(VkSemaphore timeline, uint64_t slotDoneValue)
{
	if (slotDoneValue == 0) return;

	uint64_t currentValue = 0;
	vkGetSemaphoreCounterValue(_device, timeline, &currentValue);

	if (slotDoneValue > currentValue)
	{
		VkSemaphoreWaitInfo waitInfo{};
		waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		waitInfo.semaphoreCount = 1;
		waitInfo.pSemaphores = &timeline;
		waitInfo.pValues = &slotDoneValue;

		vkWaitSemaphores(_device, &waitInfo, UINT64_MAX);
	}
}*/

void GpuSerialComputeStrategy::DispatchAfterRender(
	uint32_t deformableIndex,
	uint32_t slot,
	VkSemaphore timeline,
	uint64_t waitValue,
	uint64_t signalValue,
	const CubemapRenderTarget& target)
{
	// ------------------------------------------------------------
	// 2) Reset & record command buffer
	// ------------------------------------------------------------
	VkCommandBuffer cmd = _computeCommandBuffers[slot];
	VK_CHECK(vkResetCommandBuffer(cmd, 0));

	VkCommandBufferBeginInfo begin{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO
	};
	VK_CHECK(vkBeginCommandBuffer(cmd, &begin));

	// Descriptor update MUST happen before bind
	UpdateComputeDescriptorSet(slot, target);

	assert(slot < _computeDescriptorSets.size());

	auto& pipe = _renderPipelineManager->GetPipelineObject(_computePipeline);

	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipe._handle);

	vkCmdBindDescriptorSets(
		cmd,
		VK_PIPELINE_BIND_POINT_COMPUTE,
		pipe._pipelineLayout,
		0, 1,
		&_computeDescriptorSets[slot],
		0, nullptr
	);

	// Push constants
	ComputePushConstants pc{};
	pc.uFaceSize = { (int)_faceSize, (int)_faceSize };
	pc.uNumTriangles = _cageMesh._faces.rows();

	vkCmdPushConstants(
		cmd,
		pipe._pipelineLayout,
		VK_SHADER_STAGE_COMPUTE_BIT,
		0, sizeof(pc), &pc
	);

	// ------------------------------------------------------------
	// 3) Clear buffers
	// ------------------------------------------------------------
	vkCmdFillBuffer(
		cmd,
		_slots[slot].lambda._deviceBuffer,
		0, VK_WHOLE_SIZE, 0
	);

	vkCmdFillBuffer(
		cmd,
		_slots[slot].wsum._deviceBuffer,
		0, VK_WHOLE_SIZE, 0
	);

	VkBufferMemoryBarrier clearBarrier[2]{};
	for (int i = 0; i < 2; ++i)
	{
		clearBarrier[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
		clearBarrier[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		clearBarrier[i].dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
		clearBarrier[i].offset = 0;
		clearBarrier[i].size = VK_WHOLE_SIZE;
	}

	clearBarrier[0].buffer = _slots[slot].lambda._deviceBuffer;
	clearBarrier[1].buffer = _slots[slot].wsum._deviceBuffer;

	vkCmdPipelineBarrier(
		cmd,
		VK_PIPELINE_STAGE_TRANSFER_BIT,
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
		0,
		0, nullptr,
		2, clearBarrier,
		0, nullptr
	);

	// ------------------------------------------------------------
	// 4) Dispatch
	// ------------------------------------------------------------
	uint32_t groupsX = (_faceSize + kDispatchGroupSize - 1) / kDispatchGroupSize;
	uint32_t groupsY = (_faceSize + kDispatchGroupSize - 1) / kDispatchGroupSize;

	vkCmdDispatch(cmd, groupsX, groupsY, 6);

	// Barrier for transfer reads (copy stage)
	VkBufferMemoryBarrier postBarrier[2]{};
	for (int i = 0; i < 2; ++i)
	{
		postBarrier[i] = clearBarrier[i];
		postBarrier[i].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
		postBarrier[i].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	}

	vkCmdPipelineBarrier(
		cmd,
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
		VK_PIPELINE_STAGE_TRANSFER_BIT,
		0,
		0, nullptr,
		2, postBarrier,
		0, nullptr
	);

	VK_CHECK(vkEndCommandBuffer(cmd));

	// ------------------------------------------------------------
	// 5) Submit with monotonic timeline signal
	// ------------------------------------------------------------

	VkTimelineSemaphoreSubmitInfo timelineInfo{
		.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO,
		.waitSemaphoreValueCount = 1,
		.pWaitSemaphoreValues = &waitValue,
		.signalSemaphoreValueCount = 1,
		.pSignalSemaphoreValues = &signalValue
	};

	VkPipelineStageFlags waitStage =
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;

	/*VkSubmitInfo submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit.pNext = &timelineInfo;

	submit.waitSemaphoreCount = 1;
	submit.pWaitSemaphores = &timeline;
	submit.pWaitDstStageMask = &waitStage;

	submit.commandBufferCount = 1;
	submit.pCommandBuffers = &cmd;

	submit.signalSemaphoreCount = 1;
	submit.pSignalSemaphores = &timeline;

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
	VK_CHECK(vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE));*/
	VkCommandBufferSubmitInfo cmdInfo{
	.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
	.commandBuffer = cmd
	};

	VkSemaphoreSubmitInfo waitInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = timeline,
		.value = waitValue,
		.stageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT
	};

	VkSemaphoreSubmitInfo signalInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = timeline,
		.value = signalValue,
		.stageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT
	};

	VkSubmitInfo2 submit2{
		.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2,
		.waitSemaphoreInfoCount = 1,
		.pWaitSemaphoreInfos = &waitInfo,
		.commandBufferInfoCount = 1,
		.pCommandBufferInfos = &cmdInfo,
		.signalSemaphoreInfoCount = 1,
		.pSignalSemaphoreInfos = &signalInfo
	};

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
	vkQueueSubmit2(queue, 1, &submit2, VK_NULL_HANDLE);
}

void GpuSerialComputeStrategy::SubmitReadbackCopy(
	uint32_t slot,
	VkSemaphore timeline,
	uint64_t waitValue,
	uint64_t signalValue)
{
	VkCommandBuffer cmd = _copyCommandBuffers[slot];

	VK_CHECK(vkResetCommandBuffer(cmd, 0));

	VkCommandBufferBeginInfo beginInfo{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
		.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
	};
	VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

	VkBufferCopy lambdaCopy{
		.srcOffset = 0,
		.dstOffset = 0,
		.size = _cageMesh._vertices.rows() * sizeof(float)
	};

	VkBufferCopy wsumCopy{
		.srcOffset = 0,
		.dstOffset = 0,
		.size = sizeof(float)
	};

	vkCmdCopyBuffer(
		cmd,
		_slots[slot].lambda._deviceBuffer,
		_slots[slot].lambdaStaging._deviceBuffer,
		1, &lambdaCopy
	);

	vkCmdCopyBuffer(
		cmd,
		_slots[slot].wsum._deviceBuffer,
		_slots[slot].wsumStaging._deviceBuffer,
		1, &wsumCopy
	);

	VK_CHECK(vkEndCommandBuffer(cmd));

	VkPipelineStageFlags waitStage =
		VK_PIPELINE_STAGE_TRANSFER_BIT;

	VkTimelineSemaphoreSubmitInfo timelineInfo{
		.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO,
		.waitSemaphoreValueCount = 1,
		.pWaitSemaphoreValues = &waitValue,
		.signalSemaphoreValueCount = 1,
		.pSignalSemaphoreValues = &signalValue
	};

	VkSubmitInfo submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit.pNext = &timelineInfo;

	submit.waitSemaphoreCount = 1;
	submit.pWaitSemaphores = &timeline;
	submit.pWaitDstStageMask = &waitStage;

	submit.commandBufferCount = 1;
	submit.pCommandBuffers = &cmd;

	submit.signalSemaphoreCount = 1;
	submit.pSignalSemaphores = &timeline;

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
	VK_CHECK(vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE));
}

void GpuSerialComputeStrategy::ConsumeSlot(
	uint32_t deformableIndex,
	uint32_t slot,
	VkSemaphore timeline, 
	uint32_t waitValue)
{
	//assert(slot < _slots.size());
	//assert(deformableIndex < _lambdaResults.size());

	const size_t c = _cageMesh._vertices.rows();

	//assert(_lambdaResults[deformableIndex].size() == c);
	//assert(_slots[slot].lambdaStaging.sizeBytes >= C * sizeof(float));
	//assert(_slots[slot].wsumStaging.sizeBytes >= sizeof(float));
	uint64_t signal = waitValue;
	VkSemaphoreWaitInfo wait{};
	wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
	wait.semaphoreCount = 1;
	wait.pSemaphores = &timeline;
	wait.pValues = &signal;
	vkWaitSemaphores(_device, &wait, UINT64_MAX);

	const size_t C = _cageMesh._vertices.rows();

	const float* lambda =
		(float*)_slots[slot].lambdaStaging._mappedData;
	const float wsum =
		*(float*)_slots[slot].wsumStaging._mappedData;

	for (size_t c = 0; c < C; ++c)
		_lambdaResults[deformableIndex][c] = lambda[c];

	_wsumResults[deformableIndex] = wsum;
}
Eigen::MatrixXd toEigenMatrix(const std::vector<std::vector<float>>& v)
{
	const std::size_t rows = v.size();
	const std::size_t cols = rows ? v[0].size() : 0;

	Eigen::MatrixXd m(rows, cols);

	for (std::size_t i = 0; i < rows; ++i)
	{
		assert(v[i].size() == cols); // ensure rectangular
		for (std::size_t j = 0; j < cols; ++j)
		{
			m(static_cast<Eigen::Index>(i),
				static_cast<Eigen::Index>(j)) = static_cast<double>(v[i][j]);
		}
	}
	return m;
}

Eigen::MatrixXd GpuSerialComputeStrategy::Readback()
{
	for (int i = 0; i < _lambdaResults.size(); i++) {
		for (int j = 0; j < _lambdaResults[i].size(); j++) {
			_lambdaResults[i][j] /= _wsumResults[i];
		}
	}
	//WriteWeightsToFile("GpuSerialComputeStrategy_Readback.txt");
	return toEigenMatrix(_lambdaResults);
}

void GpuSerialComputeStrategy::CreatePipelineAndLayouts() {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = _transferQueueFamily;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_computeCommandPool) != VK_SUCCESS)
		throw std::runtime_error("Failed to create compute command pool!");

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
		{ 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT },
		// 
		 { 5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT }
	};

	_computeLayout = _descriptorPool->CreateDescriptorSetLayout(bindings);

	ComputePipelineObjectProxy proxy;
	proxy._renderPipelineManager = _renderPipelineManager;
	proxy._shaderModule = "assets/shaders/PMVCComputeAtmoicDepth.comp.spv";
	proxy._descriptorSetLayouts = { _computeLayout };

	VkPushConstantRange range{};
	range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	range.offset = 0;
	range.size = sizeof(ComputePushConstants);
	proxy._pushConstantRanges = { range };

	_computePipeline = proxy.Build();
}

void GpuSerialComputeStrategy::CreateDepthSampler() {
	VkSamplerCreateInfo samplerInfo{};
	samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;

	// For depth, you might want linear filtering
	samplerInfo.magFilter = VK_FILTER_NEAREST;
	samplerInfo.minFilter = VK_FILTER_NEAREST;

	samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
	samplerInfo.minLod = 0.0f;
	samplerInfo.maxLod = 0.0f;
	samplerInfo.mipLodBias = 0.0f;

	samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;

	samplerInfo.anisotropyEnable = VK_FALSE;
	
	// Enable comparison for shadow mapping if needed
	//samplerInfo.compareEnable = VK_TRUE;  // Set to true if doing shadow comparison
	//samplerInfo.compareOp = VK_COMPARE_OP_LESS;  // Or appropriate comparison

	samplerInfo.unnormalizedCoordinates = VK_FALSE;
	samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;

	vkCreateSampler(_device, &samplerInfo, nullptr, &_depthSampler);
}

void GpuSerialComputeStrategy::CreateSampler() {
	VkSamplerCreateInfo samplerInfo{};
	samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;

	// NO filtering
	samplerInfo.magFilter = VK_FILTER_NEAREST;
	samplerInfo.minFilter = VK_FILTER_NEAREST;

	// NO mipmapping
	samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
	samplerInfo.minLod = 0.0f;
	samplerInfo.maxLod = 0.0f;
	samplerInfo.mipLodBias = 0.0f;

	// Clamp (doesn’t really matter since texelFetch ignores addressing)
	samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;

	// No anisotropy
	samplerInfo.anisotropyEnable = VK_FALSE;

	// No comparison
	samplerInfo.compareEnable = VK_FALSE;

	// Normalized coordinates irrelevant for texelFetch
	samplerInfo.unnormalizedCoordinates = VK_FALSE;

	// Border color unused
	samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
	vkCreateSampler(_device, &samplerInfo, nullptr, &_barySampler);
}


void GpuSerialComputeStrategy::AllocateResources()
{
	_computeDescriptorSets.resize(_targetCount);

	std::vector<VkDescriptorSetLayout> layouts(
		_targetCount,
		_computeLayout->GetReference()
	);

	VkDescriptorSetAllocateInfo allocInfo{
		VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO
	};
	allocInfo.descriptorPool = _descriptorPool;
	allocInfo.descriptorSetCount = _targetCount;
	allocInfo.pSetLayouts = layouts.data();

	VK_CHECK(vkAllocateDescriptorSets(
		_device,
		&allocInfo,
		_computeDescriptorSets.data()
	));

	//const size_t C = static_cast<size_t>(_cageMesh._vertices.rows());

	// --------------------------------------------------
	// Allocate per-slot lambda / wsum buffers
	// --------------------------------------------------
	for (uint32_t i = 0; i < _targetCount; ++i)
	{
		const VkDeviceSize lambdaBytes = static_cast<size_t>(_cageMesh._vertices.rows()) * sizeof(float);
		const VkDeviceSize wsumBytes = sizeof(float);

		// Device-local buffers
		_slots[i].lambda = _resourceManager->AllocateDeviceBuffer(
			lambdaBytes,
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
			VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
			VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
		);

		_slots[i].wsum = _resourceManager->AllocateDeviceBuffer(
			wsumBytes,
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
			VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
			VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
		);

		// Host-visible staging buffers
		_slots[i].lambdaStaging = _resourceManager->CreateBufferAndMapMemory(
			std::span<std::byte>((std::byte*)nullptr, lambdaBytes),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		);

		_slots[i].wsumStaging = _resourceManager->CreateBufferAndMapMemory(
			std::span<std::byte>((std::byte*)nullptr, wsumBytes),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		);
	}

	// --------------------------------------------------
	// Vertex index list buffer
	// --------------------------------------------------
	const int triCount = _cageMesh._faces.rows();
	std::vector<uint32_t> vertexList(triCount * 3);

	for (int t = 0; t < triCount; ++t)
	{
		vertexList[t * 3 + 0] = static_cast<uint32_t>(_cageMesh._faces(t, 0));
		vertexList[t * 3 + 1] = static_cast<uint32_t>(_cageMesh._faces(t, 1));
		vertexList[t * 3 + 2] = static_cast<uint32_t>(_cageMesh._faces(t, 2));
	}

	auto vertexListStaging = _resourceManager->CreateBufferAndCopy(
		std::span<const uint32_t>(vertexList.data(), vertexList.size()),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_vertexListBuffer = _resourceManager->AllocateDeviceBuffer(
		vertexList.size() * sizeof(uint32_t),
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(
		vertexListStaging._deviceBuffer,
		_vertexListBuffer._deviceBuffer,
		vertexList.size() * sizeof(uint32_t)
	);
}

void GpuSerialComputeStrategy::UpdateComputeDescriptorSet(uint32_t slotIndex, const CubemapRenderTarget& target)
{
	VkDescriptorImageInfo imageInfo{};
	imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	imageInfo.imageView = target.cubemapView;
	imageInfo.sampler = _barySampler;

	VkDescriptorImageInfo depthImageInfo{};
	depthImageInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
	depthImageInfo.imageView = target.depthView;  // You need to add this to CubemapRenderTarget
	depthImageInfo.sampler = _depthSampler;

	
	VkDescriptorBufferInfo vertexListInfo{
		_vertexListBuffer._deviceBuffer, 0, VK_WHOLE_SIZE
	};

	VkDescriptorBufferInfo lambdaInfo{
		_slots[slotIndex].lambda._deviceBuffer, 0, VK_WHOLE_SIZE
	};

	VkDescriptorBufferInfo wsumInfo{
		_slots[slotIndex].wsum._deviceBuffer, 0, VK_WHOLE_SIZE
	};

	VkDescriptorImageInfo weightInfo{};
	weightInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	weightInfo.imageView = _sphereWeightCalculator._solidAngleArrayView;
	weightInfo.sampler = _sphereWeightCalculator._solidAngleSampler;

	std::array<VkWriteDescriptorSet, 6> writes{};

	writes[0] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
		_computeDescriptorSets[slotIndex], 0, 0, 1,
		VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &imageInfo };

	writes[1] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
		_computeDescriptorSets[slotIndex], 1, 0, 1,
		VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &vertexListInfo };

	writes[2] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
		_computeDescriptorSets[slotIndex], 2, 0, 1,
		VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &lambdaInfo };

	writes[3] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
		_computeDescriptorSets[slotIndex], 3, 0, 1,
		VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &wsumInfo };

	writes[4] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
		_computeDescriptorSets[slotIndex], 4, 0, 1,
		VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &weightInfo };
	writes[5] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
		_computeDescriptorSets[slotIndex], 5, 0, 1,
		VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &depthImageInfo };


	vkUpdateDescriptorSets(_device, writes.size(), writes.data(), 0, nullptr);
}

void GpuSerialComputeStrategy::CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size)
{
	// Allocate a temporary one-time command buffer
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandPool = _computeCommandPool;
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
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &graphicsQueue);

	vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
	vkQueueWaitIdle(graphicsQueue);

	// Clean up
	vkFreeCommandBuffers(_device, _computeCommandPool, 1, &cmd);
}

void GpuSerialComputeStrategy::WriteWeightsToFile(const std::string& filename)
{
	std::ofstream file(filename);
	if (!file.is_open()) throw std::runtime_error("Failed to open weight write file!");

	file << "===== LAMBDA BUFFER (cubemaps=" << _lambdaResults.size()
		<< ", verts=" << (_lambdaResults.empty() ? 0 : _lambdaResults[0].size()) << ") =====\n";
	for (size_t cub = 0; cub < _lambdaResults.size(); ++cub) {
		float sum = 0.0f;
		for (size_t v = 0; v < _lambdaResults[cub].size(); ++v) {
			file << "lambda[" << cub << "][" << v << "] = " << _lambdaResults[cub][v] << "\n";
			sum += _lambdaResults[cub][v];
		}
		file << "Lambda sum: " << sum << "\n";
		file << "Sum normalize: " << _wsumResults[cub] << "\n";
	}
	file.close();
	LOG_INFO("Weights written to " + filename);
}