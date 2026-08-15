#include <Rendering/PMVC/RingComputeStrategy.h>

#include <algorithm>

uint32_t RingComputeStrategy::RequiredRenderTargetCount() const
{
	const auto vertexCount = static_cast<uint32_t>(_deformableMesh._vertices.rows());

	return std::max(1u, std::min(vertexCount, static_cast<uint32_t>(_targetCount)));
}

void RingComputeStrategy::Initialize()
{
	_targetCount = static_cast<int>(RequiredRenderTargetCount());

	_lambdaResults.resize(
		_deformableMesh._vertices.rows(),
		_cageMesh._vertices.rows()
	);
	_lambdaResults.setZero();
	_wsumResults.assign(_deformableMesh._vertices.rows(), 0.0);

	_computeCommandBuffers.resize(_targetCount);
	_copyCommandBuffers.resize(_targetCount);
	_slots.resize(_targetCount);

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

	VkCommandBufferAllocateInfo copyAllocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	copyAllocInfo.commandPool = _computeCommandPool;
	copyAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	copyAllocInfo.commandBufferCount = static_cast<uint32_t>(_copyCommandBuffers.size());

	if (vkAllocateCommandBuffers(_device, &copyAllocInfo, _copyCommandBuffers.data()) != VK_SUCCESS)
		throw std::runtime_error("Failed to allocate copy command buffers!");

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

void RingComputeStrategy::Cleanup()
{
	if (!_device)
		return;

	if (_computeCommandPool != VK_NULL_HANDLE)
	{
		if (!_computeCommandBuffers.empty())
		{
			vkFreeCommandBuffers(_device, _computeCommandPool, static_cast<uint32_t>(_computeCommandBuffers.size()), _computeCommandBuffers.data());
			_computeCommandBuffers.clear();
		}
		if (!_copyCommandBuffers.empty())
		{
			vkFreeCommandBuffers(_device, _computeCommandPool, static_cast<uint32_t>(_copyCommandBuffers.size()), _copyCommandBuffers.data());
			_copyCommandBuffers.clear();
		}

		vkDestroyCommandPool(_device, _computeCommandPool, nullptr);
		_computeCommandPool = VK_NULL_HANDLE;
	}

	// The compute pipeline is built per run together with the strategy, so it is released
	// with it instead of leaking one pipeline per weight computation.
	if (_renderPipelineManager != nullptr)
	{
		_renderPipelineManager->ReleasePipeline(_computePipeline);
		_computePipeline = PipelineHandle();
	}

	if (_barySampler != VK_NULL_HANDLE)
	{
		vkDestroySampler(_device, _barySampler, nullptr);
		_barySampler = VK_NULL_HANDLE;
	}

	if (_depthSampler != VK_NULL_HANDLE)
	{
		vkDestroySampler(_device, _depthSampler, nullptr);
		_depthSampler = VK_NULL_HANDLE;
	}

	if (_vertexListBuffer._deviceBuffer != VK_NULL_HANDLE)
	{
		_vertexListBuffer.ReleaseResource(_device);
		_vertexListBuffer = Buffer();
	}

	if (_interiorDistanceBuffer._deviceBuffer != VK_NULL_HANDLE)
	{
		_interiorDistanceBuffer.ReleaseResource(_device);
		_interiorDistanceBuffer = Buffer();
	}

	for (auto& slot : _slots)
	{
		if (slot.lambda._deviceBuffer != VK_NULL_HANDLE)
		{
			slot.lambda.ReleaseResource(_device);
			slot.lambda = Buffer();
		}
		if (slot.wsum._deviceBuffer != VK_NULL_HANDLE)
		{
			slot.wsum.ReleaseResource(_device);
			slot.wsum = Buffer();
		}
		if (slot.lambdaStaging._deviceBuffer != VK_NULL_HANDLE)
		{
			slot.lambdaStaging.ReleaseResource(_device);
			slot.lambdaStaging = MemoryMappedBuffer();
		}
		if (slot.wsumStaging._deviceBuffer != VK_NULL_HANDLE)
		{
			slot.wsumStaging.ReleaseResource(_device);
			slot.wsumStaging = MemoryMappedBuffer();
		}
	}
	_slots.clear();

	_sphereWeightCalculator.Cleanup(_device);
}

void RingComputeStrategy::BeginVertex(const uint32_t deformableIndex)
{
	if (deformableIndex >= static_cast<uint32_t>(_lambdaResults.rows()))
		return;

	// Every hit of the vertex adds its (weighted) contribution on top, so the row has to
	// start out empty.
	_lambdaResults.row(deformableIndex).setZero();
	_wsumResults[deformableIndex] = 0.0;
}

void RingComputeStrategy::DispatchAfterRender(
	const uint32_t deformableIndex,
	const uint32_t slot,
	VkSemaphore timeline,
	const uint64_t waitValue,
	const uint64_t signalValue,
	const RenderedHit& first,
	const std::optional<RenderedHit>& second)
{
	// Without a second hit the shader only needs a valid binding for set B, so it is
	// pointed at set A and disabled with a weight of zero.
	RenderedHit secondHit = second.value_or(RenderedHit{ first.colorView, first.depthView, 0.0f });

	// ------------------------------------------------------------
	// 1) Reset & record command buffer
	// ------------------------------------------------------------
	VkCommandBuffer cmd = _computeCommandBuffers[slot];
	VK_CHECK(vkResetCommandBuffer(cmd, 0));

	VkCommandBufferBeginInfo begin{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO
	};
	VK_CHECK(vkBeginCommandBuffer(cmd, &begin));

	// Descriptor update MUST happen before bind
	UpdateComputeDescriptorSet(slot, first, secondHit);

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
	pc.uFaceSize = { static_cast<int>(_faceSize), static_cast<int>(_faceSize) };
	pc.uNumTriangles = static_cast<int32_t>(_cageMesh._faces.rows());
	pc.uNumCageVertices = static_cast<int32_t>(_cageMesh._vertices.rows());
	pc.uMeshVertexIdx = static_cast<int32_t>(deformableIndex);
	pc.uNearPlane = _interiorDistance.nearPlane;
	pc.uFarPlane = _interiorDistance.farPlane;
	pc.uHitWeightA = first.weight;
	pc.uHitWeightB = secondHit.weight;
	pc.uFlags = (_offset ? ComputePushConstants::SolidAngleOnly : 0) |
		(UseInteriorDistance() ? ComputePushConstants::InteriorDistance : 0);

	vkCmdPushConstants(
		cmd,
		pipe._pipelineLayout,
		VK_SHADER_STAGE_COMPUTE_BIT,
		0, sizeof(pc), &pc
	);

	// ------------------------------------------------------------
	// 2) Clear buffers
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
	// 3) Dispatch
	// ------------------------------------------------------------
	const uint32_t groupsX = (_faceSize + kDispatchGroupSize - 1) / kDispatchGroupSize;
	const uint32_t groupsY = (_faceSize + kDispatchGroupSize - 1) / kDispatchGroupSize;

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
	// 4) Submit with monotonic timeline signal
	// ------------------------------------------------------------
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
	vkGetDeviceQueue(_device, _computeQueueFamily, 0, &queue);
	VK_CHECK(vkQueueSubmit2(queue, 1, &submit2, VK_NULL_HANDLE));
}

void RingComputeStrategy::SubmitReadbackCopy(
	const uint32_t slot,
	VkSemaphore timeline,
	const uint64_t waitValue,
	const uint64_t signalValue)
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
		.size = static_cast<VkDeviceSize>(_cageMesh._vertices.rows()) * sizeof(float)
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

	VkCommandBufferSubmitInfo cmdInfo{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
		.commandBuffer = cmd
	};

	VkSemaphoreSubmitInfo waitInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = timeline,
		.value = waitValue,
		.stageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT
	};

	VkSemaphoreSubmitInfo signalInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = timeline,
		.value = signalValue,
		.stageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT
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
	vkGetDeviceQueue(_device, _computeQueueFamily, 0, &queue);
	VK_CHECK(vkQueueSubmit2(queue, 1, &submit2, VK_NULL_HANDLE));
}

void RingComputeStrategy::AccumulateSlot(
	const uint32_t deformableIndex,
	const uint32_t slot,
	VkSemaphore timeline,
	const uint64_t waitValue)
{
	assert(slot < _slots.size());
	assert(deformableIndex < static_cast<uint32_t>(_lambdaResults.rows()));

	uint64_t signal = waitValue;
	VkSemaphoreWaitInfo wait{};
	wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
	wait.semaphoreCount = 1;
	wait.pSemaphores = &timeline;
	wait.pValues = &signal;
	VK_CHECK(vkWaitSemaphores(_device, &wait, UINT64_MAX));

	const auto cageVertexCount = static_cast<size_t>(_cageMesh._vertices.rows());

	const float* lambda =
		static_cast<const float*>(_slots[slot].lambdaStaging._mappedData);
	const float wsum =
		*static_cast<const float*>(_slots[slot].wsumStaging._mappedData);

	// The dispatch already scaled the hit by its weight, so the hits of a mesh vertex
	// simply add up here.
	for (size_t c = 0; c < cageVertexCount; ++c)
	{
		_lambdaResults(deformableIndex, c) += lambda[c];
	}

	_wsumResults[deformableIndex] += wsum;
}

Eigen::MatrixXd RingComputeStrategy::Readback()
{
	for (Eigen::Index i = 0; i < _lambdaResults.rows(); ++i)
	{
		const double wsum = _wsumResults[i];
		if (wsum != 0.0)
		{
			_lambdaResults.row(i) /= wsum;
		}
	}

	return _lambdaResults;
}

void RingComputeStrategy::CreatePipelineAndLayouts()
{
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = _computeQueueFamily;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_computeCommandPool) != VK_SUCCESS)
		throw std::runtime_error("Failed to create compute command pool!");

	std::vector<VkDescriptorSetLayoutBinding> bindings{
		// Barycentric + triangle index image
		{ 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT },
		// Vertex index list
		{ 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT },
		// lambda output
		{ 2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT },
		// wsum output
		{ 3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT },
		// Solid angle lookup
		{ 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT },
		// Depth cubemap of the first hit
		{ 5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT },
		// Interior detour table
		{ 6, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT },
		// Barycentric + triangle index image of the second hit
		{ 7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT },
		// Depth cubemap of the second hit
		{ 8, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT }
	};

	_computeLayout = _descriptorPool->CreateDescriptorSetLayout(bindings);

	ComputePipelineObjectProxy proxy;
	proxy._renderPipelineManager = _renderPipelineManager;
	proxy._shaderModule = "assets/shaders/PMVCCompute.comp.spv";
	proxy._descriptorSetLayouts = { _computeLayout };

	VkPushConstantRange range{};
	range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	range.offset = 0;
	range.size = sizeof(ComputePushConstants);
	proxy._pushConstantRanges = { range };

	_computePipeline = proxy.Build();
}

void RingComputeStrategy::AllocateResources()
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
	allocInfo.descriptorSetCount = static_cast<uint32_t>(_targetCount);
	allocInfo.pSetLayouts = layouts.data();

	VK_CHECK(vkAllocateDescriptorSets(
		_device,
		&allocInfo,
		_computeDescriptorSets.data()
	));

	// --------------------------------------------------
	// Allocate per-slot lambda / wsum buffers
	// --------------------------------------------------
	const VkDeviceSize lambdaBytes = static_cast<VkDeviceSize>(_cageMesh._vertices.rows()) * sizeof(float);
	const VkDeviceSize wsumBytes = sizeof(float);

	for (int i = 0; i < _targetCount; ++i)
	{
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
			std::span<std::byte>(static_cast<std::byte*>(nullptr), lambdaBytes),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		);

		_slots[i].wsumStaging = _resourceManager->CreateBufferAndMapMemory(
			std::span<std::byte>(static_cast<std::byte*>(nullptr), wsumBytes),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		);
	}

	// --------------------------------------------------
	// Vertex index list buffer
	// --------------------------------------------------
	const auto triCount = static_cast<int>(_cageMesh._faces.rows());
	std::vector<uint32_t> vertexList(static_cast<size_t>(triCount) * 3);

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

	// --------------------------------------------------
	// Interior detour table (column-major Eigen storage: one column of cage-vertex
	// detours per mesh vertex, so the raw data already matches the shader indexing
	// interiorDetour[meshVertexIdx * numCageVertices + cageVertexIdx]). The shader always
	// declares the binding, so a dummy element is uploaded when the variant is disabled.
	// --------------------------------------------------
	const std::vector<float> dummyTable(1, 0.0f);
	const std::span<const float> tableData = UseInteriorDistance()
		? std::span<const float>(_interiorDistance.detours->data(), static_cast<size_t>(_interiorDistance.detours->size()))
		: std::span<const float>(dummyTable.data(), dummyTable.size());

	auto tableStaging = _resourceManager->CreateBufferAndCopy(
		tableData,
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	const VkDeviceSize tableBytes = tableData.size_bytes();

	_interiorDistanceBuffer = _resourceManager->AllocateDeviceBuffer(
		tableBytes,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(
		tableStaging._deviceBuffer,
		_interiorDistanceBuffer._deviceBuffer,
		tableBytes
	);

	tableStaging.ReleaseResource(_device);
	vertexListStaging.ReleaseResource(_device);
}

void RingComputeStrategy::UpdateComputeDescriptorSet(const uint32_t slotIndex, const RenderedHit& first, const RenderedHit& second)
{
	VkDescriptorImageInfo imageInfo{};
	imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	imageInfo.imageView = first.colorView;
	imageInfo.sampler = _barySampler;

	VkDescriptorBufferInfo vertexListInfo{
		_vertexListBuffer._deviceBuffer, 0, VK_WHOLE_SIZE
	};

	VkDescriptorImageInfo depthImageInfo{};
	depthImageInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
	depthImageInfo.imageView = first.depthView;
	depthImageInfo.sampler = _depthSampler;

	VkDescriptorImageInfo secondImageInfo{};
	secondImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	secondImageInfo.imageView = second.colorView;
	secondImageInfo.sampler = _barySampler;

	VkDescriptorImageInfo secondDepthImageInfo{};
	secondDepthImageInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
	secondDepthImageInfo.imageView = second.depthView;
	secondDepthImageInfo.sampler = _depthSampler;

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

	VkDescriptorBufferInfo interiorDistanceInfo{
		_interiorDistanceBuffer._deviceBuffer, 0, VK_WHOLE_SIZE
	};

	std::array<VkWriteDescriptorSet, 9> writes{};

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

	writes[6] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
		_computeDescriptorSets[slotIndex], 6, 0, 1,
		VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &interiorDistanceInfo };

	writes[7] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
		_computeDescriptorSets[slotIndex], 7, 0, 1,
		VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &secondImageInfo };

	writes[8] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
		_computeDescriptorSets[slotIndex], 8, 0, 1,
		VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &secondDepthImageInfo };

	vkUpdateDescriptorSets(_device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
}

void RingComputeStrategy::CreateSampler()
{
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

	// Clamp (doesnt really matter since texelFetch ignores addressing)
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

	VK_CHECK(vkCreateSampler(_device, &samplerInfo, nullptr, &_barySampler));
}

void RingComputeStrategy::CreateDepthSampler()
{
	VkSamplerCreateInfo samplerInfo{};
	samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;

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

	samplerInfo.unnormalizedCoordinates = VK_FALSE;
	samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;

	VK_CHECK(vkCreateSampler(_device, &samplerInfo, nullptr, &_depthSampler));
}

void RingComputeStrategy::CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size)
{
	// Allocate a temporary one-time command buffer
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandPool = _computeCommandPool;
	allocInfo.commandBufferCount = 1;

	VkCommandBuffer cmd;
	VK_CHECK(vkAllocateCommandBuffers(_device, &allocInfo, &cmd));

	// Begin recording
	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

	// Copy command
	VkBufferCopy copyRegion{};
	copyRegion.srcOffset = 0;
	copyRegion.dstOffset = 0;
	copyRegion.size = size;
	vkCmdCopyBuffer(cmd, src, dst, 1, &copyRegion);
	VK_CHECK(vkEndCommandBuffer(cmd));

	// Submit and wait
	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &cmd;

	VkQueue queue;
	vkGetDeviceQueue(_device, _computeQueueFamily, 0, &queue);

	VK_CHECK(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));
	VK_CHECK(vkQueueWaitIdle(queue));

	// Clean up
	vkFreeCommandBuffers(_device, _computeCommandPool, 1, &cmd);
}
