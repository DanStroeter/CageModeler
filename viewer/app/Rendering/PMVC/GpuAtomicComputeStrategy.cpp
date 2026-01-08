#include <Rendering/PMVC/GpuAtomicComputeStrategy.h>
#include <Rendering/PMVC/ScopedCmdBuffer.h>
#include <Rendering/PMVC/CubemapRenderInstance.h>

uint32_t GpuAtomicComputeStrategy::RequiredRenderTargetCount() const
{
    return 2;
}

void GpuAtomicComputeStrategy::Initialize(uint32_t targetCount)
{
	_slotSync.resize(targetCount);
	// Prepare slots
	_slotDoneValue.resize(targetCount, 0ull);
	_computeCommandBuffers.resize(targetCount);

	// Create command pool
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = _transferQueueFamily;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_computeCommandPool) != VK_SUCCESS)
		throw std::runtime_error("Failed to create compute command pool!");

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

	_sphereWeightCalculator.SphereWeightInitialization(
		_faceSize,
		_device,
		_resourceManager,
		_computeCommandPool
	);
	CreateSampler();
}

void GpuAtomicComputeStrategy::WaitForTargetReuse(
	uint32_t targetIndex, VkSemaphore timeline, uint64_t slotDoneValue)
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
}

void GpuAtomicComputeStrategy::DispatchAfterRender(
	uint32_t deformableIndex,
	uint32_t slot,
	VkSemaphore timeline,
	const CubemapRenderTarget& target)
{
	// Ensure slot reuse
	WaitForTargetReuse(slot, timeline, _slotSync[slot].copyDone);

	VkCommandBuffer cmd = _computeCommandBuffers[slot];
	vkResetCommandBuffer(cmd, 0);

	VkCommandBufferBeginInfo begin{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	vkBeginCommandBuffer(cmd, &begin);

	// --- image barrier ---
	InsertImageMemoryBarrierToGeneral(cmd, target.cubemapImage, {
		VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6
		});

	UpdateComputeDescriptorSet(slot);

	auto& pipe = _renderPipelineManager->GetPipelineObject(_computePipeline);
	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipe._handle);
	vkCmdBindDescriptorSets(
		cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
		pipe._pipelineLayout, 0, 1,
		&_computeDescriptorSets[slot], 0, nullptr
	);

	ComputePushConstants pc{};
	pc.uNumCubemaps = deformableIndex;
	pc.uNumCageVertices = _cageMesh._vertices.rows();
	pc.uFaceSize = { (int)_faceSize, (int)_faceSize };
	pc.uFacesPerCubemap = 6;
	pc.uNumTriangles = _cageMesh._faces.rows();

	vkCmdPushConstants(
		cmd, pipe._pipelineLayout,
		VK_SHADER_STAGE_COMPUTE_BIT,
		0, sizeof(pc), &pc
	);

	// Clear slot buffers
	vkCmdFillBuffer(cmd,
		_slots[slot].lambda._deviceBuffer, 0, VK_WHOLE_SIZE, 0);
	vkCmdFillBuffer(cmd,
		_slots[slot].wsum._deviceBuffer, 0, VK_WHOLE_SIZE, 0);

	// Barrier: transfer ? compute
	VkBufferMemoryBarrier clearBarrier[2]{};
	for (int i = 0; i < 2; ++i) {
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
		0, 0, nullptr, 2, clearBarrier, 0, nullptr
	);

	uint32_t groupsX = (_faceSize + kDispatchGroupSize - 1) / kDispatchGroupSize; 
	uint32_t groupsY = (_faceSize + kDispatchGroupSize - 1) / kDispatchGroupSize;

	vkCmdDispatch(cmd, groupsX, groupsY, 6);

	// Barrier: compute transfer (for copy)
	VkBufferMemoryBarrier postBarrier[2]{};
	for (int i = 0; i < 2; ++i) {
		postBarrier[i] = clearBarrier[i];
		postBarrier[i].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
		postBarrier[i].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	}

	vkCmdPipelineBarrier(
		cmd,
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
		VK_PIPELINE_STAGE_TRANSFER_BIT,
		0, 0, nullptr, 2, postBarrier, 0, nullptr
	);

	vkEndCommandBuffer(cmd);

	// --- Timeline submission ---
	uint64_t signal = NextTimelineValue();
	_slotSync[slot].computeDone = signal;

	VkTimelineSemaphoreSubmitInfo timelineInfo{
		VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO
	};
	timelineInfo.waitSemaphoreValueCount = 1;
	timelineInfo.pWaitSemaphoreValues = &_slotSync[slot].renderDone;
	timelineInfo.signalSemaphoreValueCount = 1;
	timelineInfo.pSignalSemaphoreValues = &signal;

	VkSemaphore semaphores[] = { timeline };

	VkPipelineStageFlags waitStage =
		VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;

	VkSubmitInfo submit{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
	submit.pNext = &timelineInfo;
	submit.waitSemaphoreCount = 1;
	submit.pWaitSemaphores = semaphores;
	submit.pWaitDstStageMask = &waitStage;
	submit.signalSemaphoreCount = 1;
	submit.pSignalSemaphores = semaphores;
	submit.commandBufferCount = 1;
	submit.pCommandBuffers = &cmd;

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
	vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE);
}

void GpuAtomicComputeStrategy::SubmitReadbackCopy(
	uint32_t slot,
	VkSemaphore timeline)
{
	ScopedCmdBuffer scoped(_device, _computeCommandPool);
	VkCommandBuffer cmd = scoped.Get();

	const size_t C = _cageMesh._vertices.rows();

	VkBufferCopy lambdaCopy{ 0, 0, C * sizeof(float) };
	VkBufferCopy wsumCopy{ 0, 0, sizeof(float) };

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

	vkEndCommandBuffer(cmd);

	uint64_t signal = NextTimelineValue();
	_slotSync[slot].copyDone = signal;

	VkTimelineSemaphoreSubmitInfo timelineInfo{
		VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO
	};
	timelineInfo.waitSemaphoreValueCount = 1;
	timelineInfo.pWaitSemaphoreValues = &_slotSync[slot].computeDone;
	timelineInfo.signalSemaphoreValueCount = 1;
	timelineInfo.pSignalSemaphoreValues = &signal;

	VkPipelineStageFlags waitStage =
		VK_PIPELINE_STAGE_TRANSFER_BIT;

	VkSubmitInfo submit{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
	submit.pNext = &timelineInfo;
	submit.waitSemaphoreCount = 1;
	submit.pWaitSemaphores = &timeline;
	submit.pWaitDstStageMask = &waitStage;
	submit.signalSemaphoreCount = 1;
	submit.pSignalSemaphores = &timeline;
	submit.commandBufferCount = 1;
	submit.pCommandBuffers = &cmd;

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
	vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE);
}

void GpuAtomicComputeStrategy::ConsumeSlot(
	uint32_t deformableIndex,
	uint32_t slot,
	VkSemaphore timeline)
{
	VkSemaphoreWaitInfo wait{};
	wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
	wait.semaphoreCount = 1;
	wait.pSemaphores = &timeline;
	wait.pValues = &_slotSync[slot].copyDone;

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

uint64_t GpuAtomicComputeStrategy::GetSlotCompletionValue(uint32_t targetIndex) const
{
	return _slotDoneValue[targetIndex];
}

void GpuAtomicComputeStrategy::Readback(
	uint32_t cubemapIdx,
	uint32_t targetIndex,
	const std::string&)
{

	const size_t C = static_cast<size_t>(_cageMesh._vertices.rows());

	VkBufferCopy lambdaCopy{ 0, 0, C * sizeof(float) };
	VkBufferCopy wsumCopy{ 0, 0, sizeof(float) };

	ScopedCmdBuffer scoped(_device, _computeCommandPool);
	VkCommandBuffer cmd = scoped.Get();

	vkCmdCopyBuffer(
		cmd,
		_slots[targetIndex].lambda._deviceBuffer,
		_slots[targetIndex].lambdaStaging._deviceBuffer,
		1, &lambdaCopy
	);

	vkCmdCopyBuffer(
		cmd,
		_slots[targetIndex].wsum._deviceBuffer,
		_slots[targetIndex].wsumStaging._deviceBuffer,
		1, &wsumCopy
	);

	VkQueue transferQueue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &transferQueue);

	scoped.SubmitAndWait(transferQueue);

	// --------------------------------------------------
	// CPU storage
	// --------------------------------------------------
	const float* lambdaCPU =
		reinterpret_cast<const float*>(_slots[targetIndex].lambdaStaging._mappedData);

	const float wsumCPU =
		*reinterpret_cast<const float*>(_slots[targetIndex].wsumStaging._mappedData);

	// Allocate CPU arrays once
	if (_lambdaResults.empty())
	{
		const size_t D = _deformableMesh._vertices.rows();
		_lambdaResults.resize(D);
		for (auto& row : _lambdaResults)
			row.resize(C, 0.0f);

		_wsumResults.resize(D, 0.0f);
	}

	// Store results (NO accumulation)
	for (size_t c = 0; c < C; ++c)
		_lambdaResults[cubemapIdx][c] = lambdaCPU[c];

	_wsumResults[cubemapIdx] = wsumCPU;
}

void GpuAtomicComputeStrategy::WaitAll(VkSemaphore _timeline)
{
	if (_timelineValue == 0) return;

	VkSemaphoreWaitInfo waitInfo{};
	waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
	waitInfo.semaphoreCount = 1;
	waitInfo.pSemaphores = &_timeline;
	waitInfo.pValues = &_timelineValue;

	vkWaitSemaphores(_device, &waitInfo, UINT64_MAX);
}

void GpuAtomicComputeStrategy::CreatePipelineAndLayouts() {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = _transferQueueFamily;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_computeCommandPool) != VK_SUCCESS)
		throw std::runtime_error("Failed to create compute command pool!");

	std::vector<VkDescriptorSetLayoutBinding> bindings{
		// Image
		{0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT},
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

	ComputePipelineObjectProxy proxy;
	proxy._renderPipelineManager = _renderPipelineManager;
	proxy._shaderModule = "assets/shaders/PMVCComputeAtmoic.comp.spv";
	proxy._descriptorSetLayouts = { _computeLayout };

	VkPushConstantRange range{};
	range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	range.offset = 0;
	range.size = sizeof(ComputePushConstants);
	proxy._pushConstantRanges = { range };

	_computePipeline = proxy.Build();
}

void GpuAtomicComputeStrategy::AllocateResources()
{
	const uint32_t slotCount = static_cast<uint32_t>(_slotDoneValue.size());
	const size_t C = static_cast<size_t>(_cageMesh._vertices.rows());

	_slots.resize(slotCount);

	// --------------------------------------------------
	// Allocate per-slot lambda / wsum buffers
	// --------------------------------------------------
	for (uint32_t i = 0; i < slotCount; ++i)
	{
		const VkDeviceSize lambdaBytes = C * sizeof(float);
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
	// Vertex index list buffer (unchanged)
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

void GpuAtomicComputeStrategy::UpdateComputeDescriptorSet(uint32_t slotIndex)
{
	VkDescriptorImageInfo imageInfo{};
	imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
	imageInfo.imageView = _baryTexImageView;

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

	std::array<VkWriteDescriptorSet, 5> writes{};

	writes[0] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
		_computeDescriptorSets[slotIndex], 0, 0, 1,
		VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &imageInfo };

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

	vkUpdateDescriptorSets(_device, writes.size(), writes.data(), 0, nullptr);
}

void GpuAtomicComputeStrategy::storeLambdaForVertex(const float* lambdaCPU)
{
	const size_t numCageVerts = static_cast<size_t>(_cageMesh._vertices.rows());
	if (numCageVerts == 0) return;

	_lambdaResults.resize(numCageVerts);
	for (int i = 0; i < _lambdaResults.size(); i++) {
		_lambdaResults[i].resize(_deformableMesh._vertices.rows(), 0);
	}

	for (int j = 0; j < _lambdaResults.size(); j++) {
		float* dst = _lambdaResults[j].data();
		for (size_t i = 0; i < numCageVerts; i++)
		{
			dst[i] += lambdaCPU[i];
		}
	}
}

void GpuAtomicComputeStrategy::storeWsumForVertex(const float* wsumCPU)
{
	if (!wsumCPU) return;

	_wsumResults.resize(_deformableMesh._vertices.rows(), 0);

	float* dst = _wsumResults.data();
	for (size_t i = 0; i < _wsumResults.size(); i++)
	{
		dst[i] += wsumCPU[i];
	}
}

void GpuAtomicComputeStrategy::InsertImageMemoryBarrierToGeneral(
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
void GpuAtomicComputeStrategy::CreateSampler() {
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

void GpuAtomicComputeStrategy::CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size)
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

void GpuAtomicComputeStrategy::WriteWeightsToFile(const std::string& filename)
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
	}
	file << "\n===== Pixels Processed per Cubemap(" << _wsumResults.size() << " floats) =====\n";
	for (size_t cub = 0; cub < _wsumResults.size(); ++cub) {
		file << "wsum[" << cub << "] = " << _wsumResults[cub] << "\n";
	}

	file.close();
	LOG_INFO("Weights written to " + filename);
}