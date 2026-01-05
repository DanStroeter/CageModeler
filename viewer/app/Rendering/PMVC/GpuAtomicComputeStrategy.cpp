#include <Rendering/PMVC/GpuAtomicComputeStrategy.h>
#include <Rendering/PMVC/ScopedCmdBuffer.h>
#include <Rendering/PMVC/CubemapRenderInstance.h>

uint32_t GpuAtomicComputeStrategy::RequiredRenderTargetCount() const
{
    return 1;
}

void GpuAtomicComputeStrategy::Initialize(uint32_t targetCount)
{
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
}

void GpuAtomicComputeStrategy::WaitForTargetReuse(
	uint32_t targetIndex, VkSemaphore timeline, uint64_t slotDoneValue)
{
	if (slotDoneValue == 0) return;

	VkSemaphoreWaitInfo waitInfo{};
	waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
	waitInfo.semaphoreCount = 1;
	waitInfo.pSemaphores = &timeline;
	waitInfo.pValues = &slotDoneValue;

	vkWaitSemaphores(_device, &waitInfo, UINT64_MAX);
}

void GpuAtomicComputeStrategy::DispatchAfterRender(uint32_t cubemapIdx,
	uint32_t targetIndex,
	VkSemaphore timeline,
	uint64_t renderDoneValue,
	uint64_t copyDoneValue,
	const CubemapRenderTarget& target)
{
	// Signal value for this slot
	uint64_t signalValue = ++_timelineValue;

	// Update per-target image view
	_baryTexImageView = target.cubemapView;
	UpdateComputeDescriptorSet();

	VkCommandBuffer cmd = _computeCommandBuffers[targetIndex];

	vkResetCommandBuffer(cmd, 0);

	VkCommandBufferBeginInfo begin{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	vkBeginCommandBuffer(cmd, &begin);

	VkImageSubresourceRange range{ VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6 };
	InsertImageMemoryBarrierToGeneral(cmd, target.cubemapImage, range);

	const auto& pipeObj = _renderPipelineManager.get()->GetPipelineObject(_computePipeline);
	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeObj._handle);
	vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
		pipeObj._pipelineLayout, 0, 1, &_computeDescriptorSet, 0, nullptr);

	ComputePushConstants pc{};
	pc.uNumCubemaps = 1;
	pc.uNumCageVertices = static_cast<uint32_t>(_cageMesh._vertices.rows());
	pc.uFaceSize = { (int)_faceSize, (int)_faceSize };
	pc.uFacesPerCubemap = kCubemapFaceCount;
	pc.uNumTriangles = static_cast<uint32_t>(_cageMesh._faces.rows());

	vkCmdPushConstants(cmd, pipeObj._pipelineLayout,
		VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pc), &pc);

	uint32_t groupsX = (_faceSize + kDispatchGroupSize - 1) / kDispatchGroupSize;
	uint32_t groupsY = (_faceSize + kDispatchGroupSize - 1) / kDispatchGroupSize;

	vkCmdDispatch(cmd, groupsX, groupsY, kCubemapFaceCount);
	vkEndCommandBuffer(cmd);

	// Submit with timeline signal
	VkTimelineSemaphoreSubmitInfo timelineInfo{ VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO };
	timelineInfo.signalSemaphoreValueCount = 1;
	timelineInfo.pSignalSemaphoreValues = &signalValue;

	VkSubmitInfo submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit.pNext = &timelineInfo;
	submit.signalSemaphoreCount = 1;
	submit.pSignalSemaphores = &timeline;
	submit.commandBufferCount = 1;
	submit.pCommandBuffers = &cmd;

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
	vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE);

	_slotDoneValue[targetIndex] = signalValue;
}

uint64_t GpuAtomicComputeStrategy::GetSlotCompletionValue(uint32_t targetIndex) const
{
	return _slotDoneValue[targetIndex];
}

void GpuAtomicComputeStrategy::Readback(uint32_t cubemapIdx, uint32_t targetIndex, const std::string&)
{

	ScopedCmdBuffer scoped(_device, _computeCommandPool);
	VkCommandBuffer cmd = scoped.Get();

	VkBufferCopy lambdaCopy{ cubemapIdx * _lambdaBuffer._allocatedSize, 0, _lambdaBuffer._allocatedSize };
	vkCmdCopyBuffer(cmd, _lambdaBuffer._deviceBuffer, _lambdaStagingBuffer._deviceBuffer, 1, &lambdaCopy);

	VkBufferCopy wsumCopy{ cubemapIdx * sizeof(float), 0, sizeof(float) };
	vkCmdCopyBuffer(cmd, _wsumBuffer._deviceBuffer, _wsumStagingBuffer._deviceBuffer, 1, &wsumCopy);

	// Map and store CPU-side
	float* lambdaCPU = reinterpret_cast<float*>(_lambdaStagingBuffer._mappedData);
	float* wsumCPU = reinterpret_cast<float*>(_wsumStagingBuffer._mappedData);

	// store results however you want
	storeLambdaForVertex(cubemapIdx, lambdaCPU);
	storeWsumForVertex(cubemapIdx, wsumCPU);
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

	ComputePipelineObjectProxy proxy;
	proxy._renderPipelineManager = _renderPipelineManager;
	proxy._shaderModule = "assets/shaders/PMVCComputeAtomic.comp.spv";
	proxy._descriptorSetLayouts = { _computeLayout };

	VkPushConstantRange range{};
	range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	range.offset = 0;
	range.size = sizeof(ComputePushConstants);
	proxy._pushConstantRanges = { range };

	_computePipeline = proxy.Build();


}

void GpuAtomicComputeStrategy::AllocateResources(){
	VkDescriptorSetLayout layout = _computeLayout->GetReference();

	VkDescriptorSetAllocateInfo alloc{};
	alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	alloc.descriptorPool = _descriptorPool;
	alloc.descriptorSetCount = 1;
	alloc.pSetLayouts = &layout;

	vkAllocateDescriptorSets(_device, &alloc, &_computeDescriptorSet);

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

	VkCommandBufferAllocateInfo alloc2{};
	alloc2.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	alloc2.commandPool = _computeCommandPool;
	alloc2.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	alloc2.commandBufferCount = 1;

	vkAllocateCommandBuffers(_device, &alloc2, &_computeCommandBuffer);
}

void GpuAtomicComputeStrategy::UpdateComputeDescriptorSet() {

	VkDescriptorImageInfo imageInfo{};
	imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
	imageInfo.imageView = _baryTexImageView;
	imageInfo.sampler = _sampler;

	VkDescriptorBufferInfo vertexListInfo{ _vertexListBuffer._deviceBuffer, 0, VK_WHOLE_SIZE };
	VkDescriptorBufferInfo lambdaInfo{ _lambdaBuffer._deviceBuffer, 0, VK_WHOLE_SIZE };
	VkDescriptorBufferInfo wsumInfo{ _wsumBuffer._deviceBuffer, 0, VK_WHOLE_SIZE };

	// Solid angle array image comes from outside — must be set into class before compute
	VkDescriptorImageInfo weightInfo{};
	weightInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	weightInfo.imageView = _sphereWeightCalculator._solidAngleArrayView;
	weightInfo.sampler = _sphereWeightCalculator._solidAngleSampler;

	std::array<VkWriteDescriptorSet, 5> writes{};
	writes[0] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 0, 0, 1,
				  VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &imageInfo, nullptr, nullptr };
	writes[1] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 1, 0, 1,
				  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &vertexListInfo, nullptr };
	writes[2] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 2, 0, 1,
				  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &lambdaInfo, nullptr };
	writes[3] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 3, 0, 1,
				  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &wsumInfo, nullptr };
	writes[4] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 4, 0, 1,
				  VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &weightInfo, nullptr, nullptr };

	vkUpdateDescriptorSets(_device, writes.size(), writes.data(), 0, nullptr);

}

void GpuAtomicComputeStrategy::storeLambdaForVertex(uint32_t cubeIndex, const float* lambdaCPU)
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

void GpuAtomicComputeStrategy::storeWsumForVertex(uint32_t cubeIndex, const float* wsumCPU)
{
	if (!wsumCPU) return;

	if (_wsumResults.size() <= cubeIndex)
		_wsumResults.resize(cubeIndex + 1, 0.0f);

	_wsumResults[cubeIndex] = wsumCPU[0];
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