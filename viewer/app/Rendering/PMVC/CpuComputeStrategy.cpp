#include <Rendering/PMVC/CpuComputeStrategy.h>
#include <Rendering/PMVC/ScopedCmdBuffer.h>
#include <Rendering/PMVC/CubemapRenderInstance.h>

uint32_t CpuComputeStrategy::RequiredRenderTargetCount() const
{
	return static_cast<uint32_t>(_deformableMesh._vertices.rows());
}

void CpuComputeStrategy::Initialize()
{
	//_slotSync.resize(_targetCount);
	// Prepare slots
	//_slotDoneValue.resize(_targetCount, 0ull);
	_targetCount = static_cast<int>(_deformableMesh._vertices.rows());
	_slotToDeformableIndex.assign(_targetCount, UINT32_MAX);

	_lambdaResults.resize(
		_deformableMesh._vertices.rows(),
		_cageMesh._vertices.rows()
	);
	_lambdaResults.setZero();
	_wsumResults.resize(_deformableMesh._vertices.rows(), 0.0f);

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

	VkCommandBufferAllocateInfo allocInfo2{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	allocInfo2.commandPool = _computeCommandPool;
	allocInfo2.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo2.commandBufferCount = static_cast<uint32_t>(_copyCommandBuffers.size());

	if (vkAllocateCommandBuffers(_device, &allocInfo2, _copyCommandBuffers.data()) != VK_SUCCESS)
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

Eigen::MatrixXd CpuComputeStrategy::Readback()
{
	for (int i = 0; i < _lambdaResults.rows(); ++i) {
		_lambdaResults.row(i) /= _wsumResults[i];
	}
	//WriteWeightsToFile("CpuComputeStrategy_Readback.txt");
	return _lambdaResults;
}

void CpuComputeStrategy::AllocateResources()
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
/*
void CpuComputeStrategy::CreateSampler() {
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
*/
/*
void CpuComputeStrategy::CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size)
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
*/
void CpuComputeStrategy::WriteWeightsToFile(const std::string& filename)
{
	std::ofstream file(filename);
	if (!file.is_open())
		throw std::runtime_error("Failed to open weight write file!");

	const Eigen::Index rows = _lambdaResults.rows();
	const Eigen::Index cols = _lambdaResults.cols();

	file << "===== LAMBDA BUFFER (rows=" << rows
		<< ", cols=" << cols << ") =====\n";

	for (Eigen::Index r = 0; r < rows; ++r)
	{
		double sum = 0.0;

		for (Eigen::Index c = 0; c < cols; ++c)
		{
			const double value = _lambdaResults(r, c);
			file << "lambda[" << r << "][" << c << "] = " << value << "\n";
			sum += value;
		}

		file << "Lambda sum: " << sum << "\n";

		if (static_cast<size_t>(r) < _wsumResults.size())
			file << "Sum normalize: " << _wsumResults[r] << "\n";
		else
			file << "Sum normalize: (missing)\n";
	}

	file.close();
	LOG_INFO("Weights written to " + filename);
}

void CpuComputeStrategy::CreateDepthSampler() {
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

void CpuComputeStrategy::RecordReadback(
	uint32_t slot,
	const CubemapRenderTarget& target,
	uint32_t deformableIndex)
{
	assert(slot < _computeCommandBuffers.size());
	assert(slot < _computeDescriptorSets.size());
	assert(slot < _slotToDeformableIndex.size());

	_slotToDeformableIndex[slot] = deformableIndex;

	VkCommandBuffer cmd = _computeCommandBuffers[slot];
	VK_CHECK(vkResetCommandBuffer(cmd, 0));

	VkCommandBufferBeginInfo begin{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO
	};
	VK_CHECK(vkBeginCommandBuffer(cmd, &begin));

	UpdateComputeDescriptorSet(slot, target);

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

	ComputePushConstants pc{};
	pc.uFaceSize = { (int)_faceSize, (int)_faceSize };
	pc.uNumTriangles = _cageMesh._faces.rows();

	vkCmdPushConstants(
		cmd,
		pipe._pipelineLayout,
		VK_SHADER_STAGE_COMPUTE_BIT,
		0, sizeof(pc), &pc
	);

	vkCmdFillBuffer(cmd, _slots[slot].lambda._deviceBuffer, 0, VK_WHOLE_SIZE, 0);
	vkCmdFillBuffer(cmd, _slots[slot].wsum._deviceBuffer, 0, VK_WHOLE_SIZE, 0);

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

	uint32_t groupsX = (_faceSize + kDispatchGroupSize - 1) / kDispatchGroupSize;
	uint32_t groupsY = (_faceSize + kDispatchGroupSize - 1) / kDispatchGroupSize;
	vkCmdDispatch(cmd, groupsX, groupsY, 6);

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
}

void CpuComputeStrategy::SubmitAllReadbacks(
	VkSemaphore waitSemaphore,
	uint64_t waitValue,
	VkSemaphore signalSemaphore,
	uint64_t signalValue)
{
	std::vector<VkCommandBufferSubmitInfo> cmdInfos(_computeCommandBuffers.size());
	for (size_t i = 0; i < _computeCommandBuffers.size(); ++i)
	{
		cmdInfos[i] = {
			.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
			.commandBuffer = _computeCommandBuffers[i]
		};
	}

	VkSemaphoreSubmitInfo waitInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = waitSemaphore,
		.value = waitValue,
		.stageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT
	};

	VkSemaphoreSubmitInfo signalInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = signalSemaphore,
		.value = signalValue,
		.stageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT
	};

	VkSubmitInfo2 submit2{
		.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2,
		.waitSemaphoreInfoCount = 1,
		.pWaitSemaphoreInfos = &waitInfo,
		.commandBufferInfoCount = static_cast<uint32_t>(cmdInfos.size()),
		.pCommandBufferInfos = cmdInfos.data(),
		.signalSemaphoreInfoCount = 1,
		.pSignalSemaphoreInfos = &signalInfo
	};

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
	VK_CHECK(vkQueueSubmit2(queue, 1, &submit2, VK_NULL_HANDLE));
}
/*
void CpuComputeStrategy::SubmitAllReadbackCopies(
	VkSemaphore waitSemaphore,
	uint64_t waitValue,
	VkSemaphore signalSemaphore,
	uint64_t signalValue)
{
	for (uint32_t slot = 0; slot < _copyCommandBuffers.size(); ++slot)
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

		vkCmdCopyBuffer(cmd, _slots[slot].lambda._deviceBuffer, _slots[slot].lambdaStaging._deviceBuffer, 1, &lambdaCopy);
		vkCmdCopyBuffer(cmd, _slots[slot].wsum._deviceBuffer, _slots[slot].wsumStaging._deviceBuffer, 1, &wsumCopy);

		VK_CHECK(vkEndCommandBuffer(cmd));
	}

	std::vector<VkCommandBufferSubmitInfo> cmdInfos(_copyCommandBuffers.size());
	for (size_t i = 0; i < _copyCommandBuffers.size(); ++i)
	{
		cmdInfos[i] = {
			.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
			.commandBuffer = _copyCommandBuffers[i]
		};
	}

	VkSemaphoreSubmitInfo waitInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = waitSemaphore,
		.value = waitValue,
		.stageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT
	};

	VkSemaphoreSubmitInfo signalInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = signalSemaphore,
		.value = signalValue,
		.stageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT
	};

	VkSubmitInfo2 submit2{
		.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2,
		.waitSemaphoreInfoCount = 1,
		.pWaitSemaphoreInfos = &waitInfo,
		.commandBufferInfoCount = static_cast<uint32_t>(cmdInfos.size()),
		.pCommandBufferInfos = cmdInfos.data(),
		.signalSemaphoreInfoCount = 1,
		.pSignalSemaphoreInfos = &signalInfo
	};

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
	VK_CHECK(vkQueueSubmit2(queue, 1, &submit2, VK_NULL_HANDLE));
}
*/
float CpuComputeStrategy::ComputeSolidAngle(uint32_t texelX, uint32_t texelY) const
{
	const float size = static_cast<float>(_faceSize);
	const float u = (2.0f * (static_cast<float>(texelX) + 0.5f) / size) - 1.0f;
	const float v = (2.0f * (static_cast<float>(texelY) + 0.5f) / size) - 1.0f;
	const float invResolution = 1.0f / size;

	const float x0 = u - invResolution;
	const float y0 = v - invResolution;
	const float x1 = u + invResolution;
	const float y1 = v + invResolution;

	return AreaElement(x0, y0) - AreaElement(x0, y1)
		- AreaElement(x1, y0) + AreaElement(x1, y1);
}

float CpuComputeStrategy::DecodeDepthSample(const uint8_t* texel) const
{
	switch (_depthFormat)
	{
	case VK_FORMAT_D32_SFLOAT:
		return *reinterpret_cast<const float*>(texel);
	case VK_FORMAT_D32_SFLOAT_S8_UINT:
		return *reinterpret_cast<const float*>(texel);
	case VK_FORMAT_D24_UNORM_S8_UINT:
	{
		const uint32_t packed = *reinterpret_cast<const uint32_t*>(texel);
		const uint32_t depth24 = packed & 0x00FFFFFFu;
		return static_cast<float>(depth24) / 16777215.0f;
	}
	default:
		return 1.0f;
	}
}

void CpuComputeStrategy::CopyImagesToStaging(const CubemapRenderTarget& target, const SlotReadback& slot)
{
	const VkDeviceSize colorFaceSize = VkDeviceSize(_faceSize) * _faceSize * 4 * sizeof(float);
	const VkDeviceSize depthFaceSize = VkDeviceSize(_faceSize) * _faceSize * _depthBytesPerTexel;

	VK_CHECK(vkResetCommandBuffer(_commandBuffer, 0));

	VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK_CHECK(vkBeginCommandBuffer(_commandBuffer, &beginInfo));

	VkImageMemoryBarrier2 barriers[2]{};
	barriers[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
	barriers[0].srcStageMask = VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT;
	barriers[0].srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
	barriers[0].dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
	barriers[0].dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
	barriers[0].oldLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	barriers[0].newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
	barriers[0].image = target.cubemapImage;
	barriers[0].subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	barriers[0].subresourceRange.baseMipLevel = 0;
	barriers[0].subresourceRange.levelCount = 1;
	barriers[0].subresourceRange.baseArrayLayer = 0;
	barriers[0].subresourceRange.layerCount = 6;

	barriers[1].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
	barriers[1].srcStageMask = VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
	barriers[1].srcAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
	barriers[1].dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
	barriers[1].dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
	barriers[1].oldLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
	barriers[1].newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
	barriers[1].image = target.depthImage;
	barriers[1].subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
	barriers[1].subresourceRange.baseMipLevel = 0;
	barriers[1].subresourceRange.levelCount = 1;
	barriers[1].subresourceRange.baseArrayLayer = 0;
	barriers[1].subresourceRange.layerCount = 6;

	if (VulkanUtils::FormatHasStencilComponent(_depthFormat))
	{
		barriers[1].subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
	}

	VkDependencyInfo barrierInfo{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
	barrierInfo.imageMemoryBarrierCount = 2;
	barrierInfo.pImageMemoryBarriers = barriers;
	vkCmdPipelineBarrier2(_commandBuffer, &barrierInfo);

	std::array<VkBufferImageCopy, 6> colorRegions{};
	std::array<VkBufferImageCopy, 6> depthRegions{};
	for (uint32_t face = 0; face < 6; ++face)
	{
		colorRegions[face].bufferOffset = colorFaceSize * face;
		colorRegions[face].imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		colorRegions[face].imageSubresource.mipLevel = 0;
		colorRegions[face].imageSubresource.baseArrayLayer = face;
		colorRegions[face].imageSubresource.layerCount = 1;
		colorRegions[face].imageExtent = { _faceSize, _faceSize, 1 };

		depthRegions[face].bufferOffset = depthFaceSize * face;
		depthRegions[face].imageSubresource.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
		depthRegions[face].imageSubresource.mipLevel = 0;
		depthRegions[face].imageSubresource.baseArrayLayer = face;
		depthRegions[face].imageSubresource.layerCount = 1;
		depthRegions[face].imageExtent = { _faceSize, _faceSize, 1 };
	}

	vkCmdCopyImageToBuffer(
		_commandBuffer,
		target.cubemapImage,
		VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
		slot.colorBuffer,
		static_cast<uint32_t>(colorRegions.size()),
		colorRegions.data());

	vkCmdCopyImageToBuffer(
		_commandBuffer,
		target.depthImage,
		VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
		slot.depthBuffer,
		static_cast<uint32_t>(depthRegions.size()),
		depthRegions.data());

	barriers[0].srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
	barriers[0].srcAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
	barriers[0].dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
	barriers[0].dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
	barriers[0].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
	barriers[0].newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

	barriers[1].srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
	barriers[1].srcAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
	barriers[1].dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
	barriers[1].dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
	barriers[1].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
	barriers[1].newLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

	vkCmdPipelineBarrier2(_commandBuffer, &barrierInfo);

	VK_CHECK(vkEndCommandBuffer(_commandBuffer));

	VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &_commandBuffer;
	VK_CHECK(vkQueueSubmit(_queue, 1, &submitInfo, VK_NULL_HANDLE));
	VK_CHECK(vkQueueWaitIdle(_queue));
}

void CpuComputeStrategy::ComputeOnCpu(uint32_t deformableIndex, const SlotReadback& slot)
{
	void* colorRaw = nullptr;
	void* depthRaw = nullptr;

	const VkDeviceSize colorSize = VkDeviceSize(_faceSize) * _faceSize * 6 * 4 * sizeof(float);
	const VkDeviceSize depthSize = VkDeviceSize(_faceSize) * _faceSize * 6 * _depthBytesPerTexel;

	VK_CHECK(vkMapMemory(_device, slot.colorMemory, 0, colorSize, 0, &colorRaw));
	VK_CHECK(vkMapMemory(_device, slot.depthMemory, 0, depthSize, 0, &depthRaw));

	const float* colors = reinterpret_cast<const float*>(colorRaw);
	const uint8_t* depthBytes = reinterpret_cast<const uint8_t*>(depthRaw);

	std::vector<float> lambda(_cageMesh._vertices.rows(), 0.0f);
	float wsum = 0.0f;

	const uint32_t numTriangles = static_cast<uint32_t>(_cageMesh._faces.size() / 3);
	const size_t texelCountPerFace = static_cast<size_t>(_faceSize) * _faceSize;

	for (uint32_t face = 0; face < 6; ++face)
	{
		for (uint32_t y = 0; y < _faceSize; ++y)
		{
			for (uint32_t x = 0; x < _faceSize; ++x)
			{
				const size_t texelIdx = size_t(face) * texelCountPerFace + size_t(y) * _faceSize + x;
				const size_t colorBase = texelIdx * 4;

				const float b0 = colors[colorBase + 0];
				const float b1 = colors[colorBase + 1];
				const float b2 = colors[colorBase + 2];

				const uint32_t tri = static_cast<uint32_t>(colors[colorBase + 3] * float(numTriangles) + 0.5f);
				if (tri >= numTriangles)
				{
					continue;
				}

				const float depth = DecodeDepthSample(depthBytes + texelIdx * _depthBytesPerTexel);
				if (depth >= kDepthEpsilon)
				{
					continue;
				}

				const float depthWeight = 1.0f - depth;
				const float solidAngle = _solidAngles[texelIdx];
				const float w = solidAngle * depthWeight;
				if (w <= 0.0f)
				{
					continue;
				}

				const uint32_t i0 = _vertexList[tri * 3 + 0];
				const uint32_t i1 = _vertexList[tri * 3 + 1];
				const uint32_t i2 = _vertexList[tri * 3 + 2];

				lambda[i0] += b0 * w;
				lambda[i1] += b1 * w;
				lambda[i2] += b2 * w;

				wsum += w;
			}
		}
	}

	for (size_t c = 0; c < lambda.size(); ++c)
	{
		_lambdaResults(deformableIndex, static_cast<Eigen::Index>(c)) = lambda[c];
	}
	_wsumResults[deformableIndex] = wsum;

	vkUnmapMemory(_device, slot.colorMemory);
	vkUnmapMemory(_device, slot.depthMemory);
}
