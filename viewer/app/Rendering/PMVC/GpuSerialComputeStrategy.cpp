#include <Rendering/PMVC/GpuSerialComputeStrategy.h>

uint32_t GpuSerialComputeStrategy::RequiredRenderTargetCount() const
{
    return 2;
}

void GpuSerialComputeStrategy::Initialize(uint32_t) {
	uint32_t graphicsQueueFamilyIndex = _transferQueueFamily;
	CreateComputeCommandPool(graphicsQueueFamilyIndex);
	CreateComputeDescriptorSetLayout();
	CreateComputeBuffers();
	AllocateComputeDescriptorSet();
	CreateComputePipeline();
	CreateComputeCommandBuffer();
	//UpdateComputeDescriptorSet();
	//SphereWeightInitialization(512);	#
	CreateSampler();
	VkFenceCreateInfo fenceInfo{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
	fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT; // first use is safe
	vkCreateFence(_device, &fenceInfo, nullptr, &_computeFence);
}
void GpuSerialComputeStrategy::WaitForTargetReuse(uint32_t, VkSemaphore, uint64_t) {}

/*void GpuSerialComputeStrategy::DispatchAfterRender(uint32_t cubemapIdx,
	uint32_t targetIndex,
	VkSemaphore timeline,
	uint64_t renderDoneValue,
	uint64_t copyDoneValue,
	const CubemapRenderTarget& target) {
	_baryTexImageView = target.cubemapView;
	UpdateComputeDescriptorSet();

	vkResetCommandBuffer(_computeCommandBuffer, 0);

	VkCommandBufferBeginInfo begin{};
	begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	vkBeginCommandBuffer(_computeCommandBuffer, &begin);

	// Transition image to GENERAL layout
	VkImageSubresourceRange range = {
		VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6
	};
	InsertImageMemoryBarrierToGeneral(_computeCommandBuffer, target.cubemapImage, range);

	const auto& obj = _cubemapRenderInstance._cubemapManager._renderPipelineManager->GetPipelineObject(_computePipelineHandle);

	vkCmdBindPipeline(_computeCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, obj._handle);
	vkCmdBindDescriptorSets(_computeCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, obj._pipelineLayout,
		0, 1, &_computeDescriptorSet, 0, nullptr);

	ComputePushConstants pc{};
	pc.uNumCubemaps = 1;
	pc.uNumCageVertices = static_cast<int>(_cubemapRenderInstance._cubemapManager._cageMesh._vertices.rows());
	pc.uFaceSize = glm::ivec2(512, 512);
	pc.uFacesPerCubemap = 6;
	pc.uNumTriangles = _cubemapRenderInstance._cubemapManager._cageMesh._faces.rows();

	vkCmdPushConstants(_computeCommandBuffer, obj._pipelineLayout, VK_SHADER_STAGE_COMPUTE_BIT,
		0, sizeof(pc), &pc);

	uint32_t groupsX = (512 + 7) / 8;
	uint32_t groupsY = (512 + 7) / 8;
	uint32_t groupsZ = pc.uFacesPerCubemap; // 6

	vkCmdDispatch(_computeCommandBuffer, groupsX, groupsY, groupsZ);

	vkEndCommandBuffer(_computeCommandBuffer);

	VkSubmitInfo submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit.commandBufferCount = 1;
	submit.pCommandBuffers = &_computeCommandBuffer;

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
	vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE);
	vkQueueWaitIdle(queue); // Serial: wait immediately
}*/

void GpuSerialComputeStrategy::DispatchAfterRender(
	uint32_t cubemapIdx,
	uint32_t targetIndex,
	VkSemaphore,
	uint64_t,
	uint64_t,
	const CubemapRenderTarget& target)
{
	// ---- 1. Wait until previous compute finished ----
	vkWaitForFences(_device, 1, &_computeFence, VK_TRUE, UINT64_MAX);
	vkResetFences(_device, 1, &_computeFence);

	// ---- 2. Update descriptors ----
	_baryTexImageView = target.cubemapView;
	UpdateComputeDescriptorSet();

	// ---- 3. Reset + record command buffer ----
	vkResetCommandBuffer(_computeCommandBuffer, 0);

	VkCommandBufferBeginInfo begin{};
	begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	vkBeginCommandBuffer(_computeCommandBuffer, &begin);

	// Transition cubemap to GENERAL
	VkImageSubresourceRange range{
		VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6
	};
	InsertImageMemoryBarrierToGeneral(
		_computeCommandBuffer,
		target.cubemapImage,
		range
	);

	const auto& obj =
		_cubemapRenderInstance
		._cubemapManager
		._renderPipelineManager
		->GetPipelineObject(_computePipelineHandle);

	vkCmdBindPipeline(
		_computeCommandBuffer,
		VK_PIPELINE_BIND_POINT_COMPUTE,
		obj._handle
	);

	vkCmdBindDescriptorSets(
		_computeCommandBuffer,
		VK_PIPELINE_BIND_POINT_COMPUTE,
		obj._pipelineLayout,
		0, 1,
		&_computeDescriptorSet,
		0, nullptr
	);

	ComputePushConstants pc{};
	pc.uNumCubemaps = 1;
	pc.uNumCageVertices = static_cast<int>(
		_cubemapRenderInstance
		._cubemapManager
		._cageMesh
		._vertices.rows()
		);
	pc.uFaceSize = { 512, 512 };
	pc.uFacesPerCubemap = 6;
	pc.uNumTriangles =
		_cubemapRenderInstance
		._cubemapManager
		._cageMesh
		._faces.rows();

	vkCmdPushConstants(
		_computeCommandBuffer,
		obj._pipelineLayout,
		VK_SHADER_STAGE_COMPUTE_BIT,
		0,
		sizeof(pc),
		&pc
	);

	const uint32_t groupsX = (512 + 7) / 8;
	const uint32_t groupsY = (512 + 7) / 8;
	const uint32_t groupsZ = 6;

	vkCmdDispatch(_computeCommandBuffer, groupsX, groupsY, groupsZ);

	vkEndCommandBuffer(_computeCommandBuffer);

	// ---- 4. Submit with fence ----
	VkSubmitInfo submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit.commandBufferCount = 1;
	submit.pCommandBuffers = &_computeCommandBuffer;

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);
	vkQueueSubmit(queue, 1, &submit, _computeFence);
}



uint64_t GpuSerialComputeStrategy::GetSlotCompletionValue(uint32_t) const { return 0; }

void GpuSerialComputeStrategy::Readback(uint32_t cubemapIdx,
	uint32_t targetIndex,
	const std::string& filename) {
	const uint32_t numCubemaps = static_cast<uint32_t>(_lambdaResults.size());
	const uint32_t numVerts = static_cast<uint32_t>(_cubemapRenderInstance._cubemapManager._cageMesh._vertices.rows());
	const VkDeviceSize lambdaBytes = numCubemaps * numVerts * sizeof(float);
	const VkDeviceSize wsumBytes = numCubemaps * sizeof(float);

	// Copy from GPU device to host-visible staging buffers
	//VkCommandBuffer cmd = _cubemapRenderInstance._cubemapManager.BeginOneTimeCommands();

	VkBufferCopy lambdaCopy{ 0, 0, lambdaBytes };
	VkBufferCopy wsumCopy{ 0, 0, wsumBytes };

	//vkCmdCopyBuffer(cmd,
		//_lambdaBuffer._deviceBuffer,
		//_lambdaStagingBuffer._deviceBuffer,
		//1, &lambdaCopy);

	//vkCmdCopyBuffer(cmd,
		//_wsumBuffer._deviceBuffer,
		//_wsumStagingBuffer._deviceBuffer,
		//1, &wsumCopy);

	//_cubemapRenderInstance._cubemapManager.EndOneTimeCommands(cmd);

	// Map and read from staging buffer
	const float* lambdaCPU = reinterpret_cast<float*>(_lambdaStagingBuffer._mappedData);
	const float* wsumCPU = reinterpret_cast<float*>(_wsumStagingBuffer._mappedData);

	// Resize containers
	_lambdaResults.resize(numCubemaps);
	_wsumResults.resize(numCubemaps);

	for (uint32_t cubeIdx = 0; cubeIdx < numCubemaps; ++cubeIdx)
	{
		_lambdaResults[cubeIdx].assign(lambdaCPU + cubeIdx * numVerts,
			lambdaCPU + (cubeIdx + 1) * numVerts);
		_wsumResults[cubeIdx] = wsumCPU[cubeIdx];
	}

	WriteWeightsToFile(filename);
}

void GpuSerialComputeStrategy::WaitAll(VkSemaphore) {}

//TODO resource manager, cage mesh, weights, device, pipeline manager, graphic command pool
// define which image we are processing currently
//endonetimecvommandbuffer extra class
//make init depenedent in compute type overload function->different compute pmvc functions

void GpuSerialComputeStrategy::CreateComputeCommandPool(uint32_t queueFamilyIndex)
{
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_computeCommandPool) != VK_SUCCESS)
		throw std::runtime_error("Failed to create compute command pool!");
}

void GpuSerialComputeStrategy::CreateComputeDescriptorSetLayout() {
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

	_computeLayout = _cubemapRenderInstance._cubemapManager._descriptorPool->CreateDescriptorSetLayout(bindings);
}

void GpuSerialComputeStrategy::CreateComputeBuffers()
{
	// -------------------------------
	// Lambda / wsum buffers (ONE cubemap per dispatch)
	// -------------------------------
	const size_t numVertices =
		static_cast<size_t>(_cubemapRenderInstance._cubemapManager._cageMesh._vertices.rows());

	const size_t lambdaBytes = numVertices * sizeof(float);
	const size_t wsumBytes = sizeof(float);

	_lambdaStagingBuffer = _cubemapRenderInstance._cubemapManager._resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>((std::byte*)nullptr, lambdaBytes),
		VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_wsumStagingBuffer = _cubemapRenderInstance._cubemapManager._resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>((std::byte*)nullptr, wsumBytes),
		VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_lambdaBuffer = _cubemapRenderInstance._cubemapManager._resourceManager->AllocateDeviceBuffer(
		lambdaBytes,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	_wsumBuffer = _cubemapRenderInstance._cubemapManager._resourceManager->AllocateDeviceBuffer(
		wsumBytes,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// -------------------------------
	// Vertex index list (flatten Eigen faces)
	// -------------------------------
	const int triCount = _cubemapRenderInstance._cubemapManager._cageMesh._faces.rows();
	if (_cubemapRenderInstance._cubemapManager._cageMesh._faces.cols() != 3) {
		throw std::runtime_error("EigenMesh faces must be T x 3.");
	}

	std::vector<uint32_t> vertexList(static_cast<size_t>(triCount) * 3);

	for (int t = 0; t < triCount; ++t) {
		vertexList[t * 3 + 0] =
			static_cast<uint32_t>(_cubemapRenderInstance._cubemapManager._cageMesh._faces(t, 0));
		vertexList[t * 3 + 1] =
			static_cast<uint32_t>(_cubemapRenderInstance._cubemapManager._cageMesh._faces(t, 1));
		vertexList[t * 3 + 2] =
			static_cast<uint32_t>(_cubemapRenderInstance._cubemapManager._cageMesh._faces(t, 2));
	}

	const size_t vertexListBytes =
		vertexList.size() * sizeof(uint32_t);

	// -------------------------------
	// Upload vertex list (EXPLICIT memcpy)
	// -------------------------------
	auto vertexListStaging = _cubemapRenderInstance._cubemapManager._resourceManager->CreateBufferAndCopy(
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

	_vertexListBuffer = _cubemapRenderInstance._cubemapManager._resourceManager->AllocateDeviceBuffer(
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
}

void GpuSerialComputeStrategy::AllocateComputeDescriptorSet() {
	VkDescriptorSetLayout layout = _computeLayout->GetReference();

	VkDescriptorSetAllocateInfo alloc{};
	alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	alloc.descriptorPool = _cubemapRenderInstance._cubemapManager._descriptorPool;
	alloc.descriptorSetCount = 1;
	alloc.pSetLayouts = &layout;

	vkAllocateDescriptorSets(_device, &alloc, &_computeDescriptorSet);
}

void GpuSerialComputeStrategy::UpdateComputeDescriptorSet() {

	VkDescriptorImageInfo imageInfo{};
	imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
	imageInfo.imageView = _baryTexImageView;
	imageInfo.sampler = _sampler;

	VkDescriptorBufferInfo vertexListInfo{};
	vertexListInfo.buffer = _vertexListBuffer._deviceBuffer;
	vertexListInfo.range = VK_WHOLE_SIZE;

	VkDescriptorBufferInfo lambdaInfo{};
	lambdaInfo.buffer = _lambdaBuffer._deviceBuffer;
	lambdaInfo.range = VK_WHOLE_SIZE;

	VkDescriptorBufferInfo wsumInfo{};
	wsumInfo.buffer = _wsumBuffer._deviceBuffer;
	wsumInfo.range = VK_WHOLE_SIZE;

	VkDescriptorImageInfo weightInfo{};
	weightInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	//weightInfo.imageView = _cubemapRenderInstance._cubemapManager._solidAngleArrayView;
	//weightInfo.sampler = _cubemapRenderInstance._cubemapManager._solidAngleSampler;

	std::array<VkWriteDescriptorSet, 5> writes{};

	writes[0] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 0, 0, 1,
				  VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &imageInfo, nullptr, nullptr };
	writes[1] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 1, 0, 1,
				  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &vertexListInfo, nullptr };
	writes[2] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 2, 0, 1,
				  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &lambdaInfo, nullptr };
	writes[3] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 3, 0, 1,
				  VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &wsumInfo, nullptr };
	writes[4] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, _computeDescriptorSet, 4, 0,	1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, &weightInfo, nullptr, nullptr };

	vkUpdateDescriptorSets(_device, writes.size(), writes.data(), 0, nullptr);
}

void GpuSerialComputeStrategy::CreateComputePipeline() {
	ComputePipelineObjectProxy proxy;
	proxy._renderPipelineManager = _cubemapRenderInstance._cubemapManager._renderPipelineManager;
	proxy._shaderModule = "assets/shaders/PMVCCompute.comp.spv";
	proxy._descriptorSetLayouts = { _computeLayout };

	VkPushConstantRange range{};
	range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
	range.offset = 0;
	range.size = sizeof(ComputePushConstants);
	proxy._pushConstantRanges = { range };

	_computePipelineHandle = proxy.Build();
}

void GpuSerialComputeStrategy::CreateComputeCommandBuffer() {
	VkCommandBufferAllocateInfo alloc{};
	alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	alloc.commandPool = _computeCommandPool;
	alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	alloc.commandBufferCount = 1;

	vkAllocateCommandBuffers(_device, &alloc, &_computeCommandBuffer);
}

void GpuSerialComputeStrategy::ComputeCoordinates(uint32_t cubeIndex)
{
	// We only have one cubemap image & view, reused for each vertex.
	// So we always use index 0 here.
	//(void)cubeIndex; // cubeIndex only used to index CPU-side result arrays, not GPU resources

	uint32_t numTriangles = static_cast<int>(_cubemapRenderInstance._cubemapManager._cageMesh._faces.rows());
	//LOG_DEBUG(numTriangles);
	uint32_t numCageVertices = static_cast<int>(_cubemapRenderInstance._cubemapManager._cageMesh._vertices.rows());
	//LOG_DEBUG(numCageVertices);

	_baryTexImageView = _baryTexImageViews[0];
	UpdateComputeDescriptorSet();

	vkResetCommandBuffer(_computeCommandBuffer, 0);

	VkCommandBufferBeginInfo begin{};
	begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

	vkBeginCommandBuffer(_computeCommandBuffer, &begin);

	VkImageSubresourceRange subresourceRange{};
	subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	subresourceRange.baseMipLevel = 0;
	subresourceRange.levelCount = 1;
	subresourceRange.baseArrayLayer = 0;
	subresourceRange.layerCount = 6;

	// Transition the *single* cubemap image we are reusing:
	InsertImageMemoryBarrierToGeneral(
		_computeCommandBuffer,
		_cubemapRenderInstance._cubemapRenderUnit.targets[0].cubemapImage,
		subresourceRange
	);

	const PipelineObject& obj =
		_cubemapRenderInstance._cubemapManager._renderPipelineManager->GetPipelineObject(_computePipelineHandle);

	vkCmdBindPipeline(_computeCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, obj._handle);

	vkCmdBindDescriptorSets(
		_computeCommandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
		obj._pipelineLayout, 0, 1,
		&_computeDescriptorSet, 0, nullptr);

	// --- Push constants: we are only processing ONE cubemap on this dispatch ---
	ComputePushConstants pc{};
	pc.uNumCubemaps = 1;
	pc.uNumCageVertices = static_cast<int>(_cubemapRenderInstance._cubemapManager._cageMesh._vertices.rows());
	pc.uFaceSize = glm::ivec2(512, 512);
	pc.uFacesPerCubemap = 6;
	pc.uNumTriangles = _cubemapRenderInstance._cubemapManager._cageMesh._faces.rows();

	//LOG_DEBUG(static_cast<int>(_cageMesh._faces.size()) + _cageMesh._vertices.rows() + " triangles in cage mesh.\n");

	vkCmdPushConstants(
		_computeCommandBuffer,
		obj._pipelineLayout,
		VK_SHADER_STAGE_COMPUTE_BIT,
		0,
		sizeof(ComputePushConstants),
		&pc
	);

	uint32_t groupsX = (512 + 7) / 8;
	uint32_t groupsY = (512 + 7) / 8;

	//vkCmdDispatch(_computeCommandBuffer, groupsX, groupsY, 1);  // one cubemap
	int indexCount = _cubemapRenderInstance._cubemapManager._cageMesh._faces.size();
	//LOG_DEBUG("index count : " + std::to_string(indexCount));
	int triCount = _cubemapRenderInstance._cubemapManager._cageMesh._faces.rows(); //(and assert indexCount % 3 == 0)
	//LOG_DEBUG("triangle count : " + std::to_string(triCount));
	//LOG_DEBUG("lambda gpu size: " + std::to_string(_lambdaBuffer._allocatedSize));
	//LOG_DEBUG("wsum gpu size: " + std::to_string(_wsumBuffer._allocatedSize));

//vkCmdDispatch(_computeCommandBuffer, 1, 1, 1);
	vkCmdDispatch(_computeCommandBuffer, groupsX, groupsY, static_cast<int>(_cubemapRenderInstance._cubemapManager._cageMesh._vertices.rows() * 6));

	vkEndCommandBuffer(_computeCommandBuffer);

	VkSubmitInfo submit{};
	submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit.commandBufferCount = 1;
	submit.pCommandBuffers = &_computeCommandBuffer;

	VkQueue queue;
	vkGetDeviceQueue(_device, _transferQueueFamily, 0, &queue);

	vkQueueSubmit(queue, 1, &submit, VK_NULL_HANDLE);
	vkQueueWaitIdle(queue);
}

void GpuSerialComputeStrategy::ReadbackCompute(uint32_t cubeIndex)
{
	//VkCommandBuffer cmd = _cubemapRenderInstance._cubemapManager.BeginOneTimeCommands();

	const VkDeviceSize lambdaBytes = static_cast<VkDeviceSize>(_cubemapRenderInstance._cubemapManager._cageMesh._vertices.rows()) * sizeof(float);
	const VkDeviceSize wsumBytes = sizeof(float);

	VkBufferCopy lambdaCopy{ 0, 0, lambdaBytes };
	//vkCmdCopyBuffer(cmd,
		//_lambdaBuffer._deviceBuffer,
		//_lambdaStagingBuffer._deviceBuffer,
		//1, &lambdaCopy);

	VkBufferCopy wsumCopy{ 0, 0, wsumBytes };
	//vkCmdCopyBuffer(cmd,
		//_wsumBuffer._deviceBuffer,
		//_wsumStagingBuffer._deviceBuffer,
		//1, &wsumCopy);

	//_cubemapRenderInstance._cubemapManager.EndOneTimeCommands(cmd);

	float* lambdaCPU = reinterpret_cast<float*>(_lambdaStagingBuffer._mappedData);
	float* wsumCPU = reinterpret_cast<float*>(_wsumStagingBuffer._mappedData);

	storeLambdaForVertex(cubeIndex, lambdaCPU);

	// shader writes only wsum[0]
	storeWsumForVertex(cubeIndex, wsumCPU);
}

void GpuSerialComputeStrategy::storeLambdaForVertex(uint32_t cubeIndex, const float* lambdaCPU)
{
	const size_t numCageVerts = static_cast<size_t>(_cubemapRenderInstance._cubemapManager._cageMesh._vertices.rows());
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

void GpuSerialComputeStrategy::storeWsumForVertex(uint32_t cubeIndex, const float* wsumCPU)
{
	if (!wsumCPU) return;

	if (_wsumResults.size() <= cubeIndex)
		_wsumResults.resize(cubeIndex + 1, 0.0f);

	_wsumResults[cubeIndex] = wsumCPU[0];
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
	}
	file << "\n===== Pixels Processed per Cubemap(" << _wsumResults.size() << " floats) =====\n";
	for (size_t cub = 0; cub < _wsumResults.size(); ++cub) {
		file << "wsum[" << cub << "] = " << _wsumResults[cub] << "\n";
	}

	file.close();
	LOG_INFO("Weights written to " + filename);
}

void GpuSerialComputeStrategy::CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size)
{
	// Allocate a temporary one-time command buffer
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandPool = _cubemapRenderInstance._cubemapManager._graphicCommandPool;
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
	vkFreeCommandBuffers(_device, _cubemapRenderInstance._cubemapManager._graphicCommandPool, 1, &cmd);
}

void GpuSerialComputeStrategy::InsertImageMemoryBarrierToGeneral(
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

void GpuSerialComputeStrategy::CreateSampler() {
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