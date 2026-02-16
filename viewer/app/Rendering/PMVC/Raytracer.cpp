#include <Rendering/PMVC/Raytracer.h>

#include <Rendering/PMVC/CubemapRenderInstance.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/ScreenPass.h>
#include <Editor/Light.h>
#include <random>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/compatibility.hpp>

#include <cstddef>

#include "ScopedCmdBuffer.h"

Raytracer::Raytracer(const std::shared_ptr<RenderPipelineManager>& pipelineManager,
	const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device) :
	_pipelineManager(pipelineManager),
	_resourceManager(resourceManager),
	_device(device)
{
}

Raytracer::~Raytracer()
{
}

void Raytracer::StartRayTrace() {
	LOG_INFO("Starting ray trace dispatch...");

	// Clean up any previous trace resources
	if (_traceSync.isTracing) {
		LOG_WARN("Previous trace still in progress, waiting...");
		WaitForTrace();
	}

	// 1. Reset hit buffer before tracing
	ResetHitBuffer();

	// 2. Create command buffer if needed
	if (_traceSync.commandBuffer == VK_NULL_HANDLE) {
		VkCommandBufferAllocateInfo allocInfo{};
		allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		allocInfo.commandPool = _commandPool;
		allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		allocInfo.commandBufferCount = 1;

		VK_CHECK(vkAllocateCommandBuffers(_device, &allocInfo, &_traceSync.commandBuffer));
	}

	// 3. Create fence if needed
	if (_traceSync.fence == VK_NULL_HANDLE) {
		VkFenceCreateInfo fenceInfo{};
		fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;  // Start signaled
		VK_CHECK(vkCreateFence(_device, &fenceInfo, nullptr, &_traceSync.fence));
	}

	// 4. Wait for fence to be signaled (from previous trace)
	vkWaitForFences(_device, 1, &_traceSync.fence, VK_TRUE, UINT64_MAX);
	vkResetFences(_device, 1, &_traceSync.fence);

	// 5. Reset command buffer
	vkResetCommandBuffer(_traceSync.commandBuffer, 0);

	// 6. Begin command buffer
	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK_CHECK(vkBeginCommandBuffer(_traceSync.commandBuffer, &beginInfo));

	// 7. Bind pipeline and descriptors
	vkCmdBindPipeline(_traceSync.commandBuffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, _rtPipeline.pipeline);

	vkCmdBindDescriptorSets(
		_traceSync.commandBuffer,
		VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
		_rtPipeline.layout,
		0, 1, &_rtPipeline.descriptorSet,
		0, nullptr
	);

	// 8. Push constants
	vkCmdPushConstants(
		_traceSync.commandBuffer,
		_rtPipeline.layout,
		VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR,
		0,
		sizeof(PushConstants),
		&_pushConstants
	);

	// 9. Trace rays
	vkCmdTraceRaysKHR(
		_traceSync.commandBuffer,
		&_raygenRegion,
		&_missRegion,
		&_hitRegion,
		&_callableRegion,
		_pushConstants.vertexCount,
		_pushConstants.raysPerVertex,
		1
	);

	LOG_INFO("  Tracing {} vertices x {} rays = {} total rays",
		_pushConstants.vertexCount,
		_pushConstants.raysPerVertex,
		_pushConstants.vertexCount * _pushConstants.raysPerVertex);

	// 10. End command buffer
	VK_CHECK(vkEndCommandBuffer(_traceSync.commandBuffer));

	// 11. Submit with fence
	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &_traceSync.commandBuffer;

	VK_CHECK(vkQueueSubmit(_queue, 1, &submitInfo, _traceSync.fence));

	// 12. Update state
	_traceSync.isTracing = true;
	_traceSync.frameNumber++;

	LOG_INFO("Ray trace dispatch submitted with fence, frame {}", _traceSync.frameNumber);
}

bool Raytracer::IsTraceComplete() {
	if (!_traceSync.isTracing || _traceSync.fence == VK_NULL_HANDLE) {
		return true;
	}

	VkResult result = vkGetFenceStatus(_device, _traceSync.fence);
	if (result == VK_SUCCESS) {
		_traceSync.isTracing = false;
		LOG_INFO("Trace complete for frame {}", _traceSync.frameNumber);
		return true;
	}
	else if (result == VK_NOT_READY) {
		return false;
	}
	else {
		LOG_ERROR("Fence error: {}", static_cast<int>(result));
		return false;
	}
}

void Raytracer::WaitForTrace() {
	if (!_traceSync.isTracing || _traceSync.fence == VK_NULL_HANDLE) {
		return;
	}

	LOG_INFO("Waiting for trace to complete...");
	auto startTime = std::chrono::high_resolution_clock::now();

	VK_CHECK(vkWaitForFences(_device, 1, &_traceSync.fence, VK_TRUE, UINT64_MAX));

	auto endTime = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

	_traceSync.isTracing = false;
	LOG_INFO("Trace wait completed in {} ms", duration.count());
}

void Raytracer::ResetHitBuffer() {
	LOG_INFO("Resetting hit buffer...");

	uint32_t maxTotalHits = _pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;
	VkDeviceSize hitBufferSize = static_cast<VkDeviceSize>(maxTotalHits) * sizeof(SimpleHit);

	LOG_INFO("  Zeroing {} bytes of hit buffer", hitBufferSize);

	// Create zero data
	std::vector<std::byte> zeroData(static_cast<size_t>(hitBufferSize), std::byte{ 0 });

	// Create staging buffer with zeros
	auto stagingBuffer = _resourceManager->CreateBufferAndCopy(
		std::span(zeroData),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	// Copy zeros to device-local hit buffer
	CopyBuffer(stagingBuffer._deviceBuffer, _rayBuffers.hitBuffer._deviceBuffer, hitBufferSize);

	// Clean up staging buffer
	stagingBuffer.ReleaseResource(_device);

	LOG_INFO("Hit buffer reset complete");
}

void Raytracer::Initialize()
{
	CreateCommandPool();
	vkGetDeviceQueue(
		_device,
		_queueFamilyIndex,
		0,
		&_queue);
	CreateGeometryBuffers(_cageMesh, _cageGeometry);
	CreateGeometryBuffers(_deformableMesh, _deformableGeometry);
	CreateAccelerationStructures();

	CreateRayBuffers();

	CreateRayTracingDescriptorSet();
	UpdateDescriptorSet();

	GetRaytracingComponents();
	LoadShaders();

	CreateRayTracingPipeline();
}

MeshOperationResult<MeshComputeWeightsOperationResult> Raytracer::ComputeCoordinates() {
	StartRayTrace();
	WaitForTrace();
	if (IsTraceComplete()) {
		LOG_INFO("Ray tracing done! Can read back results now");
		// TODO: Read back hit buffer and compute weights
	}

	Eigen::MatrixXd weights;
	Eigen::MatrixXd M = weights;
	Eigen::MatrixXd interpolatedWeights;
	Eigen::MatrixXd psi;
	std::vector<double> psiTri{ };
	std::vector<Eigen::Vector4d> psiQuad{ };

	weights.resize(_deformableMesh._vertices.rows(), _cageMesh._vertices.rows());

	return MeshComputeWeightsOperationResult{ std::move(M),
		std::move(weights),
		std::move(interpolatedWeights),
		std::move(psi),
		std::move(psiTri),
		std::move(psiQuad) };
}

void Raytracer::CreateCommandPool() {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = _queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_commandPool) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create command pool!");
	}
}

void Raytracer::CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size)
{
	// Allocate a temporary one-time command buffer
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandPool = _commandPool;
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
	vkGetDeviceQueue(_device, _queueFamilyIndex, 0, &graphicsQueue);

	vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
	vkQueueWaitIdle(graphicsQueue);

	// Clean up
	vkFreeCommandBuffers(_device, _commandPool, 1, &cmd);
}

void Raytracer::SubmitReadbackCopy(
	uint32_t slot,
	VkSemaphore timeline,
	uint64_t waitValue,
	uint64_t signalValue)
{
	//TODO structural reuse but needs to be adjusted for raytracing use case
	/*
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
	*/
}

void Raytracer::CreateGeometryBuffers(EigenMesh& geometry, GeometryBuffers& geometryBuffers)
{
	const auto& positions = geometry._vertices;
	const auto& faces = geometry._faces;

	std::vector<glm::vec3> vertices;
	std::vector<uint32_t>  indices;

	vertices.reserve(faces.rows() * 3);
	indices.reserve(faces.rows() * 3);

	for (int tri = 0; tri < faces.rows(); ++tri)
	{
		for (int v = 0; v < 3; ++v)
		{
			int idx = faces(tri, v);

			vertices.emplace_back(
				float(positions(idx, 0)),
				float(positions(idx, 1)),
				float(positions(idx, 2)));

			indices.push_back(indices.size());
		}
	}
	
	VkDeviceSize vertexSize = vertices.size() * sizeof(glm::vec3);
	VkDeviceSize indexSize = indices.size() * sizeof(uint32_t);

	auto stagingVertices = _resourceManager->CreateBufferAndCopy(
		std::as_bytes(std::span(vertices)),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

	auto stagingIndices = _resourceManager->CreateBufferAndCopy(
		std::as_bytes(std::span(indices)),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

	geometryBuffers.vertexBuffer = _resourceManager->AllocateDeviceBuffer(
		vertexSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	geometryBuffers.indexBuffer = _resourceManager->AllocateDeviceBuffer(
		indexSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	CopyBuffer(stagingVertices._deviceBuffer, geometryBuffers.vertexBuffer._deviceBuffer, vertexSize);
	CopyBuffer(stagingIndices._deviceBuffer, geometryBuffers.indexBuffer._deviceBuffer, indexSize);

	stagingVertices.ReleaseResource(_device);
	stagingIndices.ReleaseResource(_device);

	geometryBuffers.vertexCount = static_cast<uint32_t>(vertices.size());
	geometryBuffers.indexCount = static_cast<uint32_t>(indices.size());
}

void Raytracer::CreateAccelerationStructures()
{
	CreateBLAS();
	CreateTLAS();
}

void Raytracer::CreateBLAS()
{
	VkAccelerationStructureGeometryTrianglesDataKHR triangles{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR
	};

	triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
	triangles.vertexStride = sizeof(glm::vec3);
	triangles.maxVertex = _cageGeometry.vertexCount - 1;
	triangles.indexType = VK_INDEX_TYPE_UINT32;

	triangles.vertexData.deviceAddress =
		GetBufferAddress(_cageGeometry.vertexBuffer);

	triangles.indexData.deviceAddress =
		GetBufferAddress(_cageGeometry.indexBuffer);

	VkAccelerationStructureGeometryKHR geometry{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR
	};

	geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
	geometry.geometry.triangles = triangles;
	geometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;

	BuildAccelerationStructure(
		geometry,
		_cageGeometry.indexCount / 3,
		VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR,
		_blas);
}

void Raytracer::CreateTLAS()
{
	VkAccelerationStructureInstanceKHR instance{};
	instance.transform = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0
	};

	instance.accelerationStructureReference = _blas.deviceAddress;
	instance.mask = 0xFF;

	auto instanceBuffer = _resourceManager->CreateBufferAndCopy(
		std::as_bytes(std::span(&instance, 1)),
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

	VkAccelerationStructureGeometryKHR geometry{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR
	};

	geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
	geometry.geometry.instances.sType =
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
	geometry.geometry.instances.data.deviceAddress =
		GetBufferAddress(instanceBuffer);

	BuildAccelerationStructure(
		geometry,
		1,
		VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR,
		_tlas);

	instanceBuffer.ReleaseResource(_device);
}

void Raytracer::BuildAccelerationStructure(
	VkAccelerationStructureGeometryKHR& geometry,
	uint32_t primitiveCount,
	VkAccelerationStructureTypeKHR type,
	AccelerationStructure& outAS)
{
	VkAccelerationStructureBuildGeometryInfoKHR buildInfo{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR
	};

	buildInfo.type = type;
	buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
	buildInfo.geometryCount = 1;
	buildInfo.pGeometries = &geometry;

	VkAccelerationStructureBuildSizesInfoKHR sizeInfo{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR
	};

	vkGetAccelerationStructureBuildSizesKHR(
		_device,
		VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
		&buildInfo,
		&primitiveCount,
		&sizeInfo);

	outAS.buffer = _resourceManager->AllocateDeviceBuffer(
		sizeInfo.accelerationStructureSize,
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	VkAccelerationStructureCreateInfoKHR createInfo{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR
	};

	createInfo.buffer = outAS.buffer._deviceBuffer;
	createInfo.size = sizeInfo.accelerationStructureSize;
	createInfo.type = type;

	vkCreateAccelerationStructureKHR(
		_device,
		&createInfo,
		nullptr,
		&outAS.handle);

	auto scratch = _resourceManager->CreateScratchBuffer(
		sizeInfo.buildScratchSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0);

	buildInfo.dstAccelerationStructure = outAS.handle;
	buildInfo.scratchData.deviceAddress =
		GetBufferAddress(scratch);

	VkAccelerationStructureBuildRangeInfoKHR range{
		.primitiveCount = primitiveCount
	};

	VkAccelerationStructureBuildRangeInfoKHR* pRange = &range;

	{
		ScopedCmdBuffer cmd(_device, _commandPool);

		vkCmdBuildAccelerationStructuresKHR(
			cmd.Get(),
			1,
			&buildInfo,
			&pRange);

		cmd.SubmitAndWait(_queue);
	}

	scratch.ReleaseResource(_device);

	VkAccelerationStructureDeviceAddressInfoKHR addrInfo{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR
	};
	addrInfo.accelerationStructure = outAS.handle;

	outAS.deviceAddress =
		vkGetAccelerationStructureDeviceAddressKHR(_device, &addrInfo);
}

VkDeviceAddress Raytracer::GetBufferAddress(const Buffer& buffer) const
{
	VkBufferDeviceAddressInfo info{
		VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO
	};
	info.buffer = buffer._deviceBuffer;

	return vkGetBufferDeviceAddress(_device, &info);
}

void Raytracer::CreateRayBuffers() {
	_pushConstants.vertexCount = static_cast<uint32_t>(_deformableMesh._vertices.rows());
	_pushConstants.raysPerVertex = 64;
	_pushConstants.maxHitsPerRay = 8;
	// Generate random directions
	uint32_t totalRays = _pushConstants.vertexCount * _pushConstants.raysPerVertex;

	LOG_INFO("Setting up ray directions:");

	std::vector<glm::vec4> directions(totalRays);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<float> dist(0.0f, 1.0f);

	for (uint32_t i = 0; i < totalRays; ++i) {
		float theta = 2.0f * glm::pi<float>() * dist(gen);
		float phi = acos(1.0f - 2.0f * dist(gen));

		directions[i].x = sin(phi) * cos(theta);
		directions[i].y = sin(phi) * sin(theta);
		directions[i].z = cos(phi);
		directions[i].w = static_cast<float>(i / _pushConstants.raysPerVertex);
	}

	size_t directionsBufferSize = totalRays * sizeof(glm::vec4);

	// Create staging buffer
	auto stagingDirections = _resourceManager->CreateBufferAndCopy(
		std::span(reinterpret_cast<const std::byte*>(directions.data()), directionsBufferSize),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	// Create device-local buffer
	_rayBuffers.rayDirections = _resourceManager->AllocateDeviceBuffer(
		directionsBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	if (_rayBuffers.rayDirections._deviceBuffer == VK_NULL_HANDLE) {
		throw std::runtime_error("Failed to create ray directions buffer");
	}

	// Copy from staging to device
	CopyBuffer(stagingDirections._deviceBuffer, _rayBuffers.rayDirections._deviceBuffer, directionsBufferSize);
	stagingDirections.ReleaseResource(_device);

	LOG_INFO("  Ray directions buffer created: {}",
		(void*)_rayBuffers.rayDirections._deviceBuffer);

	// Create hit buffer
	uint32_t maxTotalHits = _pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;

	LOG_INFO("  Max total hits: {}", maxTotalHits);

	VkDeviceSize hitBufferSize = static_cast<VkDeviceSize>(maxTotalHits) * 64;

	LOG_INFO("  Hit buffer size: {} bytes", hitBufferSize);

	// Create zero data for staging
	std::vector<std::byte> zeroData(static_cast<size_t>(hitBufferSize), std::byte{ 0 });
	auto stagingHitBuffer = _resourceManager->CreateBufferAndCopy(
		std::span(zeroData),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	// Create device-local buffer
	_rayBuffers.hitBuffer = _resourceManager->AllocateDeviceBuffer(
		hitBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// Copy from staging to device
	CopyBuffer(stagingHitBuffer._deviceBuffer, _rayBuffers.hitBuffer._deviceBuffer, hitBufferSize);
	stagingHitBuffer.ReleaseResource(_device);

	LOG_INFO("  Hit buffer created: {}", (void*)_rayBuffers.hitBuffer._deviceBuffer);
	LOG_INFO("Successfully set up ray directions");
}

void Raytracer::CreateRayTracingDescriptorSet() {
	LOG_INFO("Creating ray tracing descriptor set...");

	// Validate all buffers
	auto validateBuffer = [](const MemoryMappedBuffer& buffer, const std::string& name) {
		if (buffer._deviceBuffer == VK_NULL_HANDLE) {
			LOG_ERROR("{} is null!", name);
			return false;
		}
		return true;
	};

	// 1. CREATE DESCRIPTOR SET LAYOUT
	std::vector<VkDescriptorSetLayoutBinding> bindings = {
		// Binding 0: Acceleration structure
		{
			.binding = 0,
			.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR
		},
		// Binding 1: Deformable vertices
		{
			.binding = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR
		},
		// Binding 2: Ray directions
		{
			.binding = 2,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR
		},
		// Binding 3: Hit buffer (written by hit shader)
		{
			.binding = 3,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR
		},
		// Binding 4: Cage indices
		{
			.binding = 4,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR
		},
		/* Binding 5: Atomic counter for hit buffer(NEW) NEEDED for validation one hit per face only
		{
			.binding = 7,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR
		}*/
	};

	// Create descriptor set layout
	VkDescriptorSetLayoutCreateInfo layoutInfo{
		.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
		.bindingCount = static_cast<uint32_t>(bindings.size()),
		.pBindings = bindings.data()
	};

	if (_rtPipeline.descriptorLayout != VK_NULL_HANDLE) {
		vkDestroyDescriptorSetLayout(_device, _rtPipeline.descriptorLayout, nullptr);
		_rtPipeline.descriptorLayout = VK_NULL_HANDLE;
	}

	VK_CHECK(vkCreateDescriptorSetLayout(_device, &layoutInfo, nullptr, &_rtPipeline.descriptorLayout));
	LOG_INFO("Created descriptor set layout with {} bindings", bindings.size());

	// 2. CREATE DESCRIPTOR POOL
	std::vector<VkDescriptorPoolSize> poolSizes = {
		{
			.type = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,
			.descriptorCount = 1
		},
		{
			.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 4  // deformable vertices, ray directions, hit buffer, cage indices, (atomic counter)
		}
	};

	VkDescriptorPoolCreateInfo poolInfo{
		.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
		.maxSets = 1,
		.poolSizeCount = static_cast<uint32_t>(poolSizes.size()),
		.pPoolSizes = poolSizes.data()
	};

	if (_rtPipeline.descriptorPool != VK_NULL_HANDLE) {
		vkDestroyDescriptorPool(_device, _rtPipeline.descriptorPool, nullptr);
		_rtPipeline.descriptorPool = VK_NULL_HANDLE;
	}

	VK_CHECK(vkCreateDescriptorPool(_device, &poolInfo, nullptr, &_rtPipeline.descriptorPool));
	LOG_INFO("Created descriptor pool");

	// 3. ALLOCATE DESCRIPTOR SET
	VkDescriptorSetAllocateInfo allocInfo{
		.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
		.descriptorPool = _rtPipeline.descriptorPool,
		.descriptorSetCount = 1,
		.pSetLayouts = &_rtPipeline.descriptorLayout
	};

	VK_CHECK(vkAllocateDescriptorSets(_device, &allocInfo, &_rtPipeline.descriptorSet));
	LOG_INFO("Allocated descriptor set");
}

void Raytracer::UpdateDescriptorSet() {
	// 4. UPDATE DESCRIPTOR SET
	std::vector<VkWriteDescriptorSet> descriptorWrites;
	std::vector<VkDescriptorBufferInfo> bufferInfos;

	// Reserve space for all writes
	descriptorWrites.reserve(6);  // TLAS + 5 buffers
	bufferInfos.reserve(5);       // 5 storage buffers

	// Helper lambda to add buffer writes
	auto addBufferWrite = [&](uint32_t binding, const Buffer& buffer,
		VkDeviceSize size, VkShaderStageFlags stageFlags) {
		// Create buffer info
		VkDescriptorBufferInfo bufferInfo{
			.buffer = buffer._deviceBuffer,
			.offset = 0,
			.range = size
		};
		bufferInfos.push_back(bufferInfo);

		// Create write descriptor
		VkWriteDescriptorSet write{
			.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
			.dstSet = _rtPipeline.descriptorSet,
			.dstBinding = binding,
			.dstArrayElement = 0,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.pBufferInfo = &bufferInfos.back()
		};
		descriptorWrites.push_back(write);
	};

	// Binding 0: Acceleration structure (special handling)
	VkWriteDescriptorSetAccelerationStructureKHR accelStructureWrite{
		.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR,
		.accelerationStructureCount = 1,
		.pAccelerationStructures = &_tlas.handle
	};

	VkWriteDescriptorSet accelWrite{
		.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
		.pNext = &accelStructureWrite,
		.dstSet = _rtPipeline.descriptorSet,
		.dstBinding = 0,
		.dstArrayElement = 0,
		.descriptorCount = 1,
		.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR
	};
	descriptorWrites.push_back(accelWrite);

	// Binding 1: Deformable vertices
	VkDeviceSize vertexBufferSize = _pushConstants.vertexCount * 3 * sizeof(float);
	addBufferWrite(1, _deformableGeometry.vertexBuffer, vertexBufferSize,
		VK_SHADER_STAGE_RAYGEN_BIT_KHR);

	// Binding 2: Ray directions
	uint32_t totalRays = _pushConstants.vertexCount * _pushConstants.raysPerVertex;
	VkDeviceSize directionsSize = static_cast<VkDeviceSize>(totalRays) * sizeof(glm::vec4);
	addBufferWrite(2, _rayBuffers.rayDirections, directionsSize,
		VK_SHADER_STAGE_RAYGEN_BIT_KHR);

	// Binding 3: Hit buffer
	uint32_t maxTotalHits = _pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;
	VkDeviceSize hitBufferSize = static_cast<VkDeviceSize>(maxTotalHits) * sizeof(SimpleHit);
	addBufferWrite(3, _rayBuffers.hitBuffer, hitBufferSize,
		VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR);

	// Binding 4: Cage indices
	VkDeviceSize indexBufferSize = static_cast<VkDeviceSize>(_cageMesh._faces.rows() * 3 * sizeof(uint32_t));
	addBufferWrite(4, _cageGeometry.indexBuffer, indexBufferSize,
		VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR);

	// Update all descriptors at once
	vkUpdateDescriptorSets(_device,
		static_cast<uint32_t>(descriptorWrites.size()),
		descriptorWrites.data(),
		0, nullptr);

	LOG_INFO("Updated descriptor set with {} writes", descriptorWrites.size());
}

void Raytracer::LoadShaders() {
	try {
		// Load pre-compiled SPIR-V shaders
		auto raygenCode = LoadSPIRV("Raygen.rgen.spv");
		auto missCode = LoadSPIRV("Miss.rmiss.spv");
		auto hitCode = LoadSPIRV("Hit.rchit.spv");

		_raygenShader = CreateShaderModule(raygenCode);
		_missShader = CreateShaderModule(missCode);
		_hitShader = CreateShaderModule(hitCode);

		LOG_INFO("Loaded SPIR-V shaders from assets/shaders/");
	}
	catch (const std::exception& e) {
		LOG_ERROR("Failed to load shaders: {}", e.what());
		throw;
	}
}

std::vector<uint32_t> Raytracer::LoadSPIRV(const std::string& filename) {
	std::filesystem::path shaderDir = std::filesystem::current_path() / "assets/shaders/";
	std::filesystem::path fullPath = shaderDir / filename;

	std::ifstream file(fullPath, std::ios::ate | std::ios::binary);
	if (!file.is_open()) {
		throw std::runtime_error("Failed to open shader file: " + fullPath.string());
	}

	size_t fileSize = static_cast<size_t>(file.tellg());
	if (fileSize % sizeof(uint32_t) != 0) {
		throw std::runtime_error("Shader file size is not multiple of 4 bytes");
	}

	std::vector<uint32_t> buffer(fileSize / sizeof(uint32_t));
	file.seekg(0);
	file.read(reinterpret_cast<char*>(buffer.data()), fileSize);
	file.close();

	return buffer;
}

VkShaderModule Raytracer::CreateShaderModule(const std::vector<uint32_t>& code) {
	VkShaderModuleCreateInfo createInfo{
		.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
		.codeSize = code.size() * sizeof(uint32_t),
		.pCode = code.data()
	};

	VkShaderModule shaderModule;
	if (vkCreateShaderModule(_device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create shader module");
	}

	return shaderModule;
}

void Raytracer::GetRaytracingComponents()
{
	VkPhysicalDeviceProperties2 prop2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
	_rtProperties.pNext = &_asProperties;
	prop2.pNext = &_rtProperties;
	vkGetPhysicalDeviceProperties2(_device->GetPhysicalDeviceHandle(), &prop2);
}

void Raytracer::CreateRayTracingPipeline() {
	// Shader stages
	std::vector<VkPipelineShaderStageCreateInfo> stages;

	// Ray generation stage
	VkPipelineShaderStageCreateInfo raygenStage{
		.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
		.stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR,
		.module = _raygenShader,
		.pName = "main"
	};
	stages.push_back(raygenStage);

	// Miss stage
	VkPipelineShaderStageCreateInfo missStage{
		.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
		.stage = VK_SHADER_STAGE_MISS_BIT_KHR,
		.module = _missShader,
		.pName = "main"
	};
	stages.push_back(missStage);

	// Closest hit stage
	VkPipelineShaderStageCreateInfo hitStage{
		.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
		.stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR,
		.module = _hitShader,
		.pName = "main"
	};
	stages.push_back(hitStage);

	// Shader groups
	std::vector<VkRayTracingShaderGroupCreateInfoKHR> groups;
	
	// Raygen group
	VkRayTracingShaderGroupCreateInfoKHR raygenGroup{
		.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR,
		.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR,
		.generalShader = 0, // Index of raygen stage
		.closestHitShader = VK_SHADER_UNUSED_KHR,
		.anyHitShader = VK_SHADER_UNUSED_KHR,
		.intersectionShader = VK_SHADER_UNUSED_KHR
	};
	groups.push_back(raygenGroup);

	// Miss group
	VkRayTracingShaderGroupCreateInfoKHR missGroup{
		.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR,
		.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR,
		.generalShader = 1, // Index of miss stage
		.closestHitShader = VK_SHADER_UNUSED_KHR,
		.anyHitShader = VK_SHADER_UNUSED_KHR,
		.intersectionShader = VK_SHADER_UNUSED_KHR
	};
	groups.push_back(missGroup);

	// Hit group
	VkRayTracingShaderGroupCreateInfoKHR hitGroup{
		.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR,
		.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR,
		.generalShader = VK_SHADER_UNUSED_KHR,
		.closestHitShader = 2, // Index of hit stage
		.anyHitShader = VK_SHADER_UNUSED_KHR,
		.intersectionShader = VK_SHADER_UNUSED_KHR
	};
	groups.push_back(hitGroup);

	// Pipeline layout with push constants
	VkPushConstantRange pushConstantRange{
		.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR,
		.offset = 0,
		.size = sizeof(PushConstants)
	};

	VkPipelineLayoutCreateInfo pipelineLayoutInfo{
		.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
		.setLayoutCount = 1,
		.pSetLayouts = &_rtPipeline.descriptorLayout,
		.pushConstantRangeCount = 1,
		.pPushConstantRanges = &pushConstantRange
	};

	vkCreatePipelineLayout(_device, &pipelineLayoutInfo, nullptr, &_rtPipeline.layout);

	// Create ray tracing pipeline
	VkRayTracingPipelineCreateInfoKHR pipelineInfo{
		.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR,
		.stageCount = static_cast<uint32_t>(stages.size()),
		.pStages = stages.data(),
		.groupCount = static_cast<uint32_t>(groups.size()),
		.pGroups = groups.data(),
		.maxPipelineRayRecursionDepth = _pushConstants.maxHitsPerRay + 1,
		.layout = _rtPipeline.layout
	};

	VK_CHECK(vkCreateRayTracingPipelinesKHR(
		_device,
		VK_NULL_HANDLE,
		VK_NULL_HANDLE,
		1,
		&pipelineInfo,
		nullptr,
		&_rtPipeline.pipeline
	));

	CreateShaderBindingTable(pipelineInfo);
}

void Raytracer::CreateShaderBindingTable(const VkRayTracingPipelineCreateInfoKHR& pipelineInfo) {
	// Get shader group handles
	uint32_t handleSize = _rtProperties.shaderGroupHandleSize;
	uint32_t groupCount = pipelineInfo.groupCount;
	size_t dataSize = handleSize * groupCount;

	_shaderHandles.resize(dataSize);
	VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(
		_device,
		_rtPipeline.pipeline,
		0,
		groupCount,
		dataSize,
		_shaderHandles.data()
	));

	// Calculate sizes and offsets
	auto alignUp = [](uint32_t size, uint32_t alignment) {
		return (size + alignment - 1) & ~(alignment - 1);
	};

	uint32_t handleAlignment = _rtProperties.shaderGroupHandleAlignment;
	uint32_t baseAlignment = _rtProperties.shaderGroupBaseAlignment;

	uint32_t raygenSize = alignUp(handleSize, handleAlignment);
	uint32_t missSize = alignUp(handleSize, handleAlignment);
	uint32_t hitSize = alignUp(handleSize, handleAlignment);

	// Calculate offsets
	uint32_t raygenOffset = 0;
	uint32_t missOffset = alignUp(raygenSize, baseAlignment);
	uint32_t hitOffset = alignUp(missOffset + missSize, baseAlignment);

	size_t sbtSize = hitOffset + hitSize;

	// Create SBT buffer
	_sbt.buffer = _resourceManager->AllocateDeviceBuffer(
		sbtSize,
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);
	// Create a vector with your SBT data
	std::vector<std::byte> sbtData(sbtSize);

	// Copy handles into the vector
	std::memcpy(sbtData.data() + raygenOffset, _shaderHandles.data(), handleSize);
	std::memcpy(sbtData.data() + missOffset, _shaderHandles.data() + handleSize, handleSize);
	std::memcpy(sbtData.data() + hitOffset, _shaderHandles.data() + 2 * handleSize, handleSize);

	// Create buffer with initial data
	auto staging = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>(sbtData.data(), sbtData.size()),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	CopyBuffer(staging._deviceBuffer, _sbt.buffer._deviceBuffer, sbtSize);
	staging.ReleaseResource(_device);

	// Set up SBT regions
	VkBufferDeviceAddressInfo addressInfo{
		.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO,
		.buffer = _sbt.buffer._deviceBuffer
	};
	VkDeviceAddress sbtAddress = vkGetBufferDeviceAddress(_device, &addressInfo);

	_raygenRegion = {
		.deviceAddress = sbtAddress + raygenOffset,
		.stride = raygenSize,
		.size = raygenSize
	};

	_missRegion = {
		.deviceAddress = sbtAddress + missOffset,
		.stride = missSize,
		.size = missSize
	};

	_hitRegion = {
		.deviceAddress = sbtAddress + hitOffset,
		.stride = hitSize,
		.size = hitSize
	};

	_callableRegion = {
		.deviceAddress = 0,
		.stride = 0,
		.size = 0
	};

	LOG_INFO("Shader binding table created");
}