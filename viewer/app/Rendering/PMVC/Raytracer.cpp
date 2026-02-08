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


Raytracer::Raytracer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
	const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance, uint32_t cubemapSize, VkFormat format) : _renderPipelineManager(renderPipelineManager),
	_resourceManager(resourceManager), _device(device), _instance(instance)
{ }

Raytracer::~Raytracer()
{
	CleanupShaders();

	// Clean up other resources...
	if (m_rtPipeline != VK_NULL_HANDLE) {
		vkDestroyPipeline(_device, m_rtPipeline, nullptr);
	}
	if (m_rtPipelineLayout != VK_NULL_HANDLE) {
		vkDestroyPipelineLayout(_device, m_rtPipelineLayout, nullptr);
	}
	if (_rtDescSetLayout != VK_NULL_HANDLE) {
		vkDestroyDescriptorSetLayout(_device, _rtDescSetLayout, nullptr);
	}

	// Clean up acceleration structures
	if (m_blasAccel != VK_NULL_HANDLE) {
		vkDestroyAccelerationStructureKHR(_device, m_blasAccel, nullptr);
	}
	if (m_blasMemory != VK_NULL_HANDLE) {
		vkFreeMemory(_device, m_blasMemory, nullptr);
	}

	// Clean up command pool
	if (_commandPool != VK_NULL_HANDLE) {
		vkDestroyCommandPool(_device, _commandPool, nullptr);
	}
}

void Raytracer::Initialize()
{
	// Fixed version:
	CreateCommandPool(_device->GetQueueFamilies()._graphics.value());
	assert(_commandPool != VK_NULL_HANDLE);

	// Get queue AFTER command pool creation
	vkGetDeviceQueue(
		_device,
		_device->GetQueueFamilies()._graphics.value(),
		0,
		&_queue);
	assert(_queue != VK_NULL_HANDLE);  // Check here instead

	GetRaytracingComponents();
	UploadVertexAndIndexBuffers();  // Creates _vertexBuffer and _indexBuffer


	CreateBottomLevelAS();  // Needs _vertexBuffer and _indexBuffer
	// Load shaders BEFORE creating pipeline
	LoadShaders();
	CreateRaytraceDescriptorLayout();

	// Create buffers for ray tracing
	CreateRayTracingBuffers();  // MISSING CALL
	CreateRayTracingDescriptorSet();  // MISSING CALL

	// This should be the version that actually loads shaders
	CreateRaytracingPipeline();

	// Create SBT after pipeline
	// Note: CreateShaderBindingTable() should be called inside CreateRaytracingPipeline()
	// after pipeline creation
}

/*void Raytracer::UploadVertexAndIndexBuffers()
{
	// --- Vertex buffer ---
	const auto& vertPositions = _cageMesh._vertices;
	const auto& vertFaces = _cageMesh._faces;

	std::vector<RTVertex> vertexData;
	vertexData.reserve(vertFaces.rows() * 3);

	for (int tri = 0; tri < vertFaces.rows(); ++tri)
	{
		for (int v = 0; v < 3; ++v)
		{
			int idx = vertFaces(tri, v);
			vertexData.push_back({
				glm::vec3(
					static_cast<float>(vertPositions(idx, 0)),
					static_cast<float>(vertPositions(idx, 1)),
					static_cast<float>(vertPositions(idx, 2))
				)
				});
		}
	}

	// --- Step 1: Create staging vertex buffer (CPU visible) ---
	auto stagingVertex = _resourceManager->CreateBufferAndCopy(
		std::span(vertexData),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	// --- Step 2: Create device-local vertex buffer (GPU only) ---
	_vertexBuffer = _resourceManager->AllocateDeviceBuffer(
		stagingVertex._allocatedSize,
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// --- Step 3: Copy staging -> GPU ---
	CopyBuffer(stagingVertex, _vertexBuffer, stagingVertex._allocatedSize);

	// --- Step 4: Release staging buffer ---
	stagingVertex.ReleaseResource(_device);

	// --- Index buffer ---
	const uint32_t indexCount = static_cast<uint32_t>(vertFaces.rows() * 3);
	std::vector<uint32_t> indices(indexCount);

	for (uint32_t tri = 0; tri < vertFaces.rows(); ++tri)
	{
		for (uint32_t v = 0; v < 3; ++v)
		{
			indices[tri * 3 + v] = vertFaces(tri, v);
		}
	}

	// --- Step 1: Create staging index buffer (CPU visible) ---
	auto stagingIndex = _resourceManager->CreateBufferAndCopy(
		std::span(indices),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	// --- Step 2: Create device-local index buffer (GPU only) ---
	_indexBuffer = _resourceManager->AllocateDeviceBuffer(
		stagingIndex._allocatedSize,
		VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// --- Step 3: Copy staging -> GPU ---
	CopyBuffer(stagingIndex, _indexBuffer, stagingIndex._allocatedSize);

	// --- Step 4: Release staging buffer ---
	stagingIndex.ReleaseResource(_device);
}*/
void Raytracer::UploadVertexAndIndexBuffers()
{
	const auto& positions = _cageMesh._vertices;
	const auto& faces = _cageMesh._faces;

	// -------------------------------------------------
	// 1. Build FLATTENED vertex buffer
	// -------------------------------------------------
	std::vector<RTVertex> vertices;
	vertices.reserve(faces.rows() * 3);

	for (int tri = 0; tri < faces.rows(); ++tri)
	{
		for (int v = 0; v < 3; ++v)
		{
			int idx = faces(tri, v);
			vertices.push_back({
				glm::vec3(
					float(positions(idx, 0)),
					float(positions(idx, 1)),
					float(positions(idx, 2))
				)
				});
		}
	}

	VkDeviceSize vertexBufferSize = sizeof(RTVertex) * vertices.size();

	// -------------------------------------------------
	// 2. Build LINEAR index buffer (CRITICAL FIX)
	// -------------------------------------------------
	std::vector<uint32_t> indices(vertices.size());
	for (uint32_t i = 0; i < indices.size(); ++i)
		indices[i] = i;

	VkDeviceSize indexBufferSize = sizeof(uint32_t) * indices.size();

	// -------------------------------------------------
	// 3. Staging buffers (CPU visible)
	// -------------------------------------------------
	auto stagingVertex = _resourceManager->CreateBufferAndCopy(
		std::span(vertices),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	auto stagingIndex = _resourceManager->CreateBufferAndCopy(
		std::span(indices),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	// -------------------------------------------------
	// 4. Device-local RT buffers
	// -------------------------------------------------
	_vertexBuffer = _resourceManager->AllocateDeviceBuffer(
		vertexBufferSize,
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	_indexBuffer = _resourceManager->AllocateDeviceBuffer(
		indexBufferSize,
		VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// -------------------------------------------------
	// 5. Copy to GPU
	// -------------------------------------------------
	CopyBuffer(stagingVertex, _vertexBuffer, vertexBufferSize);
	CopyBuffer(stagingIndex, _indexBuffer, indexBufferSize);

	stagingVertex.ReleaseResource(_device);
	stagingIndex.ReleaseResource(_device);
}

void Raytracer::CopyBuffer(const Buffer& src, Buffer& dst, VkDeviceSize size)
{
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandPool = _commandPool;
	allocInfo.commandBufferCount = 1;

	VkCommandBuffer cmd;
	VK_CHECK(vkAllocateCommandBuffers(_device, &allocInfo, &cmd));

	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

	VkBufferCopy copyRegion{};
	copyRegion.srcOffset = 0;
	copyRegion.dstOffset = 0;
	copyRegion.size = size;
	vkCmdCopyBuffer(cmd, src._deviceBuffer, dst._deviceBuffer, 1, &copyRegion);

	VK_CHECK(vkEndCommandBuffer(cmd));

	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &cmd;

	VK_CHECK(vkQueueSubmit(_queue, 1, &submitInfo, VK_NULL_HANDLE));
	VK_CHECK(vkQueueWaitIdle(_queue));

	vkFreeCommandBuffers(_device, _commandPool, 1, &cmd);
}

void Raytracer::OnDetach() {
	// Cleanup acceleration structures
	//m_allocator.destroyAcceleration(m_blasAccel);

	//m_allocator.deinit();
	/*if (_rtPipeline != VK_NULL_HANDLE)
		vkDestroyPipeline(_device, m_rtPipeline, nullptr);
	if (_rtPipelineLayout != VK_NULL_HANDLE)
		vkDestroyPipelineLayout(_device, m_rtPipelineLayout, nullptr);
	if (_rtDescSetLayout != VK_NULL_HANDLE)
		vkDestroyDescriptorSetLayout(_device, m_rtDescSetLayout, nullptr);

	if (_sbtBuffer.buffer != VK_NULL_HANDLE)
		vmaDestroyBuffer(m_allocator, m_sbtBuffer.buffer, m_sbtBuffer.allocation);
		*/
}

void Raytracer::CreateCommandPool(uint32_t queueFamilyIndex) {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_commandPool) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create command pool!");
	}
}

void Raytracer::CreateBottomLevelAS()
{
	const uint32_t vertexCount =
		static_cast<uint32_t>(_cageMesh._faces.rows() * 3);

	const uint32_t triangleCount =
		static_cast<uint32_t>(_cageMesh._faces.rows());

	VkAccelerationStructureGeometryKHR geometry{};
	VkAccelerationStructureBuildRangeInfoKHR rangeInfo{};

	PrimitiveToGeometry(
		_vertexBuffer,
		_indexBuffer,
		vertexCount,
		triangleCount,
		geometry,
		rangeInfo
	);

	CreateAccelerationStructure(
		VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR,
		m_blasAccel,
		m_blasMemory,
		_vertexBuffer,
		_indexBuffer,
		vertexCount,
		triangleCount,
		VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR
	);
}

void Raytracer::CreateRaytraceDescriptorLayout()
{
	// Acceleration structure (BLAS used as scene AS)
	VkDescriptorSetLayoutBinding asBinding{};
	asBinding.binding = 0;
	asBinding.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
	asBinding.descriptorCount = 1;
	asBinding.stageFlags =
		VK_SHADER_STAGE_RAYGEN_BIT_KHR |
		VK_SHADER_STAGE_ANY_HIT_BIT_KHR;

	// Output image written by raygen
	VkDescriptorSetLayoutBinding imageBinding{};
	imageBinding.binding = 1;
	imageBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
	imageBinding.descriptorCount = 1;
	imageBinding.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;

	std::array<VkDescriptorSetLayoutBinding, 2> bindings = {
		asBinding,
		imageBinding
	};

	VkDescriptorSetLayoutCreateInfo layoutInfo{};
	layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
	layoutInfo.pBindings = bindings.data();

	VkResult res = vkCreateDescriptorSetLayout(
		_device,
		&layoutInfo,
		nullptr,
		&_rtDescSetLayout
	);

	if (res != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create ray tracing descriptor set layout");
	}
}

void Raytracer::CreateAccelerationStructure(VkAccelerationStructureTypeKHR asType,
	VkAccelerationStructureKHR& accelStruct,
	VkDeviceMemory& accelMemory,
	Buffer& vertexBuffer,
	Buffer& indexBuffer,
	uint32_t vertexCount,
	uint32_t triangleCount,
	VkBuildAccelerationStructureFlagsKHR flags)
{
	VkDevice device = _device; // get Vulkan device handle

	auto alignUp = [](VkDeviceSize value, VkDeviceSize alignment) {
		return (value + alignment - 1) & ~(alignment - 1);
	};

	VkBufferDeviceAddressInfo vAddrInfo{
	VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO
	};
	vAddrInfo.buffer = vertexBuffer._deviceBuffer;
	VkDeviceAddress vertexAddress =
		vkGetBufferDeviceAddress(device, &vAddrInfo);

	VkBufferDeviceAddressInfo iAddrInfo{
		VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO
	};
	iAddrInfo.buffer = indexBuffer._deviceBuffer;
	VkDeviceAddress indexAddress =
		vkGetBufferDeviceAddress(device, &iAddrInfo);


	// --- Describe geometry for BLAS ---
	/*VkAccelerationStructureGeometryTrianglesDataKHR triangles{
		.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR,
		.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT,
		.vertexData = vertexAddress,
		.vertexStride = sizeof(RTVertex),
		.maxVertex = vertexCount - 1,
		.indexType = VK_INDEX_TYPE_UINT32,
		.indexData = indexAddress
	};

	VkAccelerationStructureGeometryKHR geometry{
		.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR,
		.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR,
		.geometry = {.triangles = triangles},
		.flags = VK_GEOMETRY_OPAQUE_BIT_KHR | VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR
	};*/

	VkAccelerationStructureGeometryKHR geometry;
	VkAccelerationStructureBuildRangeInfoKHR buildRangeInfo;

	PrimitiveToGeometry(
		vertexBuffer,
		indexBuffer,
		vertexCount,
		triangleCount,
		geometry,
		buildRangeInfo
	);

	/*VkAccelerationStructureBuildRangeInfoKHR buildRangeInfo{};
	buildRangeInfo.primitiveCount = triangleCount;
	buildRangeInfo.firstVertex = 0;
	buildRangeInfo.primitiveOffset = 0;
	buildRangeInfo.transformOffset = 0;*/

	VkAccelerationStructureBuildGeometryInfoKHR buildInfo{
		.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR,
		.type = asType,
		.flags = flags,
		.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR,
		.geometryCount = 1,
		.pGeometries = &geometry
	};

	uint32_t maxPrimCount = buildRangeInfo.primitiveCount;

	// --- Query sizes ---
	VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
	vkGetAccelerationStructureBuildSizesKHR(device,
		VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
		&buildInfo,
		&maxPrimCount,
		&sizeInfo);

	// --- Create AS buffer ---
	VkBufferCreateInfo bufferInfo{
		.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
		.size = sizeInfo.accelerationStructureSize,
		.usage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		.sharingMode = VK_SHARING_MODE_EXCLUSIVE
	};

	VkBuffer asBuffer;
	vkCreateBuffer(device, &bufferInfo, nullptr, &asBuffer);

	VkMemoryRequirements memReq;
	vkGetBufferMemoryRequirements(device, asBuffer, &memReq);

	VkMemoryAllocateFlagsInfo allocFlags{
	VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO
	};
	allocFlags.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;

	VkMemoryAllocateInfo allocInfo{
		VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO
	};
	allocInfo.allocationSize = memReq.size;
	allocInfo.memoryTypeIndex =
		FindMemoryType(memReq.memoryTypeBits,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	allocInfo.pNext = &allocFlags;

	vkAllocateMemory(device, &allocInfo, nullptr, &accelMemory);
	vkBindBufferMemory(device, asBuffer, accelMemory, 0);

	// --- Create the acceleration structure ---
	VkAccelerationStructureCreateInfoKHR asCreateInfo{
		.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR,
		.buffer = asBuffer,
		.size = sizeInfo.accelerationStructureSize,
		.type = asType
	};
	vkCreateAccelerationStructureKHR(device, &asCreateInfo, nullptr, &accelStruct);

	// --- Scratch buffer ---
	VkDeviceSize scratchSize = alignUp(sizeInfo.buildScratchSize, m_asProperties.minAccelerationStructureScratchOffsetAlignment);

	MemoryMappedBuffer scratchBuffer =
		_resourceManager->CreateScratchBuffer(
			scratchSize,
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
			VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			m_asProperties.minAccelerationStructureScratchOffsetAlignment
		);
	VkBufferDeviceAddressInfo scratchAddressInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
	scratchAddressInfo.buffer = scratchBuffer._deviceBuffer;
	VkDeviceAddress scratchAddress = vkGetBufferDeviceAddress(device, &scratchAddressInfo);

	// --- Build the acceleration structure ---
	buildInfo.dstAccelerationStructure = accelStruct;
	buildInfo.scratchData.deviceAddress = scratchAddress;

	VkAccelerationStructureBuildRangeInfoKHR* pBuildRangeInfo = &buildRangeInfo;

	/*VkCommandBuffer cmd = BeginSingleTimeCommands(device);
	vkCmdBuildAccelerationStructuresKHR(cmd, 1, &buildInfo, &pBuildRangeInfo);
	EndSingleTimeCommands(device, cmd);*/
	{
		ScopedCmdBuffer cmd(_device, _commandPool);

		vkCmdBuildAccelerationStructuresKHR(
			cmd.Get(),
			1,
			&buildInfo,
			&pBuildRangeInfo
		);

		cmd.SubmitAndWait(_queue);
	}

	// --- Cleanup scratch buffer ---
	scratchBuffer.ReleaseResource(_device);
}

void Raytracer::PrimitiveToGeometry(
	Buffer& vertexBuffer,
	Buffer& indexBuffer,
	uint32_t vertexCount,
	uint32_t triangleCount,
	VkAccelerationStructureGeometryKHR& geometry,
	VkAccelerationStructureBuildRangeInfoKHR& rangeInfo)
{
	// --- Get real GPU device addresses ---
	VkBufferDeviceAddressInfo vertexAddrInfo{
		VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO
	};
	vertexAddrInfo.buffer = vertexBuffer._deviceBuffer;
	VkDeviceAddress vertexAddress =
		vkGetBufferDeviceAddress(_device, &vertexAddrInfo);

	VkBufferDeviceAddressInfo indexAddrInfo{
		VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO
	};
	indexAddrInfo.buffer = indexBuffer._deviceBuffer;
	VkDeviceAddress indexAddress =
		vkGetBufferDeviceAddress(_device, &indexAddrInfo);

	// --- Triangle data description ---
	VkAccelerationStructureGeometryTrianglesDataKHR triangles{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR
	};
	triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
	triangles.vertexData.deviceAddress = vertexAddress;
	triangles.vertexStride = sizeof(RTVertex);  // MUST match vertex buffer
	triangles.maxVertex = vertexCount - 1;
	triangles.indexType = VK_INDEX_TYPE_UINT32;
	triangles.indexData.deviceAddress = indexAddress;
	triangles.transformData.deviceAddress = 0;
	

	// --- Geometry description ---
	geometry = VkAccelerationStructureGeometryKHR{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR
	};
	geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
	geometry.geometry.triangles = triangles;
	geometry.flags =
		VK_GEOMETRY_OPAQUE_BIT_KHR |
		VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR;

	// --- Build range info ---
	rangeInfo.primitiveCount = triangleCount;
	rangeInfo.firstVertex = 0;
	rangeInfo.primitiveOffset = 0;
	rangeInfo.transformOffset = 0;
}

void Raytracer::GetRaytracingComponents()
{
	VkPhysicalDeviceProperties2 prop2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
	m_rtProperties.pNext = &m_asProperties;
	prop2.pNext = &m_rtProperties;
	vkGetPhysicalDeviceProperties2(_device->GetPhysicalDeviceHandle(), &prop2);

}

/*void Raytracer::CreateShaderBindingTable()
{
	// Destroy old buffer if exists
	//if (_sbtBuffer.buffer != VK_NULL_HANDLE)
		//vkDestroyBuffer(_device, m_sbtBuffer.buffer, nullptr);

	VkDeviceSize bufferSize = 1024; // Placeholder size

	VkBufferCreateInfo bufferInfo{};
	bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferInfo.size = bufferSize;
	bufferInfo.usage = VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
	bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

	VkResult res = vkCreateBuffer(_device, &bufferInfo, nullptr, &_sbtBuffer._deviceBuffer);
	if (res != VK_SUCCESS)
		throw std::runtime_error("Failed to create SBT buffer");

	VkMemoryRequirements memReqs;
	vkGetBufferMemoryRequirements(_device, _sbtBuffer._deviceBuffer, &memReqs);

	VkMemoryAllocateFlagsInfo allocFlags{};
	allocFlags.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
	allocFlags.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT_KHR;

	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.allocationSize = memReqs.size;
	allocInfo.memoryTypeIndex = FindMemoryType(memReqs.memoryTypeBits,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	allocInfo.pNext = &allocFlags;

	res = vkAllocateMemory(_device, &allocInfo, nullptr, &_sbtBuffer._deviceMemory);
	if (res != VK_SUCCESS)
		throw std::runtime_error("Failed to allocate SBT memory");

	vkBindBufferMemory(_device, _sbtBuffer._deviceBuffer, _sbtBuffer._deviceMemory, 0);

	LOG_INFO("SBT buffer created (placeholder, no shaders yet)");
}*/
void Raytracer::CreateShaderBindingTable(const VkRayTracingPipelineCreateInfoKHR& pipelineInfo) {
	// Get shader group handles
	uint32_t handleSize = m_rtProperties.shaderGroupHandleSize;
	uint32_t groupCount = pipelineInfo.groupCount;
	size_t dataSize = handleSize * groupCount;

	_shaderHandles.resize(dataSize);
	VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(
		_device,
		m_rtPipeline,
		0,
		groupCount,
		dataSize,
		_shaderHandles.data()
	));

	// Calculate sizes and offsets
	auto alignUp = [](uint32_t size, uint32_t alignment) {
		return (size + alignment - 1) & ~(alignment - 1);
	};

	uint32_t handleAlignment = m_rtProperties.shaderGroupHandleAlignment;
	uint32_t baseAlignment = m_rtProperties.shaderGroupBaseAlignment;

	uint32_t raygenSize = alignUp(handleSize, handleAlignment);
	uint32_t missSize = alignUp(handleSize, handleAlignment);
	uint32_t hitSize = alignUp(handleSize, handleAlignment);

	// Calculate offsets
	uint32_t raygenOffset = 0;
	uint32_t missOffset = alignUp(raygenSize, baseAlignment);
	uint32_t hitOffset = alignUp(missOffset + missSize, baseAlignment);

	size_t sbtSize = hitOffset + hitSize;

	// Create SBT buffer
	_sbtBuffer = MemoryMappedBuffer(_resourceManager->AllocateDeviceBuffer(
		sbtSize,
		VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	));

	// Map buffer and copy handles
	auto staging = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>(),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		sbtSize
	);

	uint8_t* sbtData = static_cast<uint8_t*>(staging._mappedData);

	// Copy raygen handle
	memcpy(sbtData + raygenOffset, _shaderHandles.data(), handleSize);

	// Copy miss handle
	memcpy(sbtData + missOffset, _shaderHandles.data() + handleSize, handleSize);

	// Copy hit handle
	memcpy(sbtData + hitOffset, _shaderHandles.data() + 2 * handleSize, handleSize);

	// Copy to device
	{
		ScopedCmdBuffer cmd(_device, _commandPool);
		VkBufferCopy copyRegion{
			.srcOffset = 0,
			.dstOffset = 0,
			.size = sbtSize
		};
		vkCmdCopyBuffer(cmd.Get(), staging._deviceBuffer, _sbtBuffer._deviceBuffer, 1, &copyRegion);
		cmd.SubmitAndWait(_queue);
	}

	staging.ReleaseResource(_device);

	// Set up SBT regions
	VkBufferDeviceAddressInfo addressInfo{
		.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO,
		.buffer = _sbtBuffer._deviceBuffer
	};
	VkDeviceAddress sbtAddress = vkGetBufferDeviceAddress(_device, &addressInfo);

	m_raygenRegion = {
		.deviceAddress = sbtAddress + raygenOffset,
		.stride = raygenSize,
		.size = raygenSize
	};

	m_missRegion = {
		.deviceAddress = sbtAddress + missOffset,
		.stride = missSize,
		.size = missSize
	};

	m_hitRegion = {
		.deviceAddress = sbtAddress + hitOffset,
		.stride = hitSize,
		.size = hitSize
	};

	m_callableRegion = {
		.deviceAddress = 0,
		.stride = 0,
		.size = 0
	};

	LOG_INFO("Shader binding table created");
}

void Raytracer::CreateVertexBufferFromMesh()
{
	const EigenMesh& geom = _cageMesh;
	const auto& positions = geom._vertices;
	const auto& faces = geom._faces;

	// Flattened vertex buffer
	std::vector<RTVertex> vertexData;
	vertexData.reserve(faces.rows() * 3);

	for (int tri = 0; tri < faces.rows(); ++tri)
	{
		for (int v = 0; v < 3; ++v)
		{
			int idx = faces(tri, v);
			vertexData.push_back({
				glm::vec3(
					static_cast<float>(positions(idx, 0)),
					static_cast<float>(positions(idx, 1)),
					static_cast<float>(positions(idx, 2))
				)
				});
		}
	}

	VkDeviceSize bufferSize = sizeof(RTVertex) * vertexData.size();

	// --- Step 1: Create staging buffer (CPU visible) ---
	MemoryMappedBuffer stagingBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>(reinterpret_cast<std::byte*>(vertexData.data()), bufferSize),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	// Copy data into staging buffer
	memcpy(stagingBuffer._mappedData, vertexData.data(), static_cast<size_t>(bufferSize));

	bufferSize = sizeof(RTVertex) * vertexData.size();
	std::span<std::byte> sizeSpan(
		static_cast<std::byte*>(nullptr),
		bufferSize
	);

	// --- Step 2: Create device-local vertex buffer (GPU only) ---
	_vertexBuffer = _resourceManager->AllocateDeviceBuffer(
		bufferSize,
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// --- Step 3: Copy from staging - device-local buffer ---
	{
		ScopedCmdBuffer cmd(_device, _commandPool);
		VkBufferCopy copyRegion{};
		copyRegion.srcOffset = 0;
		copyRegion.dstOffset = 0;
		copyRegion.size = bufferSize;
		vkCmdCopyBuffer(cmd.Get(), stagingBuffer._deviceBuffer, _vertexBuffer._deviceBuffer, 1, &copyRegion);

		cmd.SubmitAndWait(_queue);
	}

	// --- Step 4: Release staging buffer ---
	stagingBuffer.ReleaseResource(_device);
}

void Raytracer::CreateIndexBufferFromMesh()
{
	const EigenMesh& geom = _cageMesh;
	const auto& faces = geom._faces;

	const uint32_t indexCount = static_cast<uint32_t>(faces.rows()) * 3;
	std::vector<uint32_t> indices(indexCount);

	for (uint32_t tri = 0; tri < faces.rows(); ++tri)
		for (uint32_t v = 0; v < 3; ++v)
			indices[tri * 3 + v] = faces(tri, v);

	VkDeviceSize bufferSize = sizeof(uint32_t) * indexCount;

	// --- Step 1: Create staging buffer (CPU visible) ---
	MemoryMappedBuffer stagingBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>(reinterpret_cast<std::byte*>(indices.data()), bufferSize),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	memcpy(stagingBuffer._mappedData, indices.data(), static_cast<size_t>(bufferSize));
	std::span<std::byte> testSpan(
		reinterpret_cast<std::byte*>(indices.data()),
		static_cast<size_t>(bufferSize)
	);

	bufferSize = sizeof(RTVertex) * indices.size();
	std::span<std::byte> sizeSpan(
		reinterpret_cast<std::byte*>(indices.data()), // pointer to first element
		sizeof(uint32_t) * indices.size()            // total byte size
	);
	// --- Step 2: Create device-local index buffer (GPU only) ---
	_indexBuffer = _resourceManager->AllocateDeviceBuffer(
		sizeof(uint32_t) * indices.size(),  // empty, copy from staging
		VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// --- Step 3: Copy from staging device-local buffer ---
	{
		ScopedCmdBuffer cmd(_device, _commandPool);
		VkBufferCopy copyRegion{};
		copyRegion.srcOffset = 0;
		copyRegion.dstOffset = 0;
		copyRegion.size = bufferSize;
		vkCmdCopyBuffer(cmd.Get(), stagingBuffer._deviceBuffer, _indexBuffer._deviceBuffer, 1, &copyRegion);

		cmd.SubmitAndWait(_queue);
	}

	// --- Step 4: Release staging buffer ---
	stagingBuffer.ReleaseResource(_device);
}

void Raytracer::CreateRaytracingPipeline()
{
	// Cleanup (safe re-creation)
	if (m_rtPipeline != VK_NULL_HANDLE)
		vkDestroyPipeline(_device, m_rtPipeline, nullptr);

	if (m_rtPipelineLayout != VK_NULL_HANDLE)
		vkDestroyPipelineLayout(_device, m_rtPipelineLayout, nullptr);

	// --------------------------------------------------------------------
	// Shader stage indices
	enum StageIndices
	{
		eRaygen = 0,
		eMiss = 1,
		eAnyHit = 2,
		eStageCount
	};

	std::array<VkPipelineShaderStageCreateInfo, eStageCount> stages{};
	for (auto& s : stages)
		s.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;

	// NOTE:
	// We do NOT attach shader modules yet.
	// This function only prepares pipeline structure.

	// --------------------------------------------------------------------
	// Shader groups
	std::vector<VkRayTracingShaderGroupCreateInfoKHR> shaderGroups;

	// Raygen group
	{
		VkRayTracingShaderGroupCreateInfoKHR group{
			VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR
		};
		group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
		group.generalShader = eRaygen;
		group.closestHitShader = VK_SHADER_UNUSED_KHR;
		group.anyHitShader = VK_SHADER_UNUSED_KHR;
		group.intersectionShader = VK_SHADER_UNUSED_KHR;
		shaderGroups.push_back(group);
	}

	// Miss group
	{
		VkRayTracingShaderGroupCreateInfoKHR group{
			VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR
		};
		group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
		group.generalShader = eMiss;
		group.closestHitShader = VK_SHADER_UNUSED_KHR;
		group.anyHitShader = VK_SHADER_UNUSED_KHR;
		group.intersectionShader = VK_SHADER_UNUSED_KHR;
		shaderGroups.push_back(group);
	}

	// Any-hit group (triangle geometry)
	{
		VkRayTracingShaderGroupCreateInfoKHR group{
			VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR
		};
		group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
		group.generalShader = VK_SHADER_UNUSED_KHR;
		group.closestHitShader = VK_SHADER_UNUSED_KHR; // IMPORTANT
		group.anyHitShader = eAnyHit;
		group.intersectionShader = VK_SHADER_UNUSED_KHR;
		shaderGroups.push_back(group);
	}

	// --------------------------------------------------------------------
	// Pipeline layout (ONLY RT descriptor set)
	VkPipelineLayoutCreateInfo layoutInfo{
		VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO
	};
	layoutInfo.setLayoutCount = 1;
	layoutInfo.pSetLayouts = &_rtDescSetLayout;
	//layoutInfo.pushConstantRangeCount = 1;
	//layoutInfo.pPushConstantRanges = &pushConstant;

	vkCreatePipelineLayout(
		_device,
		&layoutInfo,
		nullptr,
		&m_rtPipelineLayout
	);

	// --------------------------------------------------------------------
	// Ray tracing pipeline create info (NO shaders yet)
	VkRayTracingPipelineCreateInfoKHR pipelineInfo{
		VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR
	};
	pipelineInfo.stageCount = 0;               // filled later
	pipelineInfo.pStages = nullptr;
	pipelineInfo.groupCount = uint32_t(shaderGroups.size());
	pipelineInfo.pGroups = shaderGroups.data();
	pipelineInfo.maxPipelineRayRecursionDepth = 1;
	pipelineInfo.layout = m_rtPipelineLayout;

	// Pipeline will actually be created in Phase 5
	// once shader modules are available
	CreateShaderBindingTable();
	LOG_DEBUG("Ray tracing pipeline structure prepared (MVC use case)");
}

MeshOperationResult<MeshComputeWeightsOperationResult> Raytracer::ComputeCoordinates() {


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
		std::move(psiQuad)};
}

uint32_t Raytracer::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const
{
	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(_device->GetPhysicalDeviceHandle(), &memProperties);

	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
		if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
			return i;
	}
	throw std::runtime_error("Failed to find suitable memory type!");
}


std::vector<uint32_t> Raytracer::LoadSPIRV(const std::string& filename) {
	std::filesystem::path fullPath = _shaderDir / filename;

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

void Raytracer::LoadShaders() {
	try {
		// Load pre-compiled SPIR-V shaders
		auto raygenCode = LoadSPIRV("raygen.rgen.spv");
		auto missCode = LoadSPIRV("miss.rmiss.spv");
		auto hitCode = LoadSPIRV("hit.rchit.spv");

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

void Raytracer::CleanupShaders() {
	if (_raygenShader != VK_NULL_HANDLE) {
		vkDestroyShaderModule(_device, _raygenShader, nullptr);
		_raygenShader = VK_NULL_HANDLE;
	}
	if (_missShader != VK_NULL_HANDLE) {
		vkDestroyShaderModule(_device, _missShader, nullptr);
		_missShader = VK_NULL_HANDLE;
	}
	if (_hitShader != VK_NULL_HANDLE) {
		vkDestroyShaderModule(_device, _hitShader, nullptr);
		_hitShader = VK_NULL_HANDLE;
	}
}



void Raytracer::CreateRayTracingBuffers() {
	// Create buffer for deformable mesh vertices
	const auto& deformableVerts = _deformableMesh._vertices;
	size_t vertexBufferSize = deformableVerts.rows() * 3 * sizeof(float);

	std::vector<float> vertexData;
	vertexData.reserve(deformableVerts.rows() * 3);
	for (int i = 0; i < deformableVerts.rows(); ++i) {
		vertexData.push_back(static_cast<float>(deformableVerts(i, 0)));
		vertexData.push_back(static_cast<float>(deformableVerts(i, 1)));
		vertexData.push_back(static_cast<float>(deformableVerts(i, 2)));
	}

	auto staging = _resourceManager->CreateBufferAndCopy(
		std::span(reinterpret_cast<const std::byte*>(vertexData.data()), vertexBufferSize),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_deformableVertexBuffer = _resourceManager->AllocateDeviceBuffer(
		vertexBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(staging, _deformableVertexBuffer, vertexBufferSize);
	staging.ReleaseResource(_device);

	// Create buffer for ray directions
	SetupRayDirections();

	// Create cage index buffer
	const auto& faces = _cageMesh._faces;
	size_t indexBufferSize = faces.rows() * 3 * sizeof(uint32_t);

	std::vector<uint32_t> indexData;
	indexData.reserve(faces.rows() * 3);
	for (int i = 0; i < faces.rows(); ++i) {
		indexData.push_back(static_cast<uint32_t>(faces(i, 0)));
		indexData.push_back(static_cast<uint32_t>(faces(i, 1)));
		indexData.push_back(static_cast<uint32_t>(faces(i, 2)));
	}

	staging = _resourceManager->CreateBufferAndCopy(
		std::span(reinterpret_cast<const std::byte*>(indexData.data()), indexBufferSize),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_cageIndexBuffer = _resourceManager->AllocateDeviceBuffer(
		indexBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(staging, _cageIndexBuffer, indexBufferSize);
	staging.ReleaseResource(_device);

	// Create hit buffer
	uint32_t maxTotalHits = _pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;
	size_t hitBufferSize = maxTotalHits * sizeof(GLSLHitRecord);

	_hitBuffer = _resourceManager->AllocateDeviceBuffer(
		hitBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// Create MVC weights buffer
	size_t weightsBufferSize = _pushConstants.vertexCount *
		_cageMesh._vertices.rows() *
		sizeof(float);

	_mvcWeightsBuffer = _resourceManager->AllocateDeviceBuffer(
		weightsBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);
}

void Raytracer::SetupRayDirections() {
	// Generate random directions on sphere for Monte Carlo integration
	uint32_t totalRays = _pushConstants.vertexCount * _pushConstants.raysPerVertex;
	std::vector<glm::vec4> directions(totalRays);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<float> dist(0.0f, 1.0f);

	for (uint32_t i = 0; i < totalRays; ++i) {
		// Generate random point on unit sphere
		float theta = 2.0f * glm::pi<float>() * dist(gen);
		float phi = acos(1.0f - 2.0f * dist(gen));

		directions[i].x = sin(phi) * cos(theta);
		directions[i].y = sin(phi) * sin(theta);
		directions[i].z = cos(phi);
		directions[i].w = static_cast<float>(i / _pushConstants.raysPerVertex);
	}

	size_t directionsBufferSize = totalRays * sizeof(glm::vec4);

	auto staging = _resourceManager->CreateBufferAndCopy(
		std::span(reinterpret_cast<const std::byte*>(directions.data()), directionsBufferSize),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_rayDirectionsBuffer = _resourceManager->AllocateDeviceBuffer(
		directionsBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(staging, _rayDirectionsBuffer, directionsBufferSize);
	staging.ReleaseResource(_device);

	uint32_t maxTotalHits = _pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;

	// Ensure proper alignment for GLSL structures
	const size_t hitRecordSize = 64; // Must match GLSLHitRecord size
	VkDeviceSize hitBufferSize = maxTotalHits * hitRecordSize;

	// Use scalar block layout for buffer
	_hitBuffer = _resourceManager->AllocateDeviceBuffer(
		hitBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// Create staging buffer to clear hit buffer
	auto staging = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>(),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		hitBufferSize
	);

	// Clear hit buffer to zeros
	memset(staging._mappedData, 0, static_cast<size_t>(hitBufferSize));

	// Copy from staging to device
	{
		ScopedCmdBuffer cmd(_device, _commandPool);
		VkBufferCopy copyRegion{
			.srcOffset = 0,
			.dstOffset = 0,
			.size = hitBufferSize
		};
		vkCmdCopyBuffer(cmd.Get(), staging._deviceBuffer, _hitBuffer._deviceBuffer, 1, &copyRegion);
		cmd.SubmitAndWait(_queue);
	}

	staging.ReleaseResource(_device);
}

void Raytracer::CreateRayTracingDescriptorSet() {
	// Create descriptor set layout with all necessary bindings
	std::vector<VkDescriptorSetLayoutBinding> bindings = {
		// Binding 0: Acceleration structure
		{
			.binding = 0,
			.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR
		},
		// Binding 1: Output image (for debugging)
		{
			.binding = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR
		},
		// Binding 2: Deformable vertices
		{
			.binding = 2,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR
		},
		// Binding 3: Ray directions
		{
			.binding = 3,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR
		},
		// Binding 4: Hit buffer
		{
			.binding = 4,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR
		},
		// Binding 5: Cage indices
		{
			.binding = 5,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR
		}
	};

	VkDescriptorSetLayoutCreateInfo layoutInfo{
		.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
		.bindingCount = static_cast<uint32_t>(bindings.size()),
		.pBindings = bindings.data()
	};

	vkCreateDescriptorSetLayout(_device, &layoutInfo, nullptr, &_rtDescSetLayout);

	// Create descriptor pool and allocate set
	// ... (implementation depends on your descriptor pool management)
}

void Raytracer::CreateRaytracingPipeline() {
	LoadShaders();

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
		.pSetLayouts = &_rtDescSetLayout,
		.pushConstantRangeCount = 1,
		.pPushConstantRanges = &pushConstantRange
	};

	vkCreatePipelineLayout(_device, &pipelineLayoutInfo, nullptr, &m_rtPipelineLayout);

	// Create ray tracing pipeline
	VkRayTracingPipelineCreateInfoKHR pipelineInfo{
		.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR,
		.stageCount = static_cast<uint32_t>(stages.size()),
		.pStages = stages.data(),
		.groupCount = static_cast<uint32_t>(groups.size()),
		.pGroups = groups.data(),
		.maxPipelineRayRecursionDepth = _pushConstants.maxHitsPerRay + 1,
		.layout = m_rtPipelineLayout
	};

	VK_CHECK(vkCreateRayTracingPipelinesKHR(
		_device,
		VK_NULL_HANDLE,
		VK_NULL_HANDLE,
		1,
		&pipelineInfo,
		nullptr,
		&m_rtPipeline
	));

	CreateShaderBindingTable();
}

void Raytracer::TraceRays() {
	// Update push constants
	_pushConstants.vertexCount = static_cast<uint32_t>(_deformableMesh._vertices.rows());

	// Begin command buffer
	VkCommandBufferAllocateInfo allocInfo{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		.commandPool = _commandPool,
		.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
		.commandBufferCount = 1
	};

	VkCommandBuffer cmd;
	vkAllocateCommandBuffers(_device, &allocInfo, &cmd);

	VkCommandBufferBeginInfo beginInfo{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
		.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
	};
	vkBeginCommandBuffer(cmd, &beginInfo);

	// Bind pipeline and descriptor sets
	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, m_rtPipeline);
	vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
		m_rtPipelineLayout, 0, 1, &m_rtDescriptorSet, 0, nullptr);

	// Push constants
	vkCmdPushConstants(cmd, m_rtPipelineLayout,
		VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR,
		0, sizeof(PushConstants), &_pushConstants);

	// Trace rays
	vkCmdTraceRaysKHR(
		cmd,
		&m_raygenRegion,
		&m_missRegion,
		&m_hitRegion,
		&m_callableRegion,
		_pushConstants.vertexCount * _pushConstants.raysPerVertex, // width
		1, // height
		1  // depth
	);

	vkEndCommandBuffer(cmd);

	// Submit and wait
	VkSubmitInfo submitInfo{
		.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
		.commandBufferCount = 1,
		.pCommandBuffers = &cmd
	};

	vkQueueSubmit(_queue, 1, &submitInfo, VK_NULL_HANDLE);
	vkQueueWaitIdle(_queue);

	vkFreeCommandBuffers(_device, _commandPool, 1, &cmd);

	// Process hits for MVC
	ProcessHitsForMVC();
}

/*void Raytracer::ProcessHitsForMVC() {
	// This would map the hit buffer and compute MVC weights
	// For simplicity, showing the CPU-side approach

	// Map hit buffer to CPU
	// auto* hits = MapBuffer<HitRecord>(_hitBuffer);

	// Compute MVC using the formula:
	// w_i = ?_? (tan(?_{i-1}/2) + tan(?_i/2)) / ||v - p_i||
	// where v is vertex position, p_i are cage vertices

	// This is a simplified version - you'd need to implement the full MVC algorithm
	// based on the recorded hit distances and directions
}*/

MeshOperationResult<MeshComputeWeightsOperationResult> Raytracer::ComputeCoordinates() {
	// First trace rays to get hit information
	TraceRays();

	// Then compute MVC weights from hit data
	// (This would be implemented based on your specific MVC algorithm)

	Eigen::MatrixXd weights(_deformableMesh._vertices.rows(),
		_cageMesh._vertices.rows());
	weights.setZero();

	// Placeholder: weights would be computed from ray hit data
	// ...

	return MeshComputeWeightsOperationResult{
		std::move(weights), // M matrix
		Eigen::MatrixXd(),  // weights
		Eigen::MatrixXd(),  // interpolatedWeights
		Eigen::MatrixXd(),  // psi
		std::vector<double>(), // psiTri
		std::vector<Eigen::Vector4d>() // psiQuad
	};
}

std::vector<GLSLHitRecord> Raytracer::ReadHitData() {
	uint32_t maxTotalHits = _pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;
	size_t hitBufferSize = maxTotalHits * sizeof(GLSLHitRecord);

	// Create staging buffer to read back results
	auto staging = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>(),
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		hitBufferSize
	);

	// Copy from device to staging
	{
		ScopedCmdBuffer cmd(_device, _commandPool);
		VkBufferCopy copyRegion{
			.srcOffset = 0,
			.dstOffset = 0,
			.size = hitBufferSize
		};
		vkCmdCopyBuffer(cmd.Get(), _hitBuffer._deviceBuffer, staging._deviceBuffer, 1, &copyRegion);
		cmd.SubmitAndWait(_queue);
	}

	// Read data
	std::vector<GLSLHitRecord> hits(maxTotalHits);
	memcpy(hits.data(), staging._mappedData, hitBufferSize);

	staging.ReleaseResource(_device);

	return hits;
}

void Raytracer::ProcessHitsForMVC() {
	auto hits = ReadHitData();

	// Process hits to compute MVC weights
	uint32_t vertexCount = _pushConstants.vertexCount;
	uint32_t cageVertexCount = static_cast<uint32_t>(_cageMesh._vertices.rows());

	// Initialize weights matrix
	Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(vertexCount, cageVertexCount);

	// For each ray, accumulate contributions to cage vertices
	for (const auto& hit : hits) {
		if (hit.hitSequence == 0) continue; // Skip if no valid hit

		uint32_t vertexIndex = hit.rayIndex / _pushConstants.raysPerVertex;

		// Get barycentric coordinates
		double u = hit.barycentric.x;
		double v = hit.barycentric.y;
		double w = 1.0 - u - v;

		// Distribute weight to cage vertices based on barycentric coordinates
		// This is a simplified MVC calculation
		double distance = hit.distance;
		double weight = 1.0 / (distance * distance); // Inverse square distance

		weights(vertexIndex, hit.vertexIndices[0]) += weight * u;
		weights(vertexIndex, hit.vertexIndices[1]) += weight * v;
		weights(vertexIndex, hit.vertexIndices[2]) += weight * w;
	}

	// Normalize weights for each vertex (sum to 1)
	for (int i = 0; i < vertexCount; ++i) {
		double sum = weights.row(i).sum();
		if (sum > 0) {
			weights.row(i) /= sum;
		}
	}

	// Store weights for later use
	_mvcWeights = std::move(weights);
}