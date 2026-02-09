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

//static_assert(sizeof(Raytracer::GLSLHitRecord) == 64, "GLSLHitRecord must be tightly packed");
//static_assert(sizeof(Raytracer::PushConstants) % 16 == 0, "PushConstants must be 16-byte aligned");

Raytracer::Raytracer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
	const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance, uint32_t cubemapSize, VkFormat format) : _renderPipelineManager(renderPipelineManager),
	_resourceManager(resourceManager), _device(device), _instance(instance)
{
}

Raytracer::~Raytracer()
{
	CleanupShaders();

	// Clean up descriptor resources
	if (_rtDescPool != VK_NULL_HANDLE) {
		vkDestroyDescriptorPool(_device, _rtDescPool, nullptr);
	}
	if (_rtDescSetLayout != VK_NULL_HANDLE) {
		vkDestroyDescriptorSetLayout(_device, _rtDescSetLayout, nullptr);
	}

	// Clean up output image
	if (_outputImage._imageView != VK_NULL_HANDLE) {
		vkDestroyImageView(_device, _outputImage._imageView, nullptr);
	}
	if (_outputImage._image != VK_NULL_HANDLE) {
		vkDestroyImage(_device, _outputImage._image, nullptr);
	}
	if (_outputImage._memory != VK_NULL_HANDLE) {
		vkFreeMemory(_device, _outputImage._memory, nullptr);
	}

	// Clean up other resources...
	if (m_rtPipeline != VK_NULL_HANDLE) {
		vkDestroyPipeline(_device, m_rtPipeline, nullptr);
	}
	if (m_rtPipelineLayout != VK_NULL_HANDLE) {
		vkDestroyPipelineLayout(_device, m_rtPipelineLayout, nullptr);
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
		std::move(psiQuad) };
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

void Raytracer::Initialize()
{
	// Add mesh validation
	LOG_INFO("Cage mesh vertices: {}, faces: {}",
		_cageMesh._vertices.rows(),
		_cageMesh._faces.rows());
	LOG_INFO("Deformable mesh vertices: {}, faces: {}",
		_deformableMesh._vertices.rows(),
		_deformableMesh._faces.rows());

	if (_cageMesh._vertices.rows() == 0 || _cageMesh._faces.rows() == 0) {
		throw std::runtime_error("Cage mesh is empty");
	}
	if (_deformableMesh._vertices.rows() == 0) {
		throw std::runtime_error("Deformable mesh is empty");
	}

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
	CreateTopLevelAS();
	// Load shaders BEFORE creating pipeline
	LoadShaders();
	CreateRaytraceDescriptorLayout();

	// Create buffers for ray tracing
	CreateRayTracingBuffers();  // MISSING CALL
	CreateRayTracingDescriptorSet();  // MISSING CALL

	// This should be the version that actually loads shaders
	//CreateRaytracingPipeline();

	// Create SBT after pipeline
	// Note: CreateShaderBindingTable() should be called inside CreateRaytracingPipeline()
	// after pipeline creation
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

void Raytracer::GetRaytracingComponents()
{
	VkPhysicalDeviceProperties2 prop2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
	m_rtProperties.pNext = &m_asProperties;
	prop2.pNext = &m_rtProperties;
	vkGetPhysicalDeviceProperties2(_device->GetPhysicalDeviceHandle(), &prop2);

}

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

void Raytracer::CreateRayTracingBuffers() {
	// Initialize push constants with valid values first
	_pushConstants.vertexCount = static_cast<uint32_t>(_deformableMesh._vertices.rows());
	_pushConstants.raysPerVertex = 64; // Example value
	_pushConstants.maxHitsPerRay = 8;  // Example value
	_pushConstants.vertexCount = static_cast<uint32_t>(_cageMesh._vertices.rows());

	// DEBUG: Print values to verify
	LOG_INFO("Creating ray tracing buffers:");
	LOG_INFO("  Deformable vertices: {}", _pushConstants.vertexCount);
	LOG_INFO("  Rays per vertex: {}", _pushConstants.raysPerVertex);
	LOG_INFO("  Max hits per ray: {}", _pushConstants.maxHitsPerRay);
	LOG_INFO("  Cage vertices: {}", _pushConstants.vertexCount);

	// Validate inputs
	if (_pushConstants.vertexCount == 0) {
		throw std::runtime_error("Deformable mesh has no vertices");
	}
	if (_pushConstants.vertexCount == 0) {
		throw std::runtime_error("Cage mesh has no vertices");
	}

	// Create buffer for deformable mesh vertices
	const auto& deformableVerts = _deformableMesh._vertices;
	size_t vertexBufferSize = deformableVerts.rows() * 3 * sizeof(float);

	if (vertexBufferSize == 0) {
		throw std::runtime_error("Deformable vertex buffer size is zero");
	}

	std::vector<float> vertexData;
	vertexData.reserve(deformableVerts.rows() * 3);
	for (int i = 0; i < deformableVerts.rows(); ++i) {
		vertexData.push_back(static_cast<float>(deformableVerts(i, 0)));
		vertexData.push_back(static_cast<float>(deformableVerts(i, 1)));
		vertexData.push_back(static_cast<float>(deformableVerts(i, 2)));
	}

	auto stagingVertices = _resourceManager->CreateBufferAndCopy(
		std::span(reinterpret_cast<const std::byte*>(vertexData.data()), vertexBufferSize),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_deformableVertexBuffer = _resourceManager->AllocateDeviceBuffer(
		vertexBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(stagingVertices, _deformableVertexBuffer, vertexBufferSize);
	stagingVertices.ReleaseResource(_device);

	// Create buffer for ray directions
	SetupRayDirections();

	// Create cage index buffer
	const auto& faces = _cageMesh._faces;
	size_t indexBufferSize = faces.rows() * 3 * sizeof(uint32_t);

	if (indexBufferSize == 0) {
		throw std::runtime_error("Cage index buffer size is zero");
	}

	std::vector<uint32_t> indexData;
	indexData.reserve(faces.rows() * 3);
	for (int i = 0; i < faces.rows(); ++i) {
		indexData.push_back(static_cast<uint32_t>(faces(i, 0)));
		indexData.push_back(static_cast<uint32_t>(faces(i, 1)));
		indexData.push_back(static_cast<uint32_t>(faces(i, 2)));
	}

	auto stagingIndices = _resourceManager->CreateBufferAndCopy(
		std::span(reinterpret_cast<const std::byte*>(indexData.data()), indexBufferSize),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_cageIndexBuffer = _resourceManager->AllocateDeviceBuffer(
		indexBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(stagingIndices, _cageIndexBuffer, indexBufferSize);
	stagingIndices.ReleaseResource(_device);

	// Create hit buffer - moved to SetupRayDirections() to ensure proper calculation
	// Create MVC weights buffer
	size_t weightsBufferSize = _pushConstants.vertexCount *
		_pushConstants.vertexCount *
		sizeof(float);

	if (weightsBufferSize == 0) {
		throw std::runtime_error("Weights buffer size is zero");
	}

	_mvcWeightsBuffer = _resourceManager->AllocateDeviceBuffer(
		weightsBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	if (_deformableVertexBuffer._deviceBuffer == VK_NULL_HANDLE) {
		throw std::runtime_error("Failed to create deformable vertex buffer");
	}

	if (_rayDirectionsBuffer._deviceBuffer == VK_NULL_HANDLE) {
		throw std::runtime_error("Failed to create ray directions buffer");
	}
	LOG_INFO("Successfully created ray tracing buffers");
}

void Raytracer::SetupRayDirections() {
	// Validate push constants have been initialized
	if (_pushConstants.vertexCount == 0 || _pushConstants.raysPerVertex == 0) {
		throw std::runtime_error("Push constants not properly initialized in SetupRayDirections");
	}

	// Generate random directions on sphere for Monte Carlo integration
	uint32_t totalRays = _pushConstants.vertexCount * _pushConstants.raysPerVertex;

	LOG_INFO("Setting up ray directions:");
	LOG_INFO("  Total rays: {}", totalRays);

	if (totalRays == 0) {
		throw std::runtime_error("Total rays calculation resulted in zero");
	}

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

	if (directionsBufferSize == 0) {
		throw std::runtime_error("Directions buffer size is zero");
	}

	auto stagingDirections = _resourceManager->CreateBufferAndCopy(
		std::span(reinterpret_cast<const std::byte*>(directions.data()), directionsBufferSize),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_rayDirectionsBuffer = _resourceManager->AllocateDeviceBuffer(
		directionsBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(stagingDirections, _rayDirectionsBuffer, directionsBufferSize);
	stagingDirections.ReleaseResource(_device);

	// Create hit buffer
	uint32_t maxTotalHits = _pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;

	LOG_INFO("  Max total hits: {}", maxTotalHits);

	if (maxTotalHits == 0) {
		throw std::runtime_error("Max total hits calculation resulted in zero");
	}

	// Ensure proper alignment for GLSL structures
	const size_t hitRecordSize = 64; // Must match GLSLHitRecord size
	VkDeviceSize hitBufferSize = static_cast<VkDeviceSize>(maxTotalHits) * hitRecordSize;

	if (hitBufferSize == 0) {
		throw std::runtime_error("Hit buffer size is zero");
	}

	LOG_INFO("  Hit buffer size: {} bytes", hitBufferSize);

	// Use scalar block layout for buffer
	_hitBuffer = _resourceManager->AllocateDeviceBuffer(
		hitBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	// Initialize hit buffer with zeros
	std::vector<std::byte> zeroData(static_cast<size_t>(hitBufferSize), std::byte{ 0 });
	auto stagingHitBuffer = _resourceManager->CreateBufferAndCopy(
		std::span(zeroData),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	CopyBuffer(stagingHitBuffer, _hitBuffer, hitBufferSize);
	stagingHitBuffer.ReleaseResource(_device);

	LOG_INFO("Successfully set up ray directions");
}

void Raytracer::CreateRayTracingDescriptorSet() {
	LOG_INFO("Buffer status before descriptor creation:");
	LOG_INFO("  Deformable vertex buffer: {}",
		(void*)_deformableVertexBuffer._deviceBuffer);
	LOG_INFO("  Ray directions buffer: {}",
		(void*)_rayDirectionsBuffer._deviceBuffer);
	LOG_INFO("  Hit buffer: {}",
		(void*)_hitBuffer._deviceBuffer);
	LOG_INFO("  Cage index buffer: {}",
		(void*)_cageIndexBuffer._deviceBuffer);
	LOG_INFO("  MVC weights buffer: {}",
		(void*)_mvcWeightsBuffer._deviceBuffer);
	LOG_INFO("  BLAS: {}", (void*)m_blasAccel);
	LOG_INFO("  TLAS: {}", (void*)m_tlasAccel);
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
		},
		// Binding 6: MVC weights buffer (NEW - for storing computed weights)
		{
			.binding = 6,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT | VK_SHADER_STAGE_RAYGEN_BIT_KHR
		}
	};

	VkDescriptorSetLayoutCreateInfo layoutInfo{
		.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
		.bindingCount = static_cast<uint32_t>(bindings.size()),
		.pBindings = bindings.data()
	};

	if (_rtDescSetLayout != VK_NULL_HANDLE) {
		vkDestroyDescriptorSetLayout(_device, _rtDescSetLayout, nullptr);
		_rtDescSetLayout = VK_NULL_HANDLE;
	}

	VK_CHECK(vkCreateDescriptorSetLayout(_device, &layoutInfo, nullptr, &_rtDescSetLayout));

	// Create descriptor pool
	std::vector<VkDescriptorPoolSize> poolSizes = {
		{
			.type = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,
			.descriptorCount = 1
		},
		{
			.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
			.descriptorCount = 1
		},
		{
			.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 5  // deformable vertices, ray directions, hit buffer, cage indices, MVC weights
		}
	};

	VkDescriptorPoolCreateInfo poolInfo{
		.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
		.maxSets = 1,  // We only need one descriptor set for ray tracing
		.poolSizeCount = static_cast<uint32_t>(poolSizes.size()),
		.pPoolSizes = poolSizes.data()
	};

	if (_rtDescPool != VK_NULL_HANDLE) {
		vkDestroyDescriptorPool(_device, _rtDescPool, nullptr);
		_rtDescPool = VK_NULL_HANDLE;
	}

	VK_CHECK(vkCreateDescriptorPool(_device, &poolInfo, nullptr, &_rtDescPool));

	// Allocate descriptor set
	VkDescriptorSetAllocateInfo allocInfo{
		.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
		.descriptorPool = _rtDescPool,
		.descriptorSetCount = 1,
		.pSetLayouts = &_rtDescSetLayout
	};

	VK_CHECK(vkAllocateDescriptorSets(_device, &allocInfo, &_rtDescSet));

	// Write descriptor set updates
	std::vector<VkWriteDescriptorSet> descriptorWrites;
	descriptorWrites.reserve(7);

	// Binding 0: Acceleration structure
	VkWriteDescriptorSetAccelerationStructureKHR accelStructureWrite{
		.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR,
		.accelerationStructureCount = 1,
		.pAccelerationStructures = &m_tlasAccel
	};

	VkWriteDescriptorSet accelWrite{
		.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
		.pNext = &accelStructureWrite,
		.dstSet = _rtDescSet,
		.dstBinding = 0,
		.dstArrayElement = 0,
		.descriptorCount = 1,
		.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR
	};
	descriptorWrites.push_back(accelWrite);

	// Note: Binding 1 (output image) will be updated later when the image is created
	// For now, create a placeholder write
	if (_outputImage._imageView != VK_NULL_HANDLE) {
		VkDescriptorImageInfo imageInfo{
			.imageView = _outputImage._imageView,
			.imageLayout = VK_IMAGE_LAYOUT_GENERAL
		};

		descriptorWrites.push_back({
			.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
			.dstSet = _rtDescSet,
			.dstBinding = 1,
			.dstArrayElement = 0,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
			.pImageInfo = &imageInfo
			});
	}

	// Helper function to create buffer descriptor writes
	auto addBufferWrite = [&](uint32_t binding, const Buffer& buffer, VkDeviceSize size) {
		VkDescriptorBufferInfo bufferInfo{
			.buffer = buffer._deviceBuffer,
			.offset = 0,
			.range = size
		};

		descriptorWrites.push_back({
			.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
			.dstSet = _rtDescSet,
			.dstBinding = binding,
			.dstArrayElement = 0,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.pBufferInfo = &bufferInfo
			});
	};

	// Binding 2: Deformable vertices
	if (_deformableVertexBuffer._deviceBuffer != VK_NULL_HANDLE) {
		VkDeviceSize vertexBufferSize = _pushConstants.vertexCount * 3 * sizeof(float);
		addBufferWrite(2, _deformableVertexBuffer, vertexBufferSize);
	}

	// Binding 3: Ray directions
	if (_rayDirectionsBuffer._deviceBuffer != VK_NULL_HANDLE) {
		uint32_t totalRays = _pushConstants.vertexCount * _pushConstants.raysPerVertex;
		VkDeviceSize directionsSize = totalRays * sizeof(glm::vec4);
		addBufferWrite(3, _rayDirectionsBuffer, directionsSize);
	}

	// Binding 4: Hit buffer
	if (_hitBuffer._deviceBuffer != VK_NULL_HANDLE) {
		uint32_t maxTotalHits = _pushConstants.vertexCount *
			_pushConstants.raysPerVertex *
			_pushConstants.maxHitsPerRay;
		VkDeviceSize hitBufferSize = static_cast<VkDeviceSize>(maxTotalHits) * sizeof(GLSLHitRecord);
		addBufferWrite(4, _hitBuffer, hitBufferSize);
	}

	// Binding 5: Cage indices
	if (_cageIndexBuffer._deviceBuffer != VK_NULL_HANDLE) {
		VkDeviceSize indexBufferSize = _cageMesh._faces.rows() * 3 * sizeof(uint32_t);
		addBufferWrite(5, _cageIndexBuffer, indexBufferSize);
	}

	// Binding 6: MVC weights buffer
	if (_mvcWeightsBuffer._deviceBuffer != VK_NULL_HANDLE) {
		VkDeviceSize weightsBufferSize = _pushConstants.vertexCount *
			_pushConstants.vertexCount *
			sizeof(float);
		addBufferWrite(6, _mvcWeightsBuffer, weightsBufferSize);
	}

	// Update the descriptor set
	vkUpdateDescriptorSets(_device,
		static_cast<uint32_t>(descriptorWrites.size()),
		descriptorWrites.data(),
		0, nullptr);

	LOG_INFO("Created ray tracing descriptor set with {} bindings", descriptorWrites.size());
}

void Raytracer::CreateTopLevelAS() {
	// 1. Create instance for your BLAS
	VkAccelerationStructureInstanceKHR instance{};

	// Identity transform matrix
	instance.transform = {
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f
	};

	// Get BLAS device address
	VkAccelerationStructureDeviceAddressInfoKHR addressInfo{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR
	};
	addressInfo.accelerationStructure = m_blasAccel;
	instance.accelerationStructureReference =
		vkGetAccelerationStructureDeviceAddressKHR(_device, &addressInfo);

	instance.instanceCustomIndex = 0; // Used for shader indexing
	instance.mask = 0xFF; // All rays hit this
	instance.instanceShaderBindingTableRecordOffset = 0; // SBT offset
	instance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;

	// 2. Create buffer for instances
	MemoryMappedBuffer instanceBuffer = _resourceManager->CreateBufferAndCopy(
		std::span(reinterpret_cast<const std::byte*>(&instance), sizeof(instance)),
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	// 3. Build TLAS
	VkAccelerationStructureGeometryKHR geometry{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR
	};
	geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
	geometry.geometry.instances.sType =
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
	geometry.geometry.instances.arrayOfPointers = VK_FALSE;

	VkBufferDeviceAddressInfo bufferInfo{
		VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO
	};
	bufferInfo.buffer = instanceBuffer._deviceBuffer;
	geometry.geometry.instances.data.deviceAddress =
		vkGetBufferDeviceAddress(_device, &bufferInfo);

	VkAccelerationStructureBuildRangeInfoKHR rangeInfo{};
	rangeInfo.primitiveCount = 1; // One instance
	rangeInfo.primitiveOffset = 0;
	rangeInfo.firstVertex = 0;
	rangeInfo.transformOffset = 0;

	// 4. Call CreateAccelerationStructure with TLAS type
	CreateAccelerationStructure(
		VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR,
		m_tlasAccel,
		m_tlasMemory,
		geometry,
		rangeInfo,
		1, // 1 instance
		VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR
	);

	instanceBuffer.ReleaseResource(_device);
}

