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
	const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, uint32_t cubemapSize, VkFormat format) :
	_pipelineManager(pipelineManager),
	_resourceManager(resourceManager),
	_device(device)
{
}

Raytracer::~Raytracer()
{
}

void Raytracer::Initialize()
{
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



/*#include <Rendering/PMVC/Raytracer.h>

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

Raytracer::Raytracer(const std::shared_ptr<RenderPipelineManager>& pipelineManager,
	const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, uint32_t queueFamilyIndex, uint32_t cubemapSize, VkFormat format) : 
	_pipelineManager(pipelineManager),
	_resourceManager(resourceManager), 
	_queueFamilyIndex(queueFamilyIndex),
	_device(device)
{
}

Raytracer::~Raytracer()
{
	CleanupShaders();

	// Clean up descriptor resources
	if (_rtPipeline.descriptorPool != VK_NULL_HANDLE) {
		vkDestroyDescriptorPool(_device, _rtPipeline.descriptorPool, nullptr);
	}
	if (_rtPipeline.descriptorLayout != VK_NULL_HANDLE) {
		vkDestroyDescriptorSetLayout(_device, _rtPipeline.descriptorLayout, nullptr);
	}

	// Clean up output image
	if (_output.view != VK_NULL_HANDLE) {
		vkDestroyImageView(_device, _output.view, nullptr);
	}
	if (_output.image != VK_NULL_HANDLE) {
		vkDestroyImage(_device, _output.image, nullptr);
	}
	if (_output.memory != VK_NULL_HANDLE) {
		vkFreeMemory(_device, _output.memory, nullptr);
	}

	// Clean up other resources...
	if (_rtPipeline.pipeline != VK_NULL_HANDLE) {
		vkDestroyPipeline(_device, _rtPipeline.pipeline, nullptr);
	}
	if (_rtPipeline.layout != VK_NULL_HANDLE) {
		vkDestroyPipelineLayout(_device, _rtPipeline.layout, nullptr);
	}

	// Clean up acceleration structures
	if (_blas.handle != VK_NULL_HANDLE) {
		vkDestroyAccelerationStructureKHR(_device, _blas.handle, nullptr);
	}
	_blas.buffer.ReleaseResource(_device);

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
	// Clean up shader modules if they exist
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
	LOG_INFO("Initializing Raytracer...");

	CreateCommandPool();

	vkGetDeviceQueue(
		_device,
		_queueFamilyIndex,
		0,
		&_queue);

	CreateGeometryBuffers();
	CreateAccelerationStructures();
	CreateRayTracingPipeline();
	CreateShaderBindingTable();
	CreateRayBuffers();
	CreateOutputImage();
	UpdateDescriptorSet();
}

void Raytracer::CreateGeometryBuffers()
{
	const auto& positions = _cageMesh._vertices;
	const auto& faces = _cageMesh._faces;

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

	_cageGeometry.vertexBuffer = _resourceManager->AllocateDeviceBuffer(
		vertexSize,
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	_cageGeometry.indexBuffer = _resourceManager->AllocateDeviceBuffer(
		indexSize,
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	CopyBuffer(stagingVertices, _cageGeometry.vertexBuffer, vertexSize);
	CopyBuffer(stagingIndices, _cageGeometry.indexBuffer, indexSize);

	stagingVertices.ReleaseResource(_device);
	stagingIndices.ReleaseResource(_device);

	_cageGeometry.vertexCount = static_cast<uint32_t>(vertices.size());
	_cageGeometry.indexCount = static_cast<uint32_t>(indices.size());
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

void Raytracer::CreateCommandPool() {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = _queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_commandPool) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create command pool!");
	}
}

void Raytracer::CopyBuffer(const MemoryMappedBuffer& src, Buffer& dst, VkDeviceSize size)
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
	const std::string shaderDir = "viewer/app/assets/shaders/";
	std::filesystem::path fullPath = shaderDir + filename;

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
		&_rtPipeline.descriptorLayout
	);

	if (res != VK_SUCCESS)
	{
		throw std::runtime_error("Failed to create ray tracing descriptor set layout");
	}
}

void Raytracer::CreateRayBuffers() {
	// Initialize push constants
	_pushConstants.vertexCount = static_cast<uint32_t>(_deformableMesh._vertices.rows());  // Note: Check if it's _vertices or vertices
	_pushConstants.raysPerVertex = 64;
	_pushConstants.maxHitsPerRay = 8;
	// This line below seems wrong - you're overwriting vertexCount with cage mesh vertex count
	// _pushConstants.vertexCount = static_cast<uint32_t>(_cageMesh.vertices.rows());

	LOG_INFO("Creating ray tracing buffers:");
	LOG_INFO("  Deformable vertices: {}", _pushConstants.vertexCount);
	LOG_INFO("  Rays per vertex: {}", _pushConstants.raysPerVertex);
	LOG_INFO("  Max hits per ray: {}", _pushConstants.maxHitsPerRay);
	LOG_INFO("  Cage vertices: {}", static_cast<uint32_t>(_cageMesh._vertices.rows()));

	// Validate inputs
	if (_deformableMesh._vertices.rows() == 0) {
		throw std::runtime_error("Deformable mesh has no vertices");
	}
	if (_cageMesh._vertices.rows() == 0) {
		throw std::runtime_error("Cage mesh has no vertices");
	}

	// Create ray directions buffer
	size_t rayDirectionsSize = _deformableMesh._vertices.rows() *
		_pushConstants.raysPerVertex *
		3 * sizeof(float);

	_rayBuffers.rayDirections = _resourceManager->AllocateDeviceBuffer(
		rayDirectionsSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	if (_rayBuffers.rayDirections._deviceBuffer == VK_NULL_HANDLE) {
		throw std::runtime_error("Failed to create ray directions buffer");
	}

	// Create hit buffer (GPU write -> CPU read)
	size_t hitBufferSize = _deformableMesh._vertices.rows() *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay *
		sizeof(uint32_t) * 2;  // Assuming hit data includes index and distance

	_rayBuffers.hitBuffer = _resourceManager->AllocateDeviceBuffer(
		hitBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	if (_rayBuffers.hitBuffer._deviceBuffer == VK_NULL_HANDLE) {
		throw std::runtime_error("Failed to create hit buffer");
	}

	// Create MVC weights buffer (GPU write -> CPU read)
	size_t weightsBufferSize = _deformableMesh._vertices.rows() *
		_cageMesh._vertices.rows() *
		sizeof(float);

	_rayBuffers.mvcWeights = _resourceManager->AllocateDeviceBuffer(
		weightsBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	if (_rayBuffers.mvcWeights._deviceBuffer == VK_NULL_HANDLE) {
		throw std::runtime_error("Failed to create MVC weights buffer");
	}

	// Zero-initialize the weights buffer (optional - can be done on GPU)

	LOG_INFO("Successfully created ray tracing buffers");
}


void Raytracer::SetupRayDirections() {
	// Validate push constants
	if (_pushConstants.vertexCount == 0 || _pushConstants.raysPerVertex == 0) {
		throw std::runtime_error("Push constants not properly initialized in SetupRayDirections");
	}

	// Generate random directions
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
	//CopyBuffer(stagingDirections, _rayBuffers.rayDirections, directionsBufferSize);TODO 
	stagingDirections.ReleaseResource(_device);

	LOG_INFO("  Ray directions buffer created: {}",
		(void*)_rayBuffers.rayDirections._deviceBuffer);

	// Create hit buffer
	uint32_t maxTotalHits = _pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;

	LOG_INFO("  Max total hits: {}", maxTotalHits);

	if (maxTotalHits == 0) {
		throw std::runtime_error("Max total hits calculation resulted in zero");
	}

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

	if (_rayBuffers.hitBuffer._deviceBuffer == VK_NULL_HANDLE) {
		throw std::runtime_error("Failed to create hit buffer");
	}

	// Copy from staging to device
	CopyBuffer(stagingHitBuffer, _rayBuffers.hitBuffer, hitBufferSize);
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
		// Binding 4: Hit buffer (written by hit shader)
		{
			.binding = 4,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR
		},
		// Binding 5: Cage indices
		{
			.binding = 5,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR
		},
		// Binding 6: MVC weights buffer (output)
		{
			.binding = 6,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR  // Written by raygen shader
		},
		// Binding 7: Atomic counter for hit buffer (NEW)
		{
			.binding = 7,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR
		}
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
			.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
			.descriptorCount = 1
		},
		{
			.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 6  // deformable vertices, ray directions, hit buffer, cage indices, MVC weights, atomic counter
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

	// 4. UPDATE DESCRIPTOR SET
	std::vector<VkWriteDescriptorSet> descriptorWrites;
	std::vector<VkDescriptorImageInfo> imageInfos;
	std::vector<VkDescriptorBufferInfo> bufferInfos;

	// Reserve space for all writes
	descriptorWrites.reserve(8);
	bufferInfos.reserve(7);  // 6 buffers + acceleration structure

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

	// Binding 1: Output image (if created)
	if (_output.view != VK_NULL_HANDLE) {
		VkDescriptorImageInfo imageInfo{
			.imageView = _output.view,
			.imageLayout = VK_IMAGE_LAYOUT_GENERAL
		};
		imageInfos.push_back(imageInfo);

		VkWriteDescriptorSet imageWrite{
			.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
			.dstSet = _rtPipeline.descriptorSet,
			.dstBinding = 1,
			.dstArrayElement = 0,
			.descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
			.pImageInfo = &imageInfos.back()
		};
		descriptorWrites.push_back(imageWrite);
	}
	else {
		LOG_WARN("Output image not created - binding 1 will be empty");
	}

	// Binding 2: Deformable vertices
	VkDeviceSize vertexBufferSize = _pushConstants.vertexCount * 3 * sizeof(float);
	addBufferWrite(2, _deformableGeometry.vertexBuffer, vertexBufferSize,
		VK_SHADER_STAGE_RAYGEN_BIT_KHR);

	// Binding 3: Ray directions
	uint32_t totalRays = _pushConstants.vertexCount * _pushConstants.raysPerVertex;
	VkDeviceSize directionsSize = static_cast<VkDeviceSize>(totalRays) * sizeof(glm::vec4);
	addBufferWrite(3, _rayBuffers.rayDirections, directionsSize,
		VK_SHADER_STAGE_RAYGEN_BIT_KHR);

	// Binding 4: Hit buffer
	uint32_t maxTotalHits = _pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;
	VkDeviceSize hitBufferSize = static_cast<VkDeviceSize>(maxTotalHits) * sizeof(SimpleHit);
	addBufferWrite(4, _rayBuffers.hitBuffer, hitBufferSize,
		VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR);

	// Binding 5: Cage indices
	VkDeviceSize indexBufferSize = static_cast<VkDeviceSize>(_cageMesh._faces.rows() * 3 * sizeof(uint32_t));
	addBufferWrite(5, _cageGeometry.indexBuffer, indexBufferSize,
		VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR);

	// Binding 6: MVC weights buffer
	VkDeviceSize weightsBufferSize = static_cast<VkDeviceSize>(_pushConstants.vertexCount *
		_pushConstants.vertexCount *
		sizeof(float));
	addBufferWrite(6, _rayBuffers.mvcWeights, weightsBufferSize,
		VK_SHADER_STAGE_RAYGEN_BIT_KHR);

	// Binding 7: Atomic counter buffer (create if not exists)
	if (_rayBuffers.atomicCounterBuffer._deviceBuffer == VK_NULL_HANDLE) {
		// Create atomic counter buffer
		VkDeviceSize atomicBufferSize = sizeof(uint32_t);  // Single counter
		_rayBuffers.atomicCounterBuffer = _resourceManager->AllocateDeviceBuffer(
			atomicBufferSize,
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
		);

		// Initialize to zero
		uint32_t zero = 0;
		auto stagingAtomic = _resourceManager->CreateBufferAndCopy(
			std::span(reinterpret_cast<const std::byte*>(&zero), sizeof(zero)),
			VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		);

		CopyBuffer(stagingAtomic, _rayBuffers.atomicCounterBuffer, atomicBufferSize);
		stagingAtomic.ReleaseResource(_device);
	}

	addBufferWrite(7, _rayBuffers.atomicCounterBuffer, sizeof(uint32_t),
		VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR | VK_SHADER_STAGE_ANY_HIT_BIT_KHR);

	// Update all descriptors at once
	vkUpdateDescriptorSets(_device,
		static_cast<uint32_t>(descriptorWrites.size()),
		descriptorWrites.data(),
		0, nullptr);

	LOG_INFO("Updated descriptor set with {} writes", descriptorWrites.size());
	LOG_INFO("Successfully created ray tracing descriptor set");
}
*/
