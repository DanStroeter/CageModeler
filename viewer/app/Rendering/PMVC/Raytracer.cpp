#include <Rendering/PMVC/Raytracer.h>

#include <Rendering/PMVC/CubemapRenderInstance.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/ScreenPass.h>
#include <Editor/Light.h>
#include <cstddef>
#include "ScopedCmdBuffer.h"


Raytracer::Raytracer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
	const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance, uint32_t cubemapSize, VkFormat format) : _renderPipelineManager(renderPipelineManager),
	_resourceManager(resourceManager), _device(device), _instance(instance)
{ }

Raytracer::~Raytracer()
{
	//TODO cleanup
}

void TraceRays() {

}

void Raytracer::Initialize()
{
	CreateCommandPool(_device->GetQueueFamilies()._graphics.value());
	GetRaytracingComponents();
	
	CreateVertexBufferFromMesh();
	CreateIndexBufferFromMesh();

	vkGetDeviceQueue(
		_device,
		_device->GetQueueFamilies()._graphics.value(),
		0,
		&_queue);
	/*
	
	When creating _vertexBuffer and _indexBuffer, include:

VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | 
VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR

*/

	VkAccelerationStructureGeometryKHR geometry;
	VkAccelerationStructureBuildRangeInfoKHR rangeInfo;

	PrimitiveToGeometry(_vertexBuffer,
		_indexBuffer,
		static_cast<uint32_t>(_cageMesh._vertices.rows()),
		static_cast<uint32_t>(_cageMesh._faces.rows()),
		geometry,
		rangeInfo);

	VkAccelerationStructureKHR cageBLAS = VK_NULL_HANDLE; // The BLAS handle
	VkDeviceMemory cageBLASMemory = VK_NULL_HANDLE;       // Memory for the BLAS

	// Build the BLAS for the cage mesh
	CreateAccelerationStructure(
		VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR,
		cageBLAS,          // VkAccelerationStructureKHR output
		cageBLASMemory,    // VkDeviceMemory output
		_vertexBuffer,     // Vertex buffer of the cage
		_indexBuffer,      // Index buffer of the cage
		static_cast<uint32_t>(_cageMesh._vertices.rows()), // number of vertices
		static_cast<uint32_t>(_cageMesh._faces.rows()),    // number of triangles
		VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR
	);

	CreateBottomLevelAS();
	CreateRaytraceDescriptorLayout();
	CreateRaytracingPipeline();

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
	// Make sure buffers exist
	//CreateVertexBufferFromMesh();
	//CreateIndexBufferFromMesh();

	const uint32_t vertexCount =
		static_cast<uint32_t>(_deformableMesh._vertices.rows());

	const uint32_t triangleCount =
		static_cast<uint32_t>(_deformableMesh._faces.rows());

	VkAccelerationStructureGeometryKHR geometry{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR
	};

	VkAccelerationStructureBuildRangeInfoKHR rangeInfo{};

	// Convert mesh buffers to AS geometry
	PrimitiveToGeometry(
		_vertexBuffer,
		_indexBuffer,
		vertexCount,
		triangleCount,
		geometry,
		rangeInfo
	);

	// Build BLAS
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

	LOG_DEBUG("Bottom-level acceleration structure built successfully");
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
	MemoryMappedBuffer& vertexBuffer,
	MemoryMappedBuffer& indexBuffer,
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
			VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
			VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR,
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
	MemoryMappedBuffer& vertexBuffer,
	MemoryMappedBuffer& indexBuffer,
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

void Raytracer::CreateShaderBindingTable()
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
}

void Raytracer::CreateVertexBufferFromMesh()
{
	const EigenMesh& geom = _cageMesh;
	const auto& positions = geom._vertices;
	const auto& faces = geom._faces;

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

	_vertexBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>(
			reinterpret_cast<std::byte*>(vertexData.data()),
			vertexData.size() * sizeof(RTVertex)
		),
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	//memcpy(
		//_vertexBuffer._mappedData,
		//vertexData.data(),
		//vertexData.size() * sizeof(RTVertex)
	//);
}

void Raytracer::CreateIndexBufferFromMesh()
{
	const EigenMesh& geom = _cageMesh;
	const auto& faces = geom._faces;

	const uint32_t indexCount = static_cast<uint32_t>(faces.rows()) * 3;

	std::vector<uint32_t> indices(indexCount);

	for (uint32_t tri = 0; tri < faces.rows(); ++tri) {
		for (uint32_t v = 0; v < 3; ++v) {
			indices[tri * 3 + v] = faces(tri, v);
		}
	}
	//for (uint32_t i = 0; i < indexCount; ++i)
		//indices[i] = i;

	_indexBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span(indices),
		VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	//memcpy(
		//_indexBuffer._mappedData,
		//indices.data(),
		//indices.size() * sizeof(uint32_t)
	//);
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