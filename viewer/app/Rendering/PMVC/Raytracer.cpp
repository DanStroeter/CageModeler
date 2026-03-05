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
#include <Rendering/PMVC/ScopedCmdBuffer.h>

//--- Public --- 
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

void Raytracer::Initialize()
{
	const auto queueFamilies = _device->GetQueueFamilies();
	if (!queueFamilies._graphics.has_value())
	{
		throw std::runtime_error("Raytracer requires a graphics queue family for ray tracing dispatch");
	}

	_queueFamilyIndex = queueFamilies._graphics.value();
	GetRaytracingComponents();
	CreateCommandPool();
	vkGetDeviceQueue(
		_device,
		_queueFamilyIndex,
		0,
		&_queue);
	CreateCageBuffers(_cageMesh, _cageGeometry);
	CreateRayOriginBuffer(_deformableMesh, _deformableGeometry);
	CreateAccelerationStructures();

	CreateRayBuffers();

	CreateRayTracingDescriptorSet();
	UpdateDescriptorSet();

	LoadShaders();
	CreateRayTracingPipeline();
	CreateReadbackResources();
}

MeshOperationResult<MeshComputeWeightsOperationResult> Raytracer::ComputeCoordinates() {
	StartRayTrace();
	WaitForTrace();
	SubmitReadback();
	std::vector<HitBufferData> results = GetHitResults();
	WriteHitsToFile("RaytracingHits.txt", results);

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

// --- Private --- 
void Raytracer::StartRayTrace()
{
	LOG_INFO("Starting ray trace dispatch...");

	if (_traceSync.commandBuffer == VK_NULL_HANDLE)
	{
		VkCommandBufferAllocateInfo allocInfo{};
		allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		allocInfo.commandPool = _commandPool;
		allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		allocInfo.commandBufferCount = 1;

		VK_CHECK(vkAllocateCommandBuffers(
			_device, &allocInfo, &_traceSync.commandBuffer));
	}

	if (_traceSync.fence == VK_NULL_HANDLE)
	{
		VkFenceCreateInfo fenceInfo{};
		fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
		fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

		VK_CHECK(vkCreateFence(
			_device, &fenceInfo, nullptr, &_traceSync.fence));
	}

	vkWaitForFences(_device, 1, &_traceSync.fence, VK_TRUE, UINT64_MAX);
	vkResetFences(_device, 1, &_traceSync.fence);

	vkResetCommandBuffer(_traceSync.commandBuffer, 0);

	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	VK_CHECK(vkBeginCommandBuffer(_traceSync.commandBuffer, &beginInfo));

	// Bind pipeline
	vkCmdBindPipeline(
		_traceSync.commandBuffer,
		VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
		_rtPipeline.pipeline
	);

	vkCmdBindDescriptorSets(
		_traceSync.commandBuffer,
		VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
		_rtPipeline.layout,
		0,
		1,
		&_rtPipeline.descriptorSet,
		0,
		nullptr
	);

	vkCmdPushConstants(
		_traceSync.commandBuffer,
		_rtPipeline.layout,
		VK_SHADER_STAGE_RAYGEN_BIT_KHR |
		VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR,
		0,
		sizeof(PushConstants),
		&_pushConstants
	);

	const uint32_t totalRays = _pushConstants.raysPerVertex * _pushConstants.maxHitsPerRay * _pushConstants.vertexCount;

	if (vkCreateRayTracingPipelinesKHR == nullptr ||
		vkCmdTraceRaysKHR == nullptr) {

		LOG_ERROR("Ray tracing functions not available!");
	}

	vkCmdTraceRaysKHR(
		_traceSync.commandBuffer,
		&_raygenRegion,
		&_missRegion,
		&_hitRegion,
		&_callableRegion,
		totalRays,
		1,
		1
	);

	LOG_INFO("Tracing {} rays (1 vertex × {} directions)",
		totalRays,
		_pushConstants.raysPerVertex);

	VK_CHECK(vkEndCommandBuffer(_traceSync.commandBuffer));

	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &_traceSync.commandBuffer;

	VK_CHECK(vkQueueSubmit(_queue, 1, &submitInfo, _traceSync.fence));

	_traceSync.isTracing = true;
	
	//wait for idle
	vkDeviceWaitIdle(_device);

	LOG_INFO("Ray trace dispatch submitted!");
}

// --- Device Properties ---
void Raytracer::GetRaytracingComponents()
{
	_rtProperties = {};
	_rtProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;

	_asProperties = {};
	_asProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR;

	VkPhysicalDeviceProperties2 prop2{};
	prop2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
	prop2.pNext = &_rtProperties;
	_rtProperties.pNext = &_asProperties;

	vkGetPhysicalDeviceProperties2(_device->GetPhysicalDeviceHandle(), &prop2);

	LOG_INFO("Ray tracing max recursion depth: {}", _rtProperties.maxRayRecursionDepth);
	LOG_INFO("Acceleration structure max instance count: {}", _asProperties.maxInstanceCount);
}

// --- Queue and Command Pool ---
void Raytracer::CreateCommandPool() {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = _queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_commandPool) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create command pool!");
	}
}

// --- Scene Data ---
void Raytracer::CreateCageBuffers(EigenMesh& geometry, CageBuffers& cageBuffers)
{
	const auto& positions = geometry._vertices;
	const auto& faces = geometry._faces;

	// For BLAS: Expanded triangle list with sequential indices
	std::vector<glm::vec4> asVertices;
	std::vector<uint32_t> asIndices;

	asVertices.reserve(faces.rows() * 3);
	asIndices.reserve(faces.rows() * 3);

	for (int tri = 0; tri < faces.rows(); ++tri)
	{
		for (int v = 0; v < 3; ++v)
		{
			int idx = faces(tri, v);

			asVertices.emplace_back(
				float(positions(idx, 0)),
				float(positions(idx, 1)),
				float(positions(idx, 2)),
				1.0f  
			);

			asIndices.push_back(asIndices.size());  // Sequential indices
		}
	}

	// Create BLAS buffers
	VkDeviceSize asVertexSize = asVertices.size() * sizeof(glm::vec4);
	VkDeviceSize asIndexSize = asIndices.size() * sizeof(uint32_t);

	auto stagingASVertices = _resourceManager->CreateBufferAndCopy(
		std::as_bytes(std::span(asVertices)),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

	auto stagingASIndices = _resourceManager->CreateBufferAndCopy(
		std::as_bytes(std::span(asIndices)),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

	// These buffers are for BLAS build
	cageBuffers.vertexBuffer = _resourceManager->AllocateDeviceBuffer(
		asVertexSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	cageBuffers.indexBuffer = _resourceManager->AllocateDeviceBuffer(
		asIndexSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	CopyBuffer(stagingASVertices._deviceBuffer, cageBuffers.vertexBuffer._deviceBuffer, asVertexSize);
	CopyBuffer(stagingASIndices._deviceBuffer, cageBuffers.indexBuffer._deviceBuffer, asIndexSize);

	stagingASVertices.ReleaseResource(_device);
	stagingASIndices.ReleaseResource(_device);

	cageBuffers.vertexCount = static_cast<uint32_t>(asVertices.size());  // Expanded vertices
	cageBuffers.indexCount = static_cast<uint32_t>(asIndices.size());    // Sequential indices
}

void Raytracer::CreateRayOriginBuffer(EigenMesh& geometry, RayOriginBuffer& rayOriginBuffer)
{
	const auto& positions = geometry._vertices;  // Unique vertices

	// For ray origins: Unique vertices (used as starting points)
	std::vector<glm::vec4> vertices;

	vertices.reserve(positions.rows());

	for (int i = 0; i < positions.rows(); ++i)
	{
		vertices.emplace_back(
			float(positions(i, 0)),
			float(positions(i, 1)),
			float(positions(i, 2)),
			1.0f  
		);
	}

	// Create ray origin buffer
	VkDeviceSize vertexSize = vertices.size() * sizeof(glm::vec4);

	auto stagingVertices = _resourceManager->CreateBufferAndCopy(
		std::as_bytes(std::span(vertices)),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

	// This buffer is for shader storage (ray origins)
	rayOriginBuffer.vertexBuffer = _resourceManager->AllocateDeviceBuffer(
		vertexSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |  // Shader storage
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	CopyBuffer(stagingVertices._deviceBuffer, rayOriginBuffer.vertexBuffer._deviceBuffer, vertexSize);
	stagingVertices.ReleaseResource(_device);

	rayOriginBuffer.vertexCount = static_cast<uint32_t>(vertices.size());  // Unique vertices
}

// --- Acceleration Structures ---
void Raytracer::CreateAccelerationStructures()
{
	LOG_INFO("Building acceleration structures...");
	LOG_INFO("  Cage vertex count: {}", _cageGeometry.vertexCount);
	LOG_INFO("  Cage index count: {}", _cageGeometry.indexCount);
	LOG_INFO("  Cage triangle count: {}", _cageGeometry.indexCount / 3);

	CreateBLAS();
	CreateTLAS();

	// Verify TLAS was created
	if (_tlas.handle == VK_NULL_HANDLE) {
		LOG_ERROR("TLAS creation failed!");
	}
	else {
		LOG_INFO("TLAS created successfully with address: 0x{:016X}", _tlas.deviceAddress);
	}
	// Verify BLAS was created
	if (_blas.handle == VK_NULL_HANDLE) {
		LOG_ERROR("BLAS creation failed!");
	}
	else {
		LOG_INFO("BLAS created successfully with address: 0x{:016X}", _blas.deviceAddress);
	}
}

void Raytracer::CreateBLAS()
{
	VkAccelerationStructureGeometryTrianglesDataKHR triangles{
		VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR
	};

	triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
	triangles.vertexStride = sizeof(glm::vec4);
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

void Raytracer::BuildAccelerationStructure(VkAccelerationStructureGeometryKHR& geometry, uint32_t primitiveCount, VkAccelerationStructureTypeKHR type, AccelerationStructure& outAS)
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

// --- Ray Tracing Pipeline ---
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

void Raytracer::CreateRayTracingDescriptorSet() {
	LOG_INFO("Creating ray tracing descriptor set...");

	// Create Layout
	std::vector<VkDescriptorSetLayoutBinding> bindings = {
		// Binding 0: Acceleration structure
		{
			.binding = 0,
			.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_ALL
		},
		// Binding 1: Deformable vertices
		{
			.binding = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_ALL
		},
		// Binding 2: Ray directions
		{
			.binding = 2,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_ALL
		},
		// Binding 3: Hit buffer
		{
			.binding = 3,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_ALL
		},
		// Binding 4: Atomic counter for hit buffer
		{
			.binding = 4,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 1,
			.stageFlags = VK_SHADER_STAGE_ALL
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
			.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			.descriptorCount = 4
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
	// Update descriptor set with actual buffer bindings
	std::vector<VkWriteDescriptorSet> descriptorWrites;
	std::vector<VkDescriptorBufferInfo> bufferInfos;

	// Reserve space for all writes
	descriptorWrites.reserve(5);
	bufferInfos.reserve(4);

	// Helper lambda to add buffer writes
	auto addBufferWrite = [&](uint32_t binding, const Buffer& buffer,
		VkDeviceSize size) {
		VkDeviceSize bufferSize = buffer._allocatedSize; // You need to store buffer size somewhere
        VkDeviceSize actualSize = (size <= bufferSize) ? size : bufferSize;
        
        if (size > bufferSize) {
            LOG_WARN("Buffer size mismatch for binding {}: requested {} bytes but buffer only has {} bytes", 
                     binding, size, bufferSize);
        }
        
        VkDescriptorBufferInfo bufferInfo{
            .buffer = buffer._deviceBuffer,
            .offset = 0,
            .range = actualSize  // Use validated size
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

	// Binding 0: Acceleration structure
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

	// Binding 1: Deformable vertices (origins of rays)
	VkDeviceSize vertexBufferSize = static_cast<VkDeviceSize>(
		_pushConstants.vertexCount) * sizeof(glm::vec4);
	addBufferWrite(1, _deformableGeometry.vertexBuffer, vertexBufferSize);

	// Binding 2: Ray directions
	uint32_t totalRays = _pushConstants.vertexCount * _pushConstants.raysPerVertex;
	VkDeviceSize directionsSize = static_cast<VkDeviceSize>(totalRays) * sizeof(glm::vec4);
	addBufferWrite(2, _rayBuffers.rayDirections, directionsSize);

	// Binding 3: Hit buffer
	uint32_t maxTotalHits = _pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;
	VkDeviceSize hitBufferSize = static_cast<VkDeviceSize>(maxTotalHits) * sizeof(HitBufferData);
	addBufferWrite(3, _rayBuffers.hitBuffer, hitBufferSize);

	// Binding 4: Atomic counter
	VkDeviceSize atomicCounterSize = sizeof(uint32_t);
	addBufferWrite(4, _rayBuffers.atomicCounter, atomicCounterSize);

	// Update all descriptors at once
	vkUpdateDescriptorSets(_device,
		static_cast<uint32_t>(descriptorWrites.size()),
		descriptorWrites.data(),
		0, nullptr);

	LOG_INFO("Updated descriptor set with {} writes", descriptorWrites.size());
}

// --- Shaders and Shader Binding Table ---
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

	//uint32_t raygenSize = alignUp(handleSize, handleAlignment);
	//uint32_t missSize = alignUp(handleSize, handleAlignment);
	//uint32_t hitSize = alignUp(handleSize, handleAlignment);

	uint32_t handleSizeAligned = alignUp(handleSize, handleAlignment);
	uint32_t raygenStride = alignUp(handleSizeAligned, baseAlignment);
	uint32_t raygenSize = raygenStride;

	uint32_t missStride = handleSizeAligned;
	uint32_t missSize = alignUp(missStride, baseAlignment);

	uint32_t hitStride = handleSizeAligned;
	uint32_t hitSize = alignUp(hitStride, baseAlignment);
	// Calculate offsets
	uint32_t raygenOffset = 0;
	//uint32_t missOffset = alignUp(raygenSize, baseAlignment);
	//uint32_t hitOffset = alignUp(missOffset + missSize, baseAlignment);
	uint32_t missOffset = raygenOffset + raygenSize;
	uint32_t hitOffset = missOffset + missSize;


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
		.stride = raygenStride,
		.size = raygenSize
	};

	_missRegion = {
		.deviceAddress = sbtAddress + missOffset,
		.stride = missStride,
		.size = missSize
	};

	_hitRegion = {
		.deviceAddress = sbtAddress + hitOffset,
		.stride = hitStride,
		.size = hitSize
	};

	_callableRegion = {
		.deviceAddress = 0,
		.stride = 0,
		.size = 0
	};

	LOG_INFO("Shader binding table created");
	LOG_INFO("SBT regions:");
	LOG_INFO("  Raygen: address=0x{:016X}, size={}, stride={}",
		_raygenRegion.deviceAddress, _raygenRegion.size, _raygenRegion.stride);
	LOG_INFO("  Miss: address=0x{:016X}, size={}, stride={}",
		_missRegion.deviceAddress, _missRegion.size, _missRegion.stride);
	LOG_INFO("  Hit: address=0x{:016X}, size={}, stride={}",
		_hitRegion.deviceAddress, _hitRegion.size, _hitRegion.stride);
}

void Raytracer::LoadShaders() {
	try {
		auto raygenCode = LoadSPIRV("RaygenDebug.rgen.spv");
		auto missCode = LoadSPIRV("MissDebug.rmiss.spv");
		auto hitCode = LoadSPIRV("HitDebug.rchit.spv");

		LOG_INFO("Shader sizes - Raygen: {} bytes, Miss: {} bytes, Hit: {} bytes",
			raygenCode.size() * sizeof(uint32_t),
			missCode.size() * sizeof(uint32_t),
			hitCode.size() * sizeof(uint32_t));

		// Verify shaders aren't empty
		if (raygenCode.empty() || missCode.empty() || hitCode.empty()) {
			throw std::runtime_error("One or more shaders are empty!");
		}

		_raygenShader = CreateShaderModule(raygenCode);
		_missShader = CreateShaderModule(missCode);
		_hitShader = CreateShaderModule(hitCode);
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

// --- Ray Tracing Buffers ---
void Raytracer::CreateRayBuffers()
{
	_pushConstants.vertexCount = _deformableGeometry.vertexCount;

	const uint32_t totalRays = _pushConstants.vertexCount * _pushConstants.raysPerVertex;
	const uint32_t maxTotalHits = totalRays * _pushConstants.maxHitsPerRay;

	LOG_INFO("Setting up ray directions:");
	LOG_INFO("  Vertex count: {}", _pushConstants.vertexCount);
	LOG_INFO("  Total rays: {}", totalRays);
	LOG_INFO("  Max possible hits: {}", maxTotalHits);

	// Ray directions (6 axis-aligned directions for now to verify that it works)

	std::vector<glm::vec4> baseDirections =
	{
		{  1.0f,  0.0f,  0.0f, 0.0f }, // Right
		{ -1.0f,  0.0f,  0.0f, 0.0f }, // Left
		{  0.0f,  1.0f,  0.0f, 0.0f }, // Up
		{  0.0f, -1.0f,  0.0f, 0.0f }, // Down
		{  0.0f,  0.0f,  1.0f, 0.0f }, // Front
		{  0.0f,  0.0f, -1.0f, 0.0f }  // Back
	};
	std::vector<glm::vec4> allDirections;
	allDirections.reserve(totalRays);

	// For each vertex, add all 6 directions
	for (uint32_t vertexIdx = 0; vertexIdx < _pushConstants.vertexCount; ++vertexIdx) {
		for (const auto& dir : baseDirections) {
			allDirections.push_back(dir);
		}
	}

	const VkDeviceSize directionsBufferSize =
		static_cast<VkDeviceSize>(allDirections.size()) * sizeof(glm::vec4);

	LOG_INFO("  Creating directions buffer with {} bytes ({} directions)",
		directionsBufferSize, allDirections.size());

	auto stagingDirections = _resourceManager->CreateBufferAndCopy(
		std::span(reinterpret_cast<const std::byte*>(allDirections.data()),
			directionsBufferSize),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_rayBuffers.rayDirections = _resourceManager->AllocateDeviceBuffer(
		directionsBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	if (_rayBuffers.rayDirections._deviceBuffer == VK_NULL_HANDLE)
		throw std::runtime_error("Failed to create ray directions buffer");

	CopyBuffer(
		stagingDirections._deviceBuffer,
		_rayBuffers.rayDirections._deviceBuffer,
		directionsBufferSize
	);

	stagingDirections.ReleaseResource(_device);

	LOG_INFO("  Ray directions buffer created: {}",
		(void*)_rayBuffers.rayDirections._deviceBuffer);

	// Hit buffer
	const VkDeviceSize hitBufferSize = static_cast<VkDeviceSize>(maxTotalHits) * sizeof(HitBufferData);

	LOG_INFO("  Hit buffer size: {} bytes", hitBufferSize);

	std::vector<HitBufferData> zeroData(maxTotalHits, HitBufferData{ 0 });

	auto stagingHitBuffer = _resourceManager->CreateBufferAndCopy(
		std::span(zeroData),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_rayBuffers.hitBuffer = _resourceManager->AllocateDeviceBuffer(
		hitBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(
		stagingHitBuffer._deviceBuffer,
		_rayBuffers.hitBuffer._deviceBuffer,
		hitBufferSize
	);

	stagingHitBuffer.ReleaseResource(_device);

	LOG_INFO("  Hit buffer created: {}",
		(void*)_rayBuffers.hitBuffer._deviceBuffer);

	// Atomic counter buffer
	const VkDeviceSize counterBufferSize = sizeof(uint32_t);
	std::vector<uint32_t> counterInit(1, 0);

	auto stagingCounter = _resourceManager->CreateBufferAndCopy(
		std::span(counterInit),
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	_rayBuffers.atomicCounter = _resourceManager->AllocateDeviceBuffer(
		counterBufferSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
	);

	CopyBuffer(
		stagingCounter._deviceBuffer,
		_rayBuffers.atomicCounter._deviceBuffer,
		counterBufferSize
	);

	stagingCounter.ReleaseResource(_device);

	LOG_INFO("Successfully set up 6 fixed rays from 1 vertex");
}

// --- Sync ---
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

// --- Readback ---
void Raytracer::CreateReadbackResources()
{
	LOG_INFO("Creating readback resources...");

	const uint32_t maxTotalHits =
		_pushConstants.vertexCount *
		_pushConstants.raysPerVertex *
		_pushConstants.maxHitsPerRay;

	const VkDeviceSize hitBufferSize =
		static_cast<VkDeviceSize>(maxTotalHits) * sizeof(HitBufferData);

	_readback.size = hitBufferSize;

	// Command buffer
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.commandPool = _commandPool;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandBufferCount = 1;
	VK_CHECK(vkAllocateCommandBuffers(_device, &allocInfo, &_readback.copyCmd));

	// Fence
	VkFenceCreateInfo fenceInfo{};
	fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
	VK_CHECK(vkCreateFence(_device, &fenceInfo, nullptr, &_readback.copyCompleteFence));

	// Staging buffer
	_readback.stagingBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>((std::byte*)_readback.mappedData,
			static_cast<size_t>(hitBufferSize)),
		VK_BUFFER_USAGE_TRANSFER_DST_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	LOG_INFO("Readback resources created successfully");
}

void Raytracer::SubmitReadback() {
	// Wait for trace to complete
	WaitForTrace();

	// Reset copy fence
	vkResetFences(_device, 1, &_readback.copyCompleteFence);

	// Reset copy command buffer
	vkResetCommandBuffer(_readback.copyCmd, 0);

	// Begin copy command buffer
	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	VK_CHECK(vkBeginCommandBuffer(_readback.copyCmd, &beginInfo));

	VkBufferMemoryBarrier barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
	barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
	barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.buffer = _rayBuffers.hitBuffer._deviceBuffer;
	barrier.offset = 0;
	barrier.size = _readback.size;

	vkCmdPipelineBarrier(
		_readback.copyCmd,
		VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR,
		VK_PIPELINE_STAGE_TRANSFER_BIT,
		0,
		0, nullptr,
		1, &barrier,
		0, nullptr
	);

	// Copy hit buffer from device-local to staging
	VkBufferCopy copyRegion{
		.srcOffset = 0,
		.dstOffset = 0,
		.size = _readback.size
	};

	vkCmdCopyBuffer(
		_readback.copyCmd,
		_rayBuffers.hitBuffer._deviceBuffer,
		_readback.stagingBuffer._deviceBuffer,
		1, &copyRegion
	);

	VK_CHECK(vkEndCommandBuffer(_readback.copyCmd));

	// Submit copy with fence
	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &_readback.copyCmd;

	VK_CHECK(vkQueueSubmit(_queue, 1, &submitInfo, _readback.copyCompleteFence));

	LOG_INFO("Readback copy submitted");
}

std::vector<Raytracer::HitBufferData> Raytracer::GetHitResults()
{
	WaitForReadbackComplete();

	uint32_t hitCount = _readback.size;

	HitBufferData* hits = static_cast<HitBufferData*>(_readback.stagingBuffer._mappedData);

	std::vector<HitBufferData> results(hits, hits + hitCount);

	return results;
}

void Raytracer::WaitForReadbackComplete() {
	LOG_INFO("Waiting for readback to complete...");
	VK_CHECK(vkWaitForFences(_device, 1, &_readback.copyCompleteFence, VK_TRUE, UINT64_MAX));
	LOG_INFO("Readback complete");
}

void Raytracer::WriteHitsToFile(const std::string& filename, const std::vector<HitBufferData>& hits)
{
	std::ofstream file(filename);
	if (!file.is_open())
		throw std::runtime_error("Failed to open hit write file: " + filename);

	auto now = std::chrono::system_clock::now();
	auto time = std::chrono::system_clock::to_time_t(now);

	file << "========================================\n";
	file << "Ray Tracing Hit Results\n";
	file << "Generated: " << std::ctime(&time);
	file << "========================================\n\n";

	file << "===== CONFIGURATION =====\n";
	file << "Vertex count: " << _pushConstants.vertexCount << "\n";
	file << "Rays per vertex: " << _pushConstants.raysPerVertex << "\n";
	file << "Max hits per ray: " << _pushConstants.maxHitsPerRay << "\n";
	file << "Total possible hits: "
		<< (_pushConstants.vertexCount *
			_pushConstants.raysPerVertex *
			_pushConstants.maxHitsPerRay) << "\n";
	file << "Vector hit size: " << hits.size() << "\n\n";

	file << "===== Rays =====\n";
	file << "Every ray index should have " << _pushConstants.raysPerVertex * _pushConstants.maxHitsPerRay << " entires.\n";;

	for (size_t i = 0; i < hits.size(); ++i)
	{
		const auto& hit = hits[i]; 

		file << "  Ray index: " << hit.rayIndex << "\n";
		file << "\n";
	}
}

// --- Helper Functions ---
void Raytracer::CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size)
{
	ScopedCmdBuffer cmd(_device, _commandPool);

	VkBufferCopy copyRegion{};
	copyRegion.srcOffset = 0;
	copyRegion.dstOffset = 0;
	copyRegion.size = size;

	vkCmdCopyBuffer(
		cmd.Get(),   // command buffer from ScopedCmdBuffer
		src,
		dst,
		1,
		&copyRegion
	);

	cmd.SubmitAndWait(_queue);
}

VkDeviceAddress Raytracer::GetBufferAddress(const Buffer& buffer) const
{
	VkBufferDeviceAddressInfo info{
		VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO
	};
	info.buffer = buffer._deviceBuffer;

	return vkGetBufferDeviceAddress(_device, &info);
}
