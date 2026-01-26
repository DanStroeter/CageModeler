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


Raytracer::Raytracer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
	const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance, uint32_t cubemapSize, VkFormat format) : _renderPipelineManager(renderPipelineManager),
	_resourceManager(resourceManager), _device(device), _instance(instance), _cubemapSize(cubemapSize), _format(format)
{ }

Raytracer::~Raytracer()
{
	//TODO cleanup
}

void TraceRays() {

}

void Raytracer::Initialize()
{
	GetRaytracingComponents();
	CreateRaytracingPipeline();
	
	CreateVertexBufferFromMesh();
	CreateIndexBufferFromMesh();

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

	nvvk::AccelerationStructure cageBLAS;
	createAccelerationStructure(VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR,
		cageBLAS,
		geometry,
		rangeInfo,
		VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR);

	CreateBottomLevelAS();
	CreateTopLevelAS();
}

void Reaytracer::CreateBottomLevelAS()
{
	SCOPED_TIMER(__FUNCTION__);

	// Prepare geometry information for all meshes
	m_blasAccel.resize(m_sceneResource.meshes.size());

	// For now, just log that we're ready to build BLAS
	LOGI("  Ready to build %zu bottom-level acceleration structures\n", m_sceneResource.meshes.size());

	// TODO: In Phase 3, we'll add the actual building:
	// For each mesh 
	//   - create acceleration structure geometry from internal mesh primitive (primitiveToGeometry)
	//   - create acceleration structure
}

void Raytracer::CreateTopLevelAS()
{
	SCOPED_TIMER(__FUNCTION__);

	// VkTransformMatrixKHR is row-major 3x4, glm::mat4 is column-major; transpose before memcpy.
	auto toTransformMatrixKHR = [](const glm::mat4& m) {
		VkTransformMatrixKHR t;
		memcpy(&t, glm::value_ptr(glm::transpose(m)), sizeof(t));
		return t;
	};

	// Prepare instance data for TLAS
	std::vector<VkAccelerationStructureInstanceKHR> tlasInstances;
	tlasInstances.reserve(m_sceneResource.instances.size());

	for (const shaderio::GltfInstance& instance : m_sceneResource.instances)
	{
		VkAccelerationStructureInstanceKHR asInstance{};
		asInstance.transform = toTransformMatrixKHR(instance.transform);  // Position of the instance
		asInstance.instanceCustomIndex = instance.meshIndex;                       // gl_InstanceCustomIndexEXT
		// asInstance.accelerationStructureReference = m_blasAccel[instance.meshIndex].address;  // Will be set in Phase 3
		asInstance.instanceShaderBindingTableRecordOffset = 0;  // We will use the same hit group for all objects
		asInstance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_CULL_DISABLE_BIT_NV;  // No culling - double sided
		asInstance.mask = 0xFF;
		tlasInstances.emplace_back(asInstance);
	}

	// For now, just log that we're ready to build TLAS
	LOGI("  Ready to build top-level acceleration structure with %zu instances\n", tlasInstances.size());

	// TODO: In Phase 3, we'll add the actual building:
	// 1. Create and upload instance buffer
	// 2. Create TLAS geometry from instances
	// 3. Call createAccelerationStructure with TLAS type
}

void Raytracer::CreateAccelerationStructure(VkAccelerationStructureTypeKHR asType,  // The type of acceleration structure (BLAS or TLAS)
	nvvk::AccelerationStructure& accelStruct,  // The acceleration structure to create
	VkAccelerationStructureGeometryKHR& asGeometry,  // The geometry to build the acceleration structure from
	VkAccelerationStructureBuildRangeInfoKHR& asBuildRangeInfo,  // The range info for building the acceleration structure
	VkBuildAccelerationStructureFlagsKHR flags  // Build flags (e.g. prefer fast trace)
)
{
	VkDevice device = m_app->getDevice();

	// Helper function to align a value to a given alignment
	auto alignUp = [](auto value, size_t alignment) noexcept { return ((value + alignment - 1) & ~(alignment - 1)); };

	// Fill the build information with the current information, the rest is filled later (scratch buffer and destination AS)
	VkAccelerationStructureBuildGeometryInfoKHR asBuildInfo{
		.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR,
		.type = asType,  // The type of acceleration structure (BLAS or TLAS)
		.flags = flags,   // Build flags (e.g. prefer fast trace)
		.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR,  // Build mode vs update
		.geometryCount = 1,                                               // Deal with one geometry at a time
		.pGeometries = &asGeometry,  // The geometry to build the acceleration structure from
	};

	// One geometry at a time (could be multiple)
	std::vector<uint32_t> maxPrimCount(1);
	maxPrimCount[0] = asBuildRangeInfo.primitiveCount;

	// Find the size of the acceleration structure and the scratch buffer
	VkAccelerationStructureBuildSizesInfoKHR asBuildSize{ .sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
	vkGetAccelerationStructureBuildSizesKHR(device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &asBuildInfo,
		maxPrimCount.data(), &asBuildSize);

	// Make sure the scratch buffer is properly aligned
	VkDeviceSize scratchSize = alignUp(asBuildSize.buildScratchSize, m_asProperties.minAccelerationStructureScratchOffsetAlignment);

	// Create the scratch buffer to store the temporary data for the build
	nvvk::Buffer scratchBuffer;
	NVVK_CHECK(m_allocator.createBuffer(scratchBuffer, scratchSize,
		VK_BUFFER_USAGE_2_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_2_SHADER_DEVICE_ADDRESS_BIT
		| VK_BUFFER_USAGE_2_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR, VMA_MEMORY_USAGE_AUTO, {}, m_asProperties.minAccelerationStructureScratchOffsetAlignment));

	// Create the acceleration structure
	VkAccelerationStructureCreateInfoKHR createInfo{
		.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR,
		.size = asBuildSize.accelerationStructureSize,  // The size of the acceleration structure
		.type = asType,  // The type of acceleration structure (BLAS or TLAS)
	};
	NVVK_CHECK(m_allocator.createAcceleration(accelStruct, createInfo));

	// Build the acceleration structure
	{
		VkCommandBuffer cmd = m_app->createTempCmdBuffer();

		// Fill with new information for the build,scratch buffer and destination AS
		asBuildInfo.dstAccelerationStructure = accelStruct.accel;
		asBuildInfo.scratchData.deviceAddress = scratchBuffer.address;

		VkAccelerationStructureBuildRangeInfoKHR* pBuildRangeInfo = &asBuildRangeInfo;
		vkCmdBuildAccelerationStructuresKHR(cmd, 1, &asBuildInfo, &pBuildRangeInfo);

		m_app->submitAndWaitTempCmdBuffer(cmd);
	}
	// Cleanup the scratch buffer
	m_allocator.destroyBuffer(scratchBuffer);
}

void Raytracer::PrimitiveToGeometry(MemoryMappedBuffer& vertexBuffer,
	MemoryMappedBuffer& indexBuffer,
	uint32_t vertexCount,
	uint32_t triangleCount,
	VkAccelerationStructureGeometryKHR& geometry,
	VkAccelerationStructureBuildRangeInfoKHR& rangeInfo)
{
	// --- GPU buffer addresses ---
	VkDeviceAddress vertexAddress = VkDeviceAddress(vertexBuffer._deviceBuffer);
	VkDeviceAddress indexAddress = VkDeviceAddress(indexBuffer._deviceBuffer);

	// Stride per vertex (vec3, 3 floats = 12 bytes)
	VkDeviceSize vertexStride = sizeof(float) * 3;

	// --- Triangle data description ---
	VkAccelerationStructureGeometryTrianglesDataKHR triangles{
		.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR,
		.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT, // float3 positions
		.vertexData = {.deviceAddress = vertexAddress},
		.vertexStride = vertexStride,
		.maxVertex = vertexCount - 1,
		.indexType = VK_INDEX_TYPE_UINT32,       // assuming 32-bit indices
		.indexData = {.deviceAddress = indexAddress},
	};

	// --- Geometry description ---
	geometry = VkAccelerationStructureGeometryKHR{
		.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR,
		.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR,
		.geometry = {.triangles = triangles},
		.flags = VK_GEOMETRY_OPAQUE_BIT_KHR | VK_GEOMETRY_NO_DUPLICATE_ANY_HIT_INVOCATION_BIT_KHR,
	};

	// --- Build range info ---
	rangeInfo = VkAccelerationStructureBuildRangeInfoKHR{
		.primitiveCount = triangleCount,
		.firstVertex = 0,
		.primitiveOffset = 0,
		.transformOffset = 0,
	};
}

	

void Raytracer::GetRaytracingComponents()
{
	VkPhysicalDeviceProperties2 prop2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
	m_rtProperties.pNext = &m_asProperties;
	prop2.pNext = &m_rtProperties;
	vkGetPhysicalDeviceProperties2(_device->GetPhysicalDeviceHandle(), &prop2);

}

void Raytracer::CreateVertexBufferFromMesh()
{
	const EigenMesh& geom = _cageMesh;
	const auto& positions = geom._vertices;
	const auto& faces = geom._faces;

	std::vector<CubemapVertex> vertexData;
	vertexData.reserve(faces.rows() * 3);

	for (int tri = 0; tri < faces.rows(); ++tri)
	{
		for (int v = 0; v < 3; ++v)
		{
			int idx = faces(tri, v);

			glm::vec3 pos(
				static_cast<float>(positions(idx, 0)),
				static_cast<float>(positions(idx, 1)),
				static_cast<float>(positions(idx, 2))
			);

			vertexData.push_back({
				pos,
				static_cast<uint32_t>(tri), // triangle ID
				static_cast<uint32_t>(v)    // local vertex ID (0,1,2)
				});
		}
	}

	_vertexBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span<std::byte>((std::byte*)nullptr,
			vertexData.size() * sizeof(CubemapVertex)),
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	memcpy(_vertexBuffer._mappedData, vertexData.data(),
		vertexData.size() * sizeof(CubemapVertex));
}

void Raytracer::CreateIndexBufferFromMesh()
{
	const EigenMesh& geom = _cageMesh;
	const auto& faces = geom._faces;

	// Each triangle has 3 unique vertices in the vertex buffer
	const size_t vertexCount = faces.rows() * 3;

	std::vector<uint32_t> indices(vertexCount);
	for (uint32_t i = 0; i < vertexCount; ++i)
		indices[i] = i;

	_indexBuffer = _resourceManager->CreateBufferAndMapMemory(
		std::span(indices),
		VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
	);

	memcpy(_indexBuffer._mappedData, indices.data(),
		indices.size() * sizeof(uint32_t));
}

void Raytracer::CreateRaytracingPipeline()
{
	VkVertexInputBindingDescription bindingDesc{};
	bindingDesc.binding = 0;
	bindingDesc.stride = sizeof(CubemapVertex);
	bindingDesc.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

	std::array<VkVertexInputAttributeDescription, 3> attributeDescs{};
	attributeDescs[0].binding = 0;
	attributeDescs[0].location = 0; //position
	attributeDescs[0].format = VK_FORMAT_R32G32B32_SFLOAT;
	attributeDescs[0].offset = offsetof(CubemapVertex, _position);

	attributeDescs[1].binding = 0;
	attributeDescs[1].location = 1; // triangle ID
	attributeDescs[1].format = VK_FORMAT_R32_UINT;
	attributeDescs[1].offset = offsetof(CubemapVertex, _triangleID);

	attributeDescs[2].binding = 0;
	attributeDescs[2].location = 2; // vertex index
	attributeDescs[2].format = VK_FORMAT_R32_UINT;
	attributeDescs[2].offset = offsetof(CubemapVertex, _vertexIndex);

	// Vertex input state
	VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
	vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
	vertexInputInfo.vertexBindingDescriptionCount = 1;
	vertexInputInfo.pVertexBindingDescriptions = &bindingDesc;
	vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescs.size());
	vertexInputInfo.pVertexAttributeDescriptions = attributeDescs.data();

	// Blend state (disable blending, write all channels)
	VkPipelineColorBlendAttachmentState colorBlendAttachment{};
	colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT |
		VK_COLOR_COMPONENT_G_BIT |
		VK_COLOR_COMPONENT_B_BIT |
		VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachment.blendEnable = VK_FALSE;

	// Depth/stencil state
	VkPipelineDepthStencilStateCreateInfo depthStencil{};
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_TRUE;
	depthStencil.depthWriteEnable = VK_TRUE;
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
	depthStencil.depthBoundsTestEnable = VK_FALSE;
	depthStencil.stencilTestEnable = VK_FALSE;

	// Multisample state 
	VkPipelineMultisampleStateCreateInfo msaa{};
	msaa.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	msaa.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

	// Only the matrices layout is needed for cubemap rendering
	std::array descriptorSetLayouts{ _matricesLayout->GetReference() };

	VkViewport viewport{};
	viewport.x = 0.0f;
	viewport.y = 0.0f;
	viewport.width = (float)_cubemapSize;
	viewport.height = (float)_cubemapSize;
	viewport.minDepth = 0.0f;
	viewport.maxDepth = 1.0f;

	VkRect2D scissor{};
	scissor.offset = { 0, 0 };
	scissor.extent = { _cubemapSize, _cubemapSize };

	VkPushConstantRange pushConstantRange{};
	pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
	pushConstantRange.offset = 0;
	pushConstantRange.size = sizeof(CubemapPushConstants);

	// Build the pipeline
	_cubemapPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetColorBlendAttachments(std::span(&colorBlendAttachment, 1))
		.SetDepthStencilState(depthStencil)
		.SetDescriptorSetLayouts(std::span(descriptorSetLayouts))
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Cubemap.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/Cubemap.frag.spv")
		.AddPushConstantRange(pushConstantRange)
		.SetMultisampleState(msaa)
		.SetViewportAndScissor(viewport, scissor)
		.SetVertexInputBindingDescriptions(std::span(&bindingDesc, 1))
		.SetVertexInputAttributeDescriptions(std::span(attributeDescs))
		.Build();
}

MeshOperationResult<MeshComputeWeightsOperationResult> Raytracer::ComputeCoordinates() {

	Eigen::MatrixXd weights;
	Eigen::MatrixXd M = weights;
	Eigen::MatrixXd interpolatedWeights;
	Eigen::MatrixXd psi;
	std::vector<double> psiTri{ };
	std::vector<Eigen::Vector4d> psiQuad{ };
	return MeshComputeWeightsOperationResult{ std::move(M),
		std::move(weights),
		std::move(interpolatedWeights),
		std::move(psi),
		std::move(psiTri),
		std::move(psiQuad)};
}