#include <Rendering/PMVC/CubemapManager.h>

#include <Rendering/PMVC/CubemapRenderInstance.h>

#include <cagedeformations/InteriorDistance.h>

#include <Logging/Logging.h>

#include <algorithm>
#include <chrono>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/ScreenPass.h>
#include <Editor/Light.h>
#include <cstddef>


CubemapManager::CubemapManager(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
	const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance, uint32_t cubemapSize, VkFormat format) : _renderPipelineManager(renderPipelineManager),
	_resourceManager(resourceManager), _device(device), _instance(instance), _cubemapSize(cubemapSize), _format(format)
{ }

CubemapManager::~CubemapManager()
{
	Cleanup();
}

void CubemapManager::Cleanup() {
	if (!_device) return;
	vkDeviceWaitIdle(_device);
	if (_indexBuffer._deviceBuffer != VK_NULL_HANDLE) {
		_indexBuffer.ReleaseResource(_device);
		_indexBuffer = MemoryMappedBuffer();
	}
	if (_vertexBuffer._deviceBuffer != VK_NULL_HANDLE) {
		_vertexBuffer.ReleaseResource(_device);
		_vertexBuffer = MemoryMappedBuffer();
	}
	if (_renderPass != VK_NULL_HANDLE) {
		vkDestroyRenderPass(_device, _renderPass, nullptr);
		_renderPass = VK_NULL_HANDLE;
	}
	if (_renderPassCpu != VK_NULL_HANDLE) {
		vkDestroyRenderPass(_device, _renderPassCpu, nullptr);
		_renderPassCpu = VK_NULL_HANDLE;
	}
	if (_graphicCommandPool != VK_NULL_HANDLE) {
		vkDestroyCommandPool(_device, _graphicCommandPool, nullptr);
		_graphicCommandPool = VK_NULL_HANDLE;
	}
}

void CubemapManager::Initialize(uint32_t cubemapSize)
{
	if (!init) {
		_descriptorPool = CreateRenderResource<DescriptorPool>(_device);
		CreateCommandPool(_device->GetQueueFamilies()._graphics.value());
		_renderPass = CreateRenderPass(_format, false);
		_renderPassCpu = CreateRenderPass(_format, true);
		CreateDescriptorSetLayouts();
		_cubemapPipelineHandle = CreateCubemapRenderPipeline(false);
		_cubemapPipelineHandleCpu = CreateCubemapRenderPipeline(true);
		_cubemapPipelineHitHandle = CreateCubemapRenderPipeline(false, true);
		//SphereWeightInitialization(512);
		init = true;
	}
	if(cubemapSize != _cubemapSize) {
		_cubemapSize = cubemapSize;
		_renderPipelineManager->ReleasePipeline(_cubemapPipelineHandle);
		_renderPipelineManager->ReleasePipeline(_cubemapPipelineHandleCpu);
		_renderPipelineManager->ReleasePipeline(_cubemapPipelineHitHandle);
		_cubemapPipelineHandle = CreateCubemapRenderPipeline(false);
		_cubemapPipelineHandleCpu = CreateCubemapRenderPipeline(true);
		_cubemapPipelineHitHandle = CreateCubemapRenderPipeline(false, true);
	}
}

VkRenderPass CubemapManager::CreateRenderPass(VkFormat format, bool cpuTransfer) {
	VkRenderPass renderPass;
	
	VkAttachmentDescription colorAttachment{};
	colorAttachment.format = format;
	colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
	colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	if(!cpuTransfer)
		colorAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	else
		colorAttachment.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

	VkAttachmentDescription depthAttachment{};
	depthAttachment.format = _device->FindDepthFormat();
	depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
	depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	if (!cpuTransfer)
		depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
	else
		depthAttachment.finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

	VkAttachmentReference colorAttachmentRef{};
	colorAttachmentRef.attachment = 0;
	colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	VkAttachmentReference depthAttachmentRef{};
	depthAttachmentRef.attachment = 1;
	depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	VkSubpassDescription subpass{};
	subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpass.colorAttachmentCount = 1;
	subpass.pColorAttachments = &colorAttachmentRef;
	subpass.pDepthStencilAttachment = &depthAttachmentRef;

	VkSubpassDependency dependency{};
	dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
	dependency.dstSubpass = 0;
	dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
	dependency.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
	dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
	dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;


	std::array<VkAttachmentDescription, 2> attachments = { colorAttachment, depthAttachment };
	VkRenderPassCreateInfo renderPassInfo{};
	renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
	renderPassInfo.pAttachments = attachments.data();
	renderPassInfo.subpassCount = 1;
	renderPassInfo.pSubpasses = &subpass;
	renderPassInfo.dependencyCount = 1;
	renderPassInfo.pDependencies = &dependency;

	if (vkCreateRenderPass(_device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS) {
		throw std::runtime_error("failed to create Cubemap render pass!");
	}
	return renderPass;
}

void CubemapManager::CreateDescriptorSetLayouts() {
	VkDescriptorSetLayoutBinding layoutBinding{ };
	layoutBinding.binding = 0;
	layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	layoutBinding.descriptorCount = 1;
	layoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

	std::array<VkDescriptorSetLayoutBinding, 1> layoutBindings{ layoutBinding };

	_matricesLayout = _descriptorPool->CreateDescriptorSetLayout(layoutBindings);

	VkDescriptorSetLayoutBinding depthHistoryBinding{};
	depthHistoryBinding.binding = 0;
	depthHistoryBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	depthHistoryBinding.descriptorCount = 1;
	depthHistoryBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

	std::array<VkDescriptorSetLayoutBinding, 1> depthHistoryBindings{ depthHistoryBinding };
	_depthHistoryLayout = _descriptorPool->CreateDescriptorSetLayout(depthHistoryBindings);
}

PipelineHandle CubemapManager::CreateCubemapRenderPipeline(bool cpuTransfer, bool depthPeelPass)
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

	std::array<VkDescriptorSetLayout, 2> descriptorSetLayouts{
		_matricesLayout->GetReference(),
		depthPeelPass ? _depthHistoryLayout->GetReference() : VK_NULL_HANDLE
	};

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
	pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
	pushConstantRange.offset = 0;
	pushConstantRange.size = sizeof(CubemapPushConstants);

	VkRenderPass renderPass;
	if(!cpuTransfer)
		renderPass = _renderPass;
	else
		renderPass = _renderPassCpu;

	// Build the pipeline
	return _renderPipelineManager->BeginPipeline()
		.SetRenderPass(renderPass)
		.SetColorBlendAttachments(std::span(&colorBlendAttachment, 1))
		.SetDepthStencilState(depthStencil)
		.SetDescriptorSetLayouts(depthPeelPass ? std::span(descriptorSetLayouts) : std::span(descriptorSetLayouts.data(), size_t(1)))
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Cubemap.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, depthPeelPass ? "assets/shaders/CubemapHit.frag.spv" : "assets/shaders/Cubemap.frag.spv")
		.AddPushConstantRange(pushConstantRange)
		.SetMultisampleState(msaa)
		.SetViewportAndScissor(viewport, scissor)
		.SetVertexInputBindingDescriptions(std::span(&bindingDesc, 1))
		.SetVertexInputAttributeDescriptions(std::span(attributeDescs))
		.Build();
}

void CubemapManager::CreateVertexBufferFromMesh()
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

void CubemapManager::CreateIndexBufferFromMesh()
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

float CubemapManager::ComputeNearPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices)
{
	float minDist = std::numeric_limits<float>::max();
	for (const auto& v : vertices) {
		float dist = glm::length(v - camPos);
		if (dist < minDist) minDist = dist;
	}
	return minDist * 0.95f;
}

float CubemapManager::ComputeFarPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices)
{
	float maxDist = 0.0f;
	for (const auto& v : vertices) {
		float dist = glm::length(v - camPos);
		if (dist > maxDist) maxDist = dist;
	}
	return maxDist * 1.05f;
}

std::vector<CubemapVertex> CubemapManager::CreateCubemapVertexBuffer(const PolygonMesh& mesh)
{
	const auto& geom = mesh.GetGeometry();
	const auto& positions = geom._positions;
	const auto& indices = geom._indices;

	std::vector<CubemapVertex> vertexBuffer;
	vertexBuffer.reserve(indices.size());

	for (size_t tri = 0; tri < indices.size() / 3; ++tri)
	{
		uint32_t i0 = indices[tri * 3 + 0];
		uint32_t i1 = indices[tri * 3 + 1];
		uint32_t i2 = indices[tri * 3 + 2];

		vertexBuffer.push_back({ positions[i0], static_cast<uint32_t>(tri), 0 });
		vertexBuffer.push_back({ positions[i1], static_cast<uint32_t>(tri), 1 });
		vertexBuffer.push_back({ positions[i2], static_cast<uint32_t>(tri), 2 });
	}

	return vertexBuffer;
}

/*VkCommandBuffer CubemapManager::BeginOneTimeCommands() {
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandPool = _graphicCommandPool;
	allocInfo.commandBufferCount = 1;

	VkCommandBuffer cmd;
	vkAllocateCommandBuffers(_device, &allocInfo, &cmd);

	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	vkBeginCommandBuffer(cmd, &beginInfo);
	return cmd;
}

void CubemapManager::EndOneTimeCommands(VkCommandBuffer cmd) {
	vkEndCommandBuffer(cmd);
	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &cmd;

	VkQueue graphicsQueue;
	vkGetDeviceQueue(_device, _device->GetQueueFamilies()._graphics.value(), 0, &graphicsQueue);
	vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
	vkQueueWaitIdle(graphicsQueue);
	vkFreeCommandBuffers(_device, _graphicCommandPool, 1, &cmd);
}
*/

/*void CubemapManager::DebugRenderCubemaps()
{
	// 1. Create render instance in DEBUG mode
	CubemapRenderInstance instance(
		_cubemapSize,
		_format,
		ComputeType::DEBUGCUBEMAPS
	);F

	// 3. Build work range
	CubemapWorkRange range{};
	range.first = 0;
	range.count = static_cast<uint32_t>(_deformableMesh._vertices.rows());

	// 4. Execute
	//instance.DebugCubemaps(range);
}

void CubemapManager::DebugComputeCoordinates()
{
	// 1. Create render instance in DEBUG mode
	CubemapRenderInstance instance(
		_cubemapSize,
		_format,
		ComputeType::GPUSERIAL
	);

	// 3. Build work range
	CubemapWorkRange range{};
	range.first = 0;
	range.count = static_cast<uint32_t>(_deformableMesh._vertices.rows());

	// 4. Execute
	//instance.DebugPMVC(range);
}*/

void CubemapManager::CreateCommandPool(uint32_t queueFamilyIndex) {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_graphicCommandPool) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create command pool!");
	}
}

void CubemapManager::EnsureInteriorDistanceTable()
{
	// The memo hash covers only vertex counts and face topology on purpose: cage vertex
	// positions moving during deformation must not trigger a recompute, only topology
	// changes (adding/removing cage vertices or faces) invalidate the table.
	auto hashCombine = [](std::size_t& seed, std::size_t value)
	{
		seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
	};

	std::size_t hash = 0;
	hashCombine(hash, static_cast<std::size_t>(_cageMesh._vertices.rows()));
	hashCombine(hash, static_cast<std::size_t>(_deformableMesh._vertices.rows()));
	hashCombine(hash, static_cast<std::size_t>(_cageMesh._faces.rows()));
	for (Eigen::Index i = 0; i < _cageMesh._faces.size(); ++i)
	{
		hashCombine(hash, static_cast<std::size_t>(*(_cageMesh._faces.data() + i)));
	}

	if (hash == _interiorDistanceTableHash && _interiorDetours.size() > 0)
	{
		LOG_DEBUG("Reusing cached interior detour table ({} cage x {} mesh vertices).",
			_interiorDetours.rows(), _interiorDetours.cols());
		return;
	}

	LOG_INFO("Computing heat-method interior detour table ({} cage x {} mesh vertices).",
		_cageMesh._vertices.rows(), _deformableMesh._vertices.rows());
	const auto start = std::chrono::steady_clock::now();

	InteriorDistanceParams params;
	computeInteriorDistances(_cageMesh._vertices, _cageMesh._faces, _deformableMesh._vertices, params, _interiorDetours);

	// Store detours (interior minus Euclidean distance) instead of absolute distances:
	// the shader adds the interpolated detour on top of the per-pixel rasterized hit
	// distance, so wherever the cage is convex from the mesh vertex the weights reduce
	// exactly to the Euclidean variant. Interpolating absolute cage-vertex distances
	// would overestimate the hit distance mid-triangle on coarse cages.
	for (Eigen::Index mesh = 0; mesh < _interiorDetours.cols(); ++mesh)
	{
		const Eigen::Vector3d meshPosition = _deformableMesh._vertices.row(mesh).leftCols<3>();
		for (Eigen::Index cage = 0; cage < _interiorDetours.rows(); ++cage)
		{
			const Eigen::Vector3d cagePosition = _cageMesh._vertices.row(cage).leftCols<3>();
			const float euclidean = static_cast<float>((meshPosition - cagePosition).norm());
			_interiorDetours(cage, mesh) = std::max(0.f, _interiorDetours(cage, mesh) - euclidean);
		}
	}

	_interiorDistanceTableHash = hash;

	const auto end = std::chrono::steady_clock::now();
	LOG_INFO("Interior detour table computed in {} ms.",
		std::chrono::duration<double, std::milli>(end - start).count());
}

MeshOperationResult<MeshComputeWeightsOperationResult> CubemapManager::ComputeCoordinates(
	PMVCComputeType computeType,
	const bool useOffset,
	const uint32_t targetCount,
	const uint32_t hitCount,
	const bool omitNegative,
	const float alpha,
	const float beta,
	const float theta) {
	const bool useInteriorDistance) {
	assert(_device && "Device is null");
	assert(_descriptorPool && "DescriptorPool is null");
	assert(_resourceManager && "ResourceManager is null");
	assert(_renderPipelineManager && "RenderPipelineManager is null");
	assert(_matricesLayout && "MatricesLayout is null");
	assert(_depthHistoryLayout && "DepthHistoryLayout is null");
	CreateVertexBufferFromMesh();
	CreateIndexBufferFromMesh();

	// The offset variant weights by solid angle only, there is no distance in its
	// formula for the interior distance to replace.
	const Eigen::MatrixXf* interiorDetours = nullptr;
	if (useInteriorDistance && useOffset)
	{
		LOG_WARN("Interior distance PMVC is not available with the offset (PMVCO) variant, using the offset shader instead.");
	}
	else if (useInteriorDistance)
	{
		EnsureInteriorDistanceTable();
		if (_interiorDetours.size() > 0)
		{
			interiorDetours = &_interiorDetours;
		}
	}

	static constexpr const char* kComputeTypeNames[] = { "Serial", "Ring", "All", "Cpu" };
	LOG_INFO("PMVC compute: type={}, hitCount={}, omitNegative={}, offset={}, interiorDistance={}",
		kComputeTypeNames[static_cast<uint8_t>(computeType)],
		hitCount,
		omitNegative,
		useOffset,
		interiorDetours != nullptr);

	const auto instanceInitStart = std::chrono::steady_clock::now();
	CubemapRenderInstance instance(
		*this,
		_cubemapSize,
		_format,
		computeType,
		useOffset,
		targetCount,
		hitCount,
		omitNegative,
		alpha,
		beta,
		theta,
		interiorDetours,

		_device,
		_descriptorPool,
		_resourceManager,
		_renderPipelineManager,

		_cageMesh,
		_deformableMesh,

		_graphicCommandPool,
		_renderPass,
		_renderPassCpu,
		_cubemapPipelineHandle,
		_cubemapPipelineHandleCpu,
		_cubemapPipelineHitHandle,

		_matricesLayout,
		_depthHistoryLayout,

		_indexBuffer,
		_vertexBuffer
	);
	const auto instanceInitEnd = std::chrono::steady_clock::now();
	const auto instanceInitMs = std::chrono::duration<double, std::milli>(instanceInitEnd - instanceInitStart).count();
	CubemapWorkRange range{};
	range.first = 0;
	range.count = static_cast<uint32_t>(_deformableMesh._vertices.rows());
	Eigen::MatrixXd weights;
	instance.ComputeCoordinates(range, weights);
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
		std::move(psiQuad),
		instance.GetRenderMs(),
		instance.GetComputeMs(),
		instance.GetComputeTotalMs(),
		instance.GetTransferMs(),
		instanceInitMs};
}
