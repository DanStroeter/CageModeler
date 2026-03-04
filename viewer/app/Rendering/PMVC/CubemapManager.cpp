#include <Rendering/PMVC/CubemapManager.h>

#include <Rendering/PMVC/CubemapRenderInstance.h>
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
	//TODO cleanup
}

void CubemapManager::Initialize()
{
	_descriptorPool = CreateRenderResource<DescriptorPool>(_device);
	CreateCommandPool(_device->GetQueueFamilies()._graphics.value());
	CreateRenderPass(_format);
	CreateDescriptorSetLayouts();
	CreateCubemapRenderPipeline();
	//SphereWeightInitialization(512);
}

void CubemapManager::CreateRenderPass(VkFormat format) {
	VkAttachmentDescription colorAttachment{};
	colorAttachment.format = format;
	colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
	colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	colorAttachment.initialLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	colorAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

	VkAttachmentDescription depthAttachment{};
	depthAttachment.format = _device->FindDepthFormat();
	depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
	depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.initialLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	depthAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

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

	if (vkCreateRenderPass(_device, &renderPassInfo, nullptr, &_renderPass) != VK_SUCCESS) {
		throw std::runtime_error("failed to create Cubemap render pass!");
	}
}

void CubemapManager::CreateDescriptorSetLayouts() {
	VkDescriptorSetLayoutBinding layoutBinding{ };
	layoutBinding.binding = 0;
	layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	layoutBinding.descriptorCount = 1;
	layoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

	std::array<VkDescriptorSetLayoutBinding, 1> layoutBindings{ layoutBinding };

	_matricesLayout = _descriptorPool->CreateDescriptorSetLayout(layoutBindings);
}

void CubemapManager::CreateCubemapRenderPipeline()
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

MeshOperationResult<MeshComputeWeightsOperationResult> CubemapManager::ComputeCoordinates() {
	assert(_device && "Device is null");
	assert(_descriptorPool && "DescriptorPool is null");
	assert(_resourceManager && "ResourceManager is null");
	assert(_renderPipelineManager && "RenderPipelineManager is null");
	assert(_matricesLayout && "MatricesLayout is null");
	CreateVertexBufferFromMesh();
	CreateIndexBufferFromMesh();
	CubemapRenderInstance instance(
		*this,
		_cubemapSize,
		_format,
		ComputeType::GPUSERIAL,

		_device,
		_descriptorPool,
		_resourceManager,
		_renderPipelineManager,

		_cageMesh,
		_deformableMesh,

		_graphicCommandPool,
		_renderPass,
		_cubemapPipelineHandle,

		_matricesLayout,

		_indexBuffer,
		_vertexBuffer
	);
	CubemapWorkRange range{};
	range.first = 0;
	range.count = static_cast<uint32_t>(_deformableMesh._vertices.rows());
	Eigen::MatrixXd weights;
	instance.ComputeCoordinatesGPUSerial(range, weights);
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