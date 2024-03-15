#include <Rendering/Scene/SceneRenderer.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/ScreenPass.h>
#include <Editor/Light.h>

namespace
{
	/// 9 gizmos and 3 meshes.
	constexpr std::size_t TotalNumSceneObjects = 16;

	[[nodiscard]] WireframeRenderMode GetWireframeRenderModeFromSelectedTool(const std::size_t toolIndex)
	{
		switch (toolIndex)
		{
			case 0:
				return WireframeRenderMode::Points;
			case 1:
				return WireframeRenderMode::Edges;
			case 2:
				return WireframeRenderMode::Points | WireframeRenderMode::Edges;
			default:
				return WireframeRenderMode::None;
		}
	}
}

SceneRenderer::SceneRenderer(const RenderResourceRef<Device>& device,
							 const RenderResourceRef<DescriptorPool>& descriptorPool,
							 const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
							 const std::shared_ptr<RenderProxyCollector>& renderProxyCollector,
							 const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
							 const std::shared_ptr<RenderResourceManager>& resourceManager,
							 const VkRenderPass& renderPass)
	: _device(device)
	, _renderCommandScheduler(renderCommandScheduler)
	, _renderProxyCollector(renderProxyCollector)
	, _renderPipelineManager(renderPipelineManager)
	, _resourceManager(resourceManager)
	, _descriptorPool(descriptorPool)
	, _renderPass(renderPass)
	, _objectsBufferData(device)
{
	// Calculate required alignment based on minimum device offset alignment
	_objectsBufferDynamicAlignment = _device->GetMinimumMemoryAlignment<ModelInfo>();
	_objectsBufferData.Resize(TotalNumSceneObjects);
}

void SceneRenderer::Initialize()
{
	// Creates all descriptor set layouts.
	CreateDescriptorSetLayouts();

	// Creates, maps and updates the uniform buffers.
	CreateUniformBuffers();

	// Allocates and updates the descriptor sets.
	AllocateDescriptorSets();

	// Creates all graphics pipelines.
	CreateRenderPipelines();

	// Creates the grid and the gradient background.
	CreateScreenPasses();
}

void SceneRenderer::CreateScreenPasses()
{
	std::array<std::vector<VkDescriptorSet>, VulkanUtils::NumRenderFramesInFlight> descriptorSets { };
	for (std::size_t i = 0; i < descriptorSets.size(); ++i)
	{
		descriptorSets[i] = std::vector { _matricesDescriptorSets[i] };
	}

	const auto backgroundScreenPass = std::make_shared<ScreenPass>(MeshProxySolidPipeline {
		_backgroundPipelineHandle,
		_staticMeshInfluenceMapPipelineHandle,
		descriptorSets });
	_screenPasses.push_back(backgroundScreenPass);
	backgroundScreenPass->CollectRenderProxy(_renderProxyCollector, _device, _renderCommandScheduler);

	const auto gridScreenPass = std::make_shared<ScreenPass>(MeshProxySolidPipeline {
		_gridPipelineHandle,
		_staticMeshInfluenceMapPipelineHandle,
		descriptorSets });
	_screenPasses.push_back(gridScreenPass);
	gridScreenPass->CollectRenderProxy(_renderProxyCollector, _device, _renderCommandScheduler);
}

std::shared_ptr<PolygonMesh> SceneRenderer::AddCage(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& indices)
{
	std::array<std::vector<VkDescriptorSet>, VulkanUtils::NumRenderFramesInFlight> descriptorSets { };
	for (std::size_t i = 0; i < descriptorSets.size(); ++i)
	{
		descriptorSets[i] = std::vector { _matricesDescriptorSets[i], _objectDataDescriptorSets[i] };
	}

	const auto mesh = std::make_shared<PolygonMesh>(vertices,
		indices,
		MeshProxySolidPipeline {
			_cagePipelineHandle,
			_staticMeshInfluenceMapPipelineHandle,
			descriptorSets },
		MeshProxyWireframePipeline { _pointsPipelineHandle, _edgesPipelineHandle, _polysPipelineHandle, descriptorSets },
		true,
		WireframeRenderMode::Points | WireframeRenderMode::Edges | WireframeRenderMode::Polygons);
	mesh->CollectRenderProxy(_renderProxyCollector, _device, _renderCommandScheduler);

	return mesh;
}

std::shared_ptr<PolygonMesh> SceneRenderer::AddMesh(const Eigen::MatrixXd& vertices,
	const Eigen::MatrixXi& indices)
{
	std::array<std::vector<VkDescriptorSet>, VulkanUtils::NumRenderFramesInFlight> solidDescriptorSets { };
	for (std::size_t i = 0; i < solidDescriptorSets.size(); ++i)
	{
		solidDescriptorSets[i] = std::vector { _matricesDescriptorSets[i], _lightsDescriptorSets[i], _objectDataDescriptorSets[i] };
	}

	std::array<std::vector<VkDescriptorSet>, VulkanUtils::NumRenderFramesInFlight> wireframeDescriptorSets { };
	for (std::size_t i = 0; i < wireframeDescriptorSets.size(); ++i)
	{
		wireframeDescriptorSets[i] = std::vector { _matricesDescriptorSets[i], _objectDataDescriptorSets[i] };
	}

	const auto mesh = std::make_shared<PolygonMesh>(vertices,
		indices,
		MeshProxySolidPipeline {
			_staticMeshPipelineHandle,
			_staticMeshInfluenceMapPipelineHandle,
			solidDescriptorSets },
		MeshProxyWireframePipeline { _pointsPipelineHandle, _edgesPipelineHandle, _polysPipelineHandle, wireframeDescriptorSets },
		false,
		WireframeRenderMode::None);
	mesh->CollectRenderProxy(_renderProxyCollector, _device, _renderCommandScheduler);

	return mesh;
}

std::shared_ptr<PolygonMesh> SceneRenderer::AddGizmo(const MeshGeometry& geom)
{
	std::array<std::vector<VkDescriptorSet>, VulkanUtils::NumRenderFramesInFlight> solidDescriptorSets { };
	for (std::size_t i = 0; i < solidDescriptorSets.size(); ++i)
	{
		solidDescriptorSets[i] = std::vector { _matricesDescriptorSets[i], _objectDataDescriptorSets[i] };
	}

	const auto mesh = std::make_shared<PolygonMesh>(geom,
		MeshProxySolidPipeline {
			_gizmoPipelineHandle,
			_staticMeshInfluenceMapPipelineHandle,
			solidDescriptorSets },
		MeshProxyWireframePipeline { _pointsPipelineHandle, _edgesPipelineHandle, _polysPipelineHandle, solidDescriptorSets },
		false,
		WireframeRenderMode::None);
	mesh->CollectRenderProxy(_renderProxyCollector, _device, _renderCommandScheduler);

	return mesh;
}

void SceneRenderer::RemoveMesh(const std::shared_ptr<PolygonMesh>& mesh)
{
	mesh->DestroyRenderProxy(_renderProxyCollector);
}

void SceneRenderer::AddLightSource(const PointLightGPU& light)
{
	_lightSources.push_back(light);
}

void SceneRenderer::CreateResources()
{
	CreateRenderPipelines();
}

void SceneRenderer::Render(const double deltaTime, const uint32_t currentFrameIndex, const ViewInfo& viewInfo)
{
	// Destroy all render proxies that were previously marked.
	_renderProxyCollector->DestroyPendingRenderProxies();

	// First we recreate all render proxies that were scheduled previously.
	_renderProxyCollector->RecreateDirtyRenderProxies();

	FrameInfo frameInfo { };
	frameInfo._view = viewInfo._view;
	frameInfo._projection = viewInfo._projection;
	frameInfo._viewportSize = glm::vec2(viewInfo._renderSize.x, viewInfo._renderSize.y);

	// Copy the global matrices to the GPU buffer.
	memcpy(_matricesUniformBuffers[currentFrameIndex]._mappedData, &frameInfo, sizeof(FrameInfo));

	// Copy the lights data to the GPU buffer.
	memcpy(_lightsUniformBuffers[currentFrameIndex]._mappedData, _lightSources.data(), sizeof(PointLightGPU) * _lightSources.size());

	// Update the buffer data for each object.
	_renderProxyCollector->UpdateObjectsData(_objectsBufferData, viewInfo, _objectsDynamicUniformBuffers[currentFrameIndex]);

	// Render all proxies.
	_renderProxyCollector->Render(currentFrameIndex, deltaTime, viewInfo);
}

void SceneRenderer::CreateRenderPipelines()
{
	// Create all pipelines.
	CreateBackgroundPipeline();
	CreateViewportGridPipeline();
	CreateStaticMeshPipeline();
	CreateCagePipeline();
	CreateWireframePipelines();
	CreateGizmoPipeline();
}

void SceneRenderer::CreateDescriptorSetLayouts()
{
	// Create the descriptor sets.
	{
		VkDescriptorSetLayoutBinding layoutBinding { };
		layoutBinding.binding = 0;
		layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		layoutBinding.descriptorCount = 1;
		layoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

		std::array layoutBindings { layoutBinding };

		_matricesLayout = _descriptorPool->CreateDescriptorSetLayout(layoutBindings);
	}

	{
		VkDescriptorSetLayoutBinding layoutBinding { };
		layoutBinding.binding = 1;
		layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		layoutBinding.descriptorCount = 1;
		layoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

		std::array layoutBindings { layoutBinding };

		_lightsLayout = _descriptorPool->CreateDescriptorSetLayout(layoutBindings);
	}

	{
		VkDescriptorSetLayoutBinding layoutBinding { };
		layoutBinding.binding = 2;
		layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
		layoutBinding.descriptorCount = 1;
		layoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

		std::array layoutBindings { layoutBinding };

		_objectDataLayout = _descriptorPool->CreateDescriptorSetLayout(layoutBindings);
	}
}

void SceneRenderer::CreateBackgroundPipeline()
{
	// Our attachments will write to all color channels, but no blending is enabled.
	std::array<VkPipelineColorBlendAttachmentState, 1> colorBlendAttachments { };
	colorBlendAttachments[0].colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachments[0].blendEnable = VK_TRUE;
	colorBlendAttachments[0].srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	colorBlendAttachments[0].dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	colorBlendAttachments[0].colorBlendOp = VK_BLEND_OP_ADD;
	colorBlendAttachments[0].srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	colorBlendAttachments[0].dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	colorBlendAttachments[0].alphaBlendOp = VK_BLEND_OP_ADD;

	// Create the depth and stencil descriptions.
	VkPipelineDepthStencilStateCreateInfo depthStencil { };
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_FALSE;
	depthStencil.depthWriteEnable = VK_FALSE;
	depthStencil.stencilTestEnable = VK_FALSE;
	depthStencil.front = {};
	depthStencil.back = {};

	std::array descriptorSetLayouts { _matricesLayout->GetReference() };

	_backgroundPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetColorBlendAttachments(std::span(colorBlendAttachments))
		.SetDepthStencilState(depthStencil)
		.SetDescriptorSetLayouts(std::span(descriptorSetLayouts))
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Gradient.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/Gradient.frag.spv")
		.Build();
}

void SceneRenderer::CreateStaticMeshPipeline()
{
	auto vertexInputAttributeDescriptions = PolygonMeshRenderProxy::GetAttributeDescriptions();
	auto vertexInputBindingDescriptions = PolygonMeshRenderProxy::GetBindingDescription();

	// Our attachments will write to all color channels, but no blending is enabled.
	std::array<VkPipelineColorBlendAttachmentState, 1> colorBlendAttachments { };
	colorBlendAttachments[0].colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachments[0].blendEnable = VK_TRUE;
	colorBlendAttachments[0].srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	colorBlendAttachments[0].dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	colorBlendAttachments[0].colorBlendOp = VK_BLEND_OP_ADD;
	colorBlendAttachments[0].srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	colorBlendAttachments[0].dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	colorBlendAttachments[0].alphaBlendOp = VK_BLEND_OP_ADD;

	// Create the depth and stencil descriptions.
	VkPipelineDepthStencilStateCreateInfo depthStencil { };
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_TRUE;
	depthStencil.depthWriteEnable = VK_TRUE;
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
	depthStencil.depthBoundsTestEnable = VK_FALSE;
	depthStencil.minDepthBounds = 0.0f;
	depthStencil.maxDepthBounds = 1.0f;
	depthStencil.stencilTestEnable = VK_FALSE;
	depthStencil.front = {};
	depthStencil.back = {};

	std::array setLayouts { _matricesLayout->GetReference(),
		_lightsLayout->GetReference(),
		_objectDataLayout->GetReference() };

	uint32_t renderMode { 0 };
	std::array<VkSpecializationMapEntry, 1> specializationMapEntries { };

	// Map entry for the render mode to be used by the fragment shader
	specializationMapEntries[0].constantID = 0;
	specializationMapEntries[0].size = sizeof(renderMode);
	specializationMapEntries[0].offset = 0;

	// Prepare specialization info block for the shader stage
	VkSpecializationInfo specializationInfo { };
	specializationInfo.dataSize = sizeof(renderMode);
	specializationInfo.mapEntryCount = static_cast<uint32_t>(specializationMapEntries.size());
	specializationInfo.pMapEntries = specializationMapEntries.data();
	specializationInfo.pData = &renderMode;

	_staticMeshPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetVertexInputAttributeDescriptions(std::span(vertexInputAttributeDescriptions))
		.SetVertexInputBindingDescriptions(std::span(vertexInputBindingDescriptions))
		.SetDescriptorSetLayouts(std::span(setLayouts))
		.SetColorBlendAttachments(std::span(colorBlendAttachments))
		.SetDepthStencilState(depthStencil)
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/StaticMesh.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/StaticMesh.frag.spv")
		.SetShaderModuleSpecialization(ShaderModuleType::Fragment, specializationInfo)
		.Build();

	// Switch to influence map vertex color rendering, which will take a value of 1, before we create the next pipeline.
	renderMode = 1;

	_staticMeshInfluenceMapPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetVertexInputAttributeDescriptions(std::span(vertexInputAttributeDescriptions))
		.SetVertexInputBindingDescriptions(std::span(vertexInputBindingDescriptions))
		.SetDescriptorSetLayouts(std::span(setLayouts))
		.SetColorBlendAttachments(std::span(colorBlendAttachments))
		.SetDepthStencilState(depthStencil)
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/StaticMesh.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/StaticMesh.frag.spv")
		.SetShaderModuleSpecialization(ShaderModuleType::Fragment, specializationInfo)
		.Build();
}

void SceneRenderer::CreateCagePipeline()
{
	auto vertexInputAttributeDescriptions = PolygonMeshRenderProxy::GetAttributeDescriptions();
	auto vertexInputBindingDescriptions = PolygonMeshRenderProxy::GetBindingDescription();

	// Our attachments will write to all color channels, but no blending is enabled.
	std::array<VkPipelineColorBlendAttachmentState, 1> colorBlendAttachments { };
	colorBlendAttachments[0].colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachments[0].blendEnable = VK_TRUE;
	colorBlendAttachments[0].srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	colorBlendAttachments[0].dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	colorBlendAttachments[0].colorBlendOp = VK_BLEND_OP_ADD;
	colorBlendAttachments[0].srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	colorBlendAttachments[0].dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	colorBlendAttachments[0].alphaBlendOp = VK_BLEND_OP_ADD;

	// Create the depth and stencil descriptions.
	VkPipelineDepthStencilStateCreateInfo depthStencil { };
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_TRUE;
	depthStencil.depthWriteEnable = VK_TRUE;
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
	depthStencil.depthBoundsTestEnable = VK_FALSE;
	depthStencil.minDepthBounds = 0.0f;
	depthStencil.maxDepthBounds = 1.0f;
	depthStencil.stencilTestEnable = VK_FALSE;
	depthStencil.front = {};
	depthStencil.back = {};

	std::array setLayouts { _matricesLayout->GetReference(),
		_objectDataLayout->GetReference() };

	_cagePipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetVertexInputAttributeDescriptions(std::span(vertexInputAttributeDescriptions))
		.SetVertexInputBindingDescriptions(std::span(vertexInputBindingDescriptions))
		.SetDescriptorSetLayouts(std::span(setLayouts))
		.SetColorBlendAttachments(std::span(colorBlendAttachments))
		.SetDepthStencilState(depthStencil)
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/CageMesh.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/CageMesh.frag.spv")
		.Build();
}

void SceneRenderer::CreateWireframePipelines()
{
	auto vertexInputAttributeDescriptions = PolygonMeshRenderProxy::GetAttributeDescriptions();
	auto vertexInputBindingDescriptions = PolygonMeshRenderProxy::GetBindingDescription();

	// Our attachments will write to all color channels, but no blending is enabled.
	std::array<VkPipelineColorBlendAttachmentState, 1> colorBlendAttachments { };
	colorBlendAttachments[0].colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachments[0].blendEnable = VK_TRUE;
	colorBlendAttachments[0].srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	colorBlendAttachments[0].dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	colorBlendAttachments[0].colorBlendOp = VK_BLEND_OP_ADD;
	colorBlendAttachments[0].srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	colorBlendAttachments[0].dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	colorBlendAttachments[0].alphaBlendOp = VK_BLEND_OP_ADD;

	// Create the depth and stencil descriptions.
	VkPipelineDepthStencilStateCreateInfo depthStencil { };
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_TRUE;
	depthStencil.depthWriteEnable = VK_TRUE;
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
	depthStencil.depthBoundsTestEnable = VK_FALSE;
	depthStencil.minDepthBounds = 0.0f;
	depthStencil.maxDepthBounds = 1.0f;
	depthStencil.stencilTestEnable = VK_FALSE;
	depthStencil.front = {};
	depthStencil.back = {};

	std::array setLayouts { _matricesLayout->GetReference(), _objectDataLayout->GetReference() };

	_pointsPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetVertexInputAttributeDescriptions(std::span(vertexInputAttributeDescriptions))
		.SetVertexInputBindingDescriptions(std::span(vertexInputBindingDescriptions))
		.SetDescriptorSetLayouts(std::span(setLayouts))
		.SetColorBlendAttachments(std::span(colorBlendAttachments))
		.SetDepthStencilState(depthStencil)
		.SetAssemblyState(VK_PRIMITIVE_TOPOLOGY_POINT_LIST)
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Wireframe.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/Wireframe.frag.spv")
		.Build();

	_edgesPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetVertexInputAttributeDescriptions(std::span(vertexInputAttributeDescriptions))
		.SetVertexInputBindingDescriptions(std::span(vertexInputBindingDescriptions))
		.SetDescriptorSetLayouts(std::span(setLayouts))
		.SetColorBlendAttachments(std::span(colorBlendAttachments))
		.SetDepthStencilState(depthStencil)
		.SetAssemblyState(VK_PRIMITIVE_TOPOLOGY_LINE_LIST)
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Wireframe.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/Wireframe.frag.spv")
		.Build();

	_polysPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetVertexInputAttributeDescriptions(std::span(vertexInputAttributeDescriptions))
		.SetVertexInputBindingDescriptions(std::span(vertexInputBindingDescriptions))
		.SetDescriptorSetLayouts(std::span(setLayouts))
		.SetColorBlendAttachments(std::span(colorBlendAttachments))
		.SetDepthStencilState(depthStencil)
		.SetAssemblyState(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Wireframe.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/Wireframe.frag.spv")
		.Build();
}

void SceneRenderer::CreateGizmoPipeline()
{
	auto vertexInputAttributeDescriptions = PolygonMeshRenderProxy::GetAttributeDescriptions();
	auto vertexInputBindingDescriptions = PolygonMeshRenderProxy::GetBindingDescription();

	// Our attachments will write to all color channels, but no blending is enabled.
	std::array<VkPipelineColorBlendAttachmentState, 1> colorBlendAttachments { };
	colorBlendAttachments[0].colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachments[0].blendEnable = VK_TRUE;
	colorBlendAttachments[0].srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	colorBlendAttachments[0].dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	colorBlendAttachments[0].colorBlendOp = VK_BLEND_OP_ADD;
	colorBlendAttachments[0].srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	colorBlendAttachments[0].dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	colorBlendAttachments[0].alphaBlendOp = VK_BLEND_OP_ADD;

	// Create the depth and stencil descriptions.
	VkPipelineDepthStencilStateCreateInfo depthStencil { };
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_TRUE;
	depthStencil.depthWriteEnable = VK_TRUE;
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
	depthStencil.depthBoundsTestEnable = VK_FALSE;
	depthStencil.minDepthBounds = 0.0f;
	depthStencil.maxDepthBounds = 1.0f;
	depthStencil.stencilTestEnable = VK_FALSE;
	depthStencil.front = {};
	depthStencil.back = {};

	std::array setLayouts{ _matricesLayout->GetReference(), _objectDataLayout->GetReference() };

	_gizmoPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetVertexInputAttributeDescriptions(std::span(vertexInputAttributeDescriptions))
		.SetVertexInputBindingDescriptions(std::span(vertexInputBindingDescriptions))
		.SetDescriptorSetLayouts(std::span(setLayouts))
		.SetColorBlendAttachments(std::span(colorBlendAttachments))
		.SetDepthStencilState(depthStencil)
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Gizmo.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/Gizmo.frag.spv")
		.Build();
}

void SceneRenderer::CreateViewportGridPipeline()
{
	// Blend the fragments.
	VkPipelineColorBlendAttachmentState colorBlendAttachment { };
	colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	colorBlendAttachment.blendEnable = VK_TRUE;
	colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
	colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
	colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
	colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
	colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
	colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

	// Create the depth and stencil descriptions.
	VkPipelineDepthStencilStateCreateInfo depthStencil { };
	depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencil.depthTestEnable = VK_TRUE;
	depthStencil.depthWriteEnable = VK_TRUE;
	depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
	depthStencil.depthBoundsTestEnable = VK_FALSE;
	depthStencil.minDepthBounds = 0.0f;
	depthStencil.maxDepthBounds = 1.0f;
	depthStencil.stencilTestEnable = VK_FALSE;
	depthStencil.front = { };
	depthStencil.back = { };

	std::array descriptorSetLayouts { _matricesLayout->GetReference() };

	_gridPipelineHandle = _renderPipelineManager->BeginPipeline()
		.SetRenderPass(_renderPass)
		.SetColorBlendAttachments(std::span(&colorBlendAttachment, 1))
		.SetDepthStencilState(depthStencil)
		.SetDescriptorSetLayouts(std::span(descriptorSetLayouts))
		.SetSubpassIndex(0)
		.SetShaderModule(ShaderModuleType::Vertex, "assets/shaders/Grid.vert.spv")
		.SetShaderModule(ShaderModuleType::Fragment, "assets/shaders/Grid.frag.spv")
		.Build();
}

void SceneRenderer::CreateUniformBuffers()
{
	CHECK_VK_HANDLE(_device);

	_matricesUniformBuffers.resize(VulkanUtils::NumRenderFramesInFlight);
	_lightsUniformBuffers.resize(VulkanUtils::NumRenderFramesInFlight);
	_objectsDynamicUniformBuffers.resize(VulkanUtils::NumRenderFramesInFlight);

	for (std::size_t index = 0; index < VulkanUtils::NumRenderFramesInFlight; ++index)
	{
		FrameInfo frameInfo { };
		_matricesUniformBuffers[index] = _resourceManager->CreateBufferAndMapMemory(std::span(&frameInfo, 1),
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);

		if (!_lightSources.empty())
		{
			_lightsUniformBuffers[index] = _resourceManager->CreateBufferAndMapMemory(std::span(_lightSources),
				VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT);
		}

		_objectsDynamicUniformBuffers[index] = _resourceManager->CreateBufferAndMapMemoryAligned(std::span(_objectsBufferData.Data(), TotalNumSceneObjects),
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
	}
}

void SceneRenderer::AllocateDescriptorSets()
{
	CHECK_VK_HANDLE(_device);

	std::array<VkDescriptorSetLayout, VulkanUtils::NumRenderFramesInFlight> matricesDescriptorSetLayouts { };
	std::ranges::fill(matricesDescriptorSetLayouts, _matricesLayout);
	_matricesDescriptorSets = _descriptorPool->AllocateDescriptorSets(matricesDescriptorSetLayouts);

	std::array<VkDescriptorSetLayout, VulkanUtils::NumRenderFramesInFlight> objectDataDescriptorSetLayouts { };
	std::ranges::fill(objectDataDescriptorSetLayouts, _objectDataLayout);
	_objectDataDescriptorSets = _descriptorPool->AllocateDescriptorSets(objectDataDescriptorSetLayouts);

	std::array<VkDescriptorSetLayout, VulkanUtils::NumRenderFramesInFlight> lightsDescriptorSetLayouts { };
	std::ranges::fill(lightsDescriptorSetLayouts, _lightsLayout);
	_lightsDescriptorSets = _descriptorPool->AllocateDescriptorSets(lightsDescriptorSetLayouts);

	for (std::size_t index = 0; index < VulkanUtils::NumRenderFramesInFlight; ++index)
	{
		std::array<VkWriteDescriptorSet, 3> descriptorWrites { };

		VkDescriptorBufferInfo matricesBufferInfo { };
		matricesBufferInfo.buffer = _matricesUniformBuffers[index]._deviceBuffer;
		matricesBufferInfo.offset = 0;
		matricesBufferInfo.range = sizeof(FrameInfo);

		descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[0].dstSet = _matricesDescriptorSets[index];
		descriptorWrites[0].dstBinding = 0;
		descriptorWrites[0].dstArrayElement = 0;
		descriptorWrites[0].descriptorCount = 1;
		descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		descriptorWrites[0].pBufferInfo = &matricesBufferInfo;

		VkDescriptorBufferInfo lightsBufferInfo { };
		lightsBufferInfo.buffer = _lightsUniformBuffers[index]._deviceBuffer;
		lightsBufferInfo.offset = 0;
		lightsBufferInfo.range = sizeof(PointLightGPU) * _lightSources.size();

		descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[1].dstSet = _lightsDescriptorSets[index];
		descriptorWrites[1].dstBinding = 1;
		descriptorWrites[1].dstArrayElement = 0;
		descriptorWrites[1].descriptorCount = 1;
		descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		descriptorWrites[1].pBufferInfo = &lightsBufferInfo;

		VkDescriptorBufferInfo objectBufferInfo { };
		objectBufferInfo.buffer = _objectsDynamicUniformBuffers[index]._deviceBuffer;
		objectBufferInfo.offset = 0;
		objectBufferInfo.range = _objectsBufferDynamicAlignment;

		descriptorWrites[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		descriptorWrites[2].dstSet = _objectDataDescriptorSets[index];
		descriptorWrites[2].dstBinding = 2;
		descriptorWrites[2].dstArrayElement = 0;
		descriptorWrites[2].descriptorCount = 1;
		descriptorWrites[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
		descriptorWrites[2].pBufferInfo = &objectBufferInfo;

		vkUpdateDescriptorSets(_device, static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
	}
}
