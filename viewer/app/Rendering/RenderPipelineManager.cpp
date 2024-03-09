#include <Rendering/RenderPipelineManager.h>
#include <Rendering/Core/Device.h>
#include <Rendering/Utils/VulkanUtils.h>
#include <glm/mat4x4.hpp>

RenderPipelineManager::RenderPipelineManager(const RenderResourceRef<Device>& device)
	: _allocatedPipelines(MaximumNumberPipelines)
	, _device(device)
{
	std::fill(_allocatedPipelines.begin(), _allocatedPipelines.end(), false);
}

RenderPipelineManager::~RenderPipelineManager()
{
	ReleaseResource();
}

VkShaderModule RenderPipelineManager::CreateShaderModule(const std::vector<char>& shaderCode) const
{
	CHECK_VK_HANDLE(_device);

	VkShaderModuleCreateInfo createInfo { };
	createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	createInfo.codeSize = shaderCode.size();
	createInfo.pCode = reinterpret_cast<const uint32_t*>(shaderCode.data());

	VkShaderModule result;
	VK_CHECK(vkCreateShaderModule(_device, &createInfo, nullptr, &result));

	return result;
}

void RenderPipelineManager::ReleaseResource()
{
	// Destroy all pipelines and pipeline layouts.
	for (std::size_t i = 0; i < _allocatedPipelines.size(); ++i)
	{
		if (_allocatedPipelines[i])
		{
			vkDestroyPipeline(_device, _pipelines[i]._handle, nullptr);
			vkDestroyPipelineLayout(_device, _pipelines[i]._pipelineLayout, nullptr);
		}
	}

	std::fill(_allocatedPipelines.begin(), _allocatedPipelines.end(), false);
	std::fill(_pipelines.begin(), _pipelines.end(), PipelineObject());
}

PipelineHandle RenderPipelineManager::BuildGraphicsPipeline(const GraphicsPipelineObjectProxy& objectProxy)
{
	CHECK_VK_HANDLE(_device);

	std::vector<VkPipelineShaderStageCreateInfo> shaderStages;
	shaderStages.reserve(objectProxy._shaderModules.size());

	std::vector<VkShaderModule> loadedShaderModules;
	loadedShaderModules.reserve(objectProxy._shaderModules.size());

	const auto vertexModuleIt = objectProxy._shaderModules.find(ShaderModuleType::Vertex);
	if (vertexModuleIt != objectProxy._shaderModules.end())
	{
		const auto shaderCode = VulkanUtils::ReadBinaryFile(vertexModuleIt->second.string());
		const auto shaderModule = CreateShaderModule(shaderCode);
		loadedShaderModules.push_back(shaderModule);

		// Vertex shader stage.
		VkPipelineShaderStageCreateInfo vertShaderStage { };
		vertShaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		vertShaderStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
		vertShaderStage.module = shaderModule;
		vertShaderStage.pName = "main";

		const auto foundSpecializationInfo = objectProxy._specializationInfos.find(ShaderModuleType::Vertex);
		if (foundSpecializationInfo != objectProxy._specializationInfos.end())
		{
			vertShaderStage.pSpecializationInfo = &foundSpecializationInfo->second;
		}

		shaderStages.push_back(vertShaderStage);
	}

	const auto geomModuleIt = objectProxy._shaderModules.find(ShaderModuleType::Geometry);
	if (geomModuleIt != objectProxy._shaderModules.end())
	{
		const auto shaderCode = VulkanUtils::ReadBinaryFile(geomModuleIt->second.string());
		const auto shaderModule = CreateShaderModule(shaderCode);
		loadedShaderModules.push_back(shaderModule);

		// Vertex shader stage.
		VkPipelineShaderStageCreateInfo geomShaderStage { };
		geomShaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		geomShaderStage.stage = VK_SHADER_STAGE_GEOMETRY_BIT;
		geomShaderStage.module = shaderModule;
		geomShaderStage.pName = "main";

		const auto foundSpecializationInfo = objectProxy._specializationInfos.find(ShaderModuleType::Geometry);
		if (foundSpecializationInfo != objectProxy._specializationInfos.end())
		{
			geomShaderStage.pSpecializationInfo = &foundSpecializationInfo->second;
		}

		shaderStages.push_back(geomShaderStage);
	}

	const auto fragModuleIt = objectProxy._shaderModules.find(ShaderModuleType::Fragment);
	if (fragModuleIt != objectProxy._shaderModules.end())
	{
		const auto shaderCode = VulkanUtils::ReadBinaryFile(fragModuleIt->second.string());
		const auto shaderModule = CreateShaderModule(shaderCode);
		loadedShaderModules.push_back(shaderModule);

		// Vertex shader stage.
		VkPipelineShaderStageCreateInfo fragShaderStage { };
		fragShaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		fragShaderStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
		fragShaderStage.module = shaderModule;
		fragShaderStage.pName = "main";

		const auto foundSpecializationInfo = objectProxy._specializationInfos.find(ShaderModuleType::Fragment);
		if (foundSpecializationInfo != objectProxy._specializationInfos.end())
		{
			fragShaderStage.pSpecializationInfo = &foundSpecializationInfo->second;
		}

		shaderStages.push_back(fragShaderStage);
	}

	constexpr std::array dynamicStates {
		VK_DYNAMIC_STATE_VIEWPORT,
		VK_DYNAMIC_STATE_SCISSOR
	};

	// Dynamic state creation.
	VkPipelineDynamicStateCreateInfo dynamicState { };
	dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
	dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
	dynamicState.pDynamicStates = dynamicStates.data();

	// Vertex input and vertex shader bindings.
	VkPipelineVertexInputStateCreateInfo vertexInputState { };
	vertexInputState.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
	vertexInputState.vertexBindingDescriptionCount = static_cast<uint32_t>(objectProxy._vertexBindingDescriptions.size());
	vertexInputState.pVertexBindingDescriptions = objectProxy._vertexBindingDescriptions.data();
	vertexInputState.vertexAttributeDescriptionCount = static_cast<uint32_t>(objectProxy._vertexAttributeDescriptions.size());
	vertexInputState.pVertexAttributeDescriptions = objectProxy._vertexAttributeDescriptions.data();

	// Input assembly of the pipeline.
	VkPipelineInputAssemblyStateCreateInfo inputAssemblyState { };
	inputAssemblyState.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
	inputAssemblyState.topology = objectProxy._assemblyState._topology;
	inputAssemblyState.primitiveRestartEnable = VK_FALSE;

	VkPipelineViewportStateCreateInfo viewportState { };
	viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	viewportState.viewportCount = 1;
	viewportState.scissorCount = 1;

	// Create the rasterization state to change any rasterizationState details.
	VkPipelineRasterizationStateCreateInfo rasterizationState { };
	rasterizationState.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
	rasterizationState.depthClampEnable = VK_FALSE;
	rasterizationState.rasterizerDiscardEnable = VK_FALSE;
	rasterizationState.polygonMode = VK_POLYGON_MODE_FILL;
	rasterizationState.cullMode = VK_CULL_MODE_NONE;
	rasterizationState.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
	rasterizationState.depthBiasEnable = VK_FALSE;
	rasterizationState.lineWidth = 1.0f;

	const auto sampleCount = _device->GetMaximumUsableSampleCount();

	// Create the multisampling state.
	VkPipelineMultisampleStateCreateInfo multisampleState { };
	multisampleState.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	multisampleState.rasterizationSamples = sampleCount;
	multisampleState.sampleShadingEnable = VK_TRUE;
	multisampleState.minSampleShading = 0.2f;
	multisampleState.alphaToCoverageEnable = VK_TRUE;
	multisampleState.alphaToOneEnable = VK_FALSE;

	VkPipelineColorBlendStateCreateInfo colorBlendState { };
	colorBlendState.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	colorBlendState.logicOpEnable = VK_FALSE;
	colorBlendState.logicOp = VK_LOGIC_OP_COPY;
	colorBlendState.attachmentCount = objectProxy._colorBlendAttachments.size();
	colorBlendState.pAttachments = objectProxy._colorBlendAttachments.data();

	// Create the layout of the pipeline where we can specify uniform variables and such.
	VkPipelineLayoutCreateInfo pipelineLayoutInfo { };
	pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pipelineLayoutInfo.pNext = nullptr;
	pipelineLayoutInfo.setLayoutCount = objectProxy._descriptorSetLayouts.size();
	pipelineLayoutInfo.pSetLayouts = objectProxy._descriptorSetLayouts.data();
	pipelineLayoutInfo.pushConstantRangeCount = 0;
	pipelineLayoutInfo.pPushConstantRanges = nullptr;

	VkPipelineLayout pipelineLayout;
	VK_CHECK(vkCreatePipelineLayout(_device, &pipelineLayoutInfo, nullptr, &pipelineLayout));

	// Create the actual pipeline object.
	VkGraphicsPipelineCreateInfo graphicsPipelineCreateInfo { };
	graphicsPipelineCreateInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
	graphicsPipelineCreateInfo.stageCount = static_cast<uint32_t>(shaderStages.size());
	graphicsPipelineCreateInfo.pStages = shaderStages.data();
	graphicsPipelineCreateInfo.pVertexInputState = &vertexInputState;
	graphicsPipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
	graphicsPipelineCreateInfo.pTessellationState = nullptr;
	graphicsPipelineCreateInfo.pViewportState = &viewportState;
	graphicsPipelineCreateInfo.pRasterizationState = &rasterizationState;
	graphicsPipelineCreateInfo.pMultisampleState = &multisampleState;
	graphicsPipelineCreateInfo.pDepthStencilState = &objectProxy._depthStencil;
	graphicsPipelineCreateInfo.pColorBlendState = &colorBlendState;
	graphicsPipelineCreateInfo.pDynamicState = &dynamicState;
	graphicsPipelineCreateInfo.layout = pipelineLayout;
	graphicsPipelineCreateInfo.renderPass = objectProxy._renderPass;
	graphicsPipelineCreateInfo.subpass = objectProxy._subpassIndex;
	graphicsPipelineCreateInfo.basePipelineHandle = VK_NULL_HANDLE;
	graphicsPipelineCreateInfo.basePipelineIndex = -1;

	VkPipeline graphicsPipeline = VK_NULL_HANDLE;
	VK_CHECK(vkCreateGraphicsPipelines(_device, VK_NULL_HANDLE, 1, &graphicsPipelineCreateInfo, nullptr, &graphicsPipeline));

	for (const auto shaderModule : loadedShaderModules)
	{
		vkDestroyShaderModule(_device, shaderModule, nullptr);
	}

	// Get the next free index and mark it as set.
	const auto nextFreeIndex = std::find(_allocatedPipelines.begin(), _allocatedPipelines.end(), false) - _allocatedPipelines.begin();
	_allocatedPipelines[nextFreeIndex] = true;

	// Set the value of the pipeline object.
	_pipelines[nextFreeIndex] = PipelineObject(graphicsPipeline, pipelineLayout);

	return PipelineHandle(nextFreeIndex);
}

PipelineHandle GraphicsPipelineObjectProxy::Build() const
{
	if (const auto managerShaderPtr = _renderPipelineManager.lock())
	{
		return managerShaderPtr->BuildPipeline(*this);
	}

	return PipelineHandle();
}
