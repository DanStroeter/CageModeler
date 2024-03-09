#pragma once

#include <Rendering/Core/Device.h>
#include <Rendering/Core/Pipeline.h>

#include <span>
#include <filesystem>

class RenderPipelineManager;

struct InputAssemblyState
{
	InputAssemblyState(const VkPrimitiveTopology topology, const float pointSize = 1.0f)
		: _topology(topology)
		, _pointSize(pointSize)
	{ }

	VkPrimitiveTopology _topology;
	float _pointSize = 1.0f;
};

class GraphicsPipelineObjectProxy
{
public:
	GraphicsPipelineObjectProxy& SetVertexInputBindingDescriptions(const std::span<VkVertexInputBindingDescription> vertexBindingDescriptions)
	{
		_vertexBindingDescriptions.assign(vertexBindingDescriptions.begin(), vertexBindingDescriptions.end());

		return *this;
	}

	GraphicsPipelineObjectProxy& SetVertexInputAttributeDescriptions(const std::span<VkVertexInputAttributeDescription> vertexAttributeDescriptions)
	{
		_vertexAttributeDescriptions.assign(vertexAttributeDescriptions.begin(), vertexAttributeDescriptions.end());

		return *this;
	}

	GraphicsPipelineObjectProxy& SetDescriptorSetLayouts(const std::span<VkDescriptorSetLayout> setLayouts)
	{
		_descriptorSetLayouts.assign(setLayouts.begin(), setLayouts.end());

		return *this;
	}

	GraphicsPipelineObjectProxy& SetColorBlendAttachments(const std::span<VkPipelineColorBlendAttachmentState> colorBlendAttachments)
	{
		_colorBlendAttachments.assign(colorBlendAttachments.begin(), colorBlendAttachments.end());

		return *this;
	}

	GraphicsPipelineObjectProxy& SetRenderPass(const VkRenderPass renderPass)
	{
		_renderPass = renderPass;

		return *this;
	}

	GraphicsPipelineObjectProxy& SetSubpassIndex(const uint32_t subpassIndex)
	{
		_subpassIndex = subpassIndex;

		return *this;
	}

	GraphicsPipelineObjectProxy& SetDepthStencilState(const VkPipelineDepthStencilStateCreateInfo& depthStencil)
	{
		_depthStencil = depthStencil;

		return *this;
	}

	GraphicsPipelineObjectProxy& SetAssemblyState(const InputAssemblyState assemblyState)
	{
		_assemblyState = assemblyState;

		return *this;
	}

	GraphicsPipelineObjectProxy& SetShaderModule(const ShaderModuleType moduleType, const std::filesystem::path& shaderModulePath)
	{
		_shaderModules.insert(std::make_pair(moduleType, shaderModulePath));

		return *this;
	}

	GraphicsPipelineObjectProxy& SetShaderModuleSpecialization(const ShaderModuleType moduleType, const VkSpecializationInfo& specializationInfo)
	{
		_specializationInfos.insert(std::make_pair(moduleType, specializationInfo));

		return *this;
	}

	[[nodiscard]] PipelineHandle Build() const;

private:
	friend class RenderPipelineManager;

	explicit GraphicsPipelineObjectProxy(const std::shared_ptr<RenderPipelineManager>& pipelineFactory)
		: _renderPipelineManager(pipelineFactory)
	{ }

private:
	std::weak_ptr<RenderPipelineManager> _renderPipelineManager;
	std::vector<VkVertexInputBindingDescription> _vertexBindingDescriptions;
	std::vector<VkVertexInputAttributeDescription> _vertexAttributeDescriptions;
	std::vector<VkDescriptorSetLayout> _descriptorSetLayouts;
	std::vector<VkPipelineColorBlendAttachmentState> _colorBlendAttachments;
	std::unordered_map<ShaderModuleType, VkSpecializationInfo> _specializationInfos;
	VkPipelineDepthStencilStateCreateInfo _depthStencil { };
	InputAssemblyState _assemblyState { VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST };
	std::unordered_map<ShaderModuleType, std::filesystem::path> _shaderModules;
	uint32_t _subpassIndex = 0;
	VkRenderPass _renderPass = VK_NULL_HANDLE;
};

/**
 * A manager that takes care of the pipeline creation and ownership. Creating the pipeline uses a builder
 * to set the required settings and the result from the build is a pipeline handle. Handles
 */
class RenderPipelineManager : public std::enable_shared_from_this<RenderPipelineManager>
{
public:
	explicit RenderPipelineManager(const RenderResourceRef<Device>& device);
	~RenderPipelineManager();

	[[nodiscard]] auto BeginPipeline()
	{
		return GraphicsPipelineObjectProxy(shared_from_this());
	}

	template <typename PipelineObjectProxy>
	[[nodiscard]] PipelineHandle BuildPipeline(const PipelineObjectProxy& proxyObject)
	{
		return BuildGraphicsPipeline(proxyObject);
	}

	[[nodiscard]] const PipelineObject& GetPipelineObject(const PipelineHandle handle) const
	{
		return _pipelines[handle.GetIndex()];
	}

	void ReleaseResource();

private:
	[[nodiscard]] PipelineHandle BuildGraphicsPipeline(const GraphicsPipelineObjectProxy& objectProxy);
	[[nodiscard]] VkShaderModule CreateShaderModule(const std::vector<char>& shaderCode) const;

private:
	/// Maximum number of pipelines created.
	static constexpr auto MaximumNumberPipelines = 32;

	std::vector<bool> _allocatedPipelines;
	std::array<PipelineObject, MaximumNumberPipelines> _pipelines;
	RenderResourceRef<Device> _device;
};
