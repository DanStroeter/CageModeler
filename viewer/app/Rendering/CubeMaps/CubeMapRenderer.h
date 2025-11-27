#pragma once

#include <Rendering/Core/RenderProxy.h>
#include <Rendering/Core/Device.h>
#include <Rendering/Core/DescriptorPool.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/AlignedVector.h>
#include <Rendering/RenderPipelineManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/GeometryUtils.h>
#include <Editor/Light.h>
#include <Core/Subsystem.h>
#include <Rendering/RenderSubsystem.h>
#include <Eigen/Core>

struct ViewInfo;
class ScreenPass;
class PolygonMesh;
class RenderProxyCollector;
class RenderResourceManager;
class RenderCommandScheduler;

class CubeMapRenderer: public Subsystem
{
	DECLARE_SUBSYSTEM(CubeMapRenderer)
public:
	CubeMapRenderer() = delete;
	CubeMapRenderer(const RenderResourceRef<Device>& device,
		const RenderResourceRef<DescriptorPool>& descriptorPool,
		const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::shared_ptr<RenderProxyCollector>& renderProxyCollector,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
		const std::shared_ptr<RenderResourceManager>& resourceManager,
		const VkRenderPass& renderPass);

	void Initialize(const SubsystemsCollection& collection) override
	{
		// Get the editor's RenderSubsystem
		_renderSubsystem = GetDependencySubsystem<RenderSubsystem>(collection);

		// Reuse Vulkan instance and device
		//VkDevice device = _renderSubsystem->_device->GetDevice();
		//VkInstance instance = _renderSubsystem->_instance->GetInstance();
		//VkDescriptorPool descriptorPool = _renderSubsystem->_descriptorPool->GetPool();

		// Now create your cubemap-specific resources
		// e.g., pipeline layout
		VkPipelineLayoutCreateInfo layoutInfo{};
		layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
		layoutInfo.setLayoutCount = 0; // if you use descriptors, set them here
		layoutInfo.pushConstantRangeCount = 0;

		if (vkCreatePipelineLayout(_device, &layoutInfo, nullptr, &_cubemapPipelineLayout) != VK_SUCCESS)
		{
			throw std::runtime_error("Failed to create cubemap pipeline layout!");
		}

		// Create compute or graphics pipeline using device, pipeline layout, etc.
		// _cubemapPipeline = createCubemapPipeline(device, _cubemapPipelineLayout, ...);
	}

	void Update(const double deltaTime) override {}
	void Deinitialize() override;

	// Compute coordinates for a given mesh/cage
	//void ComputeCoordinates(const MeshData& mesh, const CageData& cage);

	[[nodiscard]] std::shared_ptr<PolygonMesh> AddCage(const Eigen::MatrixXd& vertices,
		const Eigen::MatrixXi& indices);

	[[nodiscard]] std::shared_ptr<PolygonMesh> AddMesh(const Eigen::MatrixXd& vertices,
		const Eigen::MatrixXi& indices);

	void RenderCubeMap();

private:
	SubsystemPtr<RenderSubsystem> _renderSubsystem;
	// Vulkan pipeline, descriptor sets, buffers, etc.

	void CreateScreenPasses();
	void CreateDescriptorSetLayouts();
	void CreateCubeMapRenderPipelines();
	void CreateComputePipeline();
	//void CreateBackgroundPipeline();
	//void CreateStaticMeshPipeline();
	//void CreateCagePipeline();
	//void CreateWireframePipelines();
	//void CreateGizmoPipeline();
	//void CreateViewportGridPipeline();
	void AllocateDescriptorSets();
	void CreateUniformBuffers();

	VkPipeline _cubemapPipeline = VK_NULL_HANDLE;
	VkPipelineLayout _cubemapPipelineLayout = VK_NULL_HANDLE;

	VkPipeline _copmutePipeline = VK_NULL_HANDLE;
	VkPipelineLayout computePipelineLayout = VK_NULL_HANDLE;

	VkDescriptorSet _descriptorSet = VK_NULL_HANDLE;
	SubsystemPtr<RenderSubsystem> _renderSubsystem;

	RenderResourceRef<Device> _device;

	/// The render command scheduler.
	//std::shared_ptr<RenderOffScreenCommandScheduler> _renderOffScreenCommandScheduler = nullptr;
	//just schedule the commands here in this class and keep it all together

	/// The proxy collector where we register render proxies.
	//std::shared_ptr<RenderProxyCollector> _renderProxyCollector = nullptr;
	//maybe can get the vertex data from here

	/// The render pipeline manager to add the scene graphics pipelines.
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr;

	/// Pointer to the resource manager.
	std::shared_ptr<RenderResourceManager> _resourceManager = nullptr;

	/// The Vulkan descriptor pool resource.
	RenderResourceRef<DescriptorPool> _descriptorPool;

	/// The render pass we use for rendering the entire scene. It is created by the render subsystem and passed to the editor.
	VkRenderPass _renderPass = VK_NULL_HANDLE; //make new one

	RenderResourceRef<DescriptorSetLayout> _matricesLayout;
	std::vector<VkDescriptorSet> _matricesDescriptorSets;
	std::vector<MemoryMappedBuffer> _matricesUniformBuffers;
	//more buffers that i need sbbo usw?

	RenderResourceRef<DescriptorSetLayout> _objectDataLayout;
	std::vector<VkDescriptorSet> _objectDataDescriptorSets;
	std::vector<MemoryMappedBuffer> _objectsDynamicUniformBuffers;
};
