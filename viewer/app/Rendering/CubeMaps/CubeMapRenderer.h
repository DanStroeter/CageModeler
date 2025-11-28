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

//struct ViewInfo;
//class ScreenPass;
//class PolygonMesh;
//class RenderProxyCollector;
//class RenderResourceManager;
//class RenderCommandScheduler;

class CubemapRenderer: public Subsystem
{
	DECLARE_SUBSYSTEM(CubemapRenderer)
public:
	CubemapRenderer() = delete;
	CubemapRenderer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::shared_ptr<RenderResourceManager>& resourceManager);
	void Initialize(const SubsystemsCollection& collection) override;
	void Update(const double deltaTime) override {}
	void Deinitialize() override;
	
	//[[nodiscard]] std::shared_ptr<PolygonMesh> AddCage(const Eigen::MatrixXd& vertices,
		//const Eigen::MatrixXi& indices);
	//[[nodiscard]] std::shared_ptr<PolygonMesh> AddMesh(const Eigen::MatrixXd& vertices,
		//const Eigen::MatrixXi& indices);
	//void RemoveMesh(const std::shared_ptr<PolygonMesh>& mesh);

	//void ComputeCoordinates();//pass cage and mesh, return coordinates
private:
	SubsystemPtr<RenderSubsystem> _renderSubsystem;

	void CreateImageViews(uint32_t size, VkFormat format);
	
	//void CreateScreenPasses();
	//void CreateDescriptorSetLayouts();
	//void CreateCubeMapRenderPipeline();
	//void CreateComputePipeline();
	//void AllocateDescriptorSets();
	//void CreateUniformBuffers();
	//void CreateRenderPass(VkFormat format);
	//void CreateDescriptorSetLayout();
	//void CreateFramebuffer(uint32_t size);
	//void CreateCommandPool(uint32_t queueFamilyIndex);
	//void CreateCommandBuffer();
	//void CreateSyncObjects();
	//void CreateDescriptorPoolSets();
	//void CreateDescriptorPool();
	//void CreateUniformBuffer(VkDeviceSize bufferSize);
	//void CreateIndexBuffer(const std::vector<uint32_t>& indices);
	//void CreateVertexBuffer(const std::vector<Vertex>& vertices);

	VkPipeline _cubemapPipeline = VK_NULL_HANDLE;
	VkPipelineLayout _cubemapPipelineLayout = VK_NULL_HANDLE;

	VkPipeline _copmutePipeline = VK_NULL_HANDLE;
	VkPipelineLayout computePipelineLayout = VK_NULL_HANDLE;

	//VkDescriptorSet _descriptorSet = VK_NULL_HANDLE;

	RenderResourceRef<Device> _device;

	std::vector<RenderResourceRef<VkImage>> _cubemapImages = {};
	std::vector<RenderResourceRef<VkImageView>[6]> _faceImageViews = {};
	std::vector<RenderResourceRef<VkDeviceMemory>> _cubemapImageMemory = {};
	std::vector<RenderResourceRef<VkImageView>> _cubemapViews = {};

	/// The render pipeline manager to add the scene graphics pipelines.
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr; //reuse from scene

	/// Pointer to the resource manager.
	std::shared_ptr<RenderResourceManager> _resourceManager = nullptr; //reuse from scene

	/// The Vulkan descriptor pool resource.
	//RenderResourceRef<DescriptorPool> _descriptorPool; //make new one 

	/// The render pass we use for rendering the entire scene. It is created by the render subsystem and passed to the editor.
	//VkRenderPass _renderPass = VK_NULL_HANDLE; //make new one

	//RenderResourceRef<DescriptorSetLayout> _matricesLayout;
	//std::vector<VkDescriptorSet> _matricesDescriptorSets;
	//std::vector<MemoryMappedBuffer> _matricesUniformBuffers;
	//more buffers that i need
};
