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

class CubemapRenderer
{
public:
	CubemapRenderer() = delete;
	CubemapRenderer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance);
	void Initialize();
	void Deinitialize() {}; //TODO

	void ComputeCoordinates(); //Placeholder function to compute cubemap coordinates

	void SetCage(const std::shared_ptr<PolygonMesh> cage) { _cageMesh = cage; }
	void SetMesh(const std::shared_ptr<PolygonMesh> mesh) { _deformableMesh = mesh; }
private:
	SubsystemPtr<RenderSubsystem> _renderSubsystem;

	void CreateImageViews(uint32_t size, VkFormat format);
	void CreateRenderPass(VkFormat format);
	void CreateDescriptorSetLayouts();
	void CreateCubemapRenderPipeline();
	void CreateFramebuffer(uint32_t size);

	//void CreateComputePipeline();
	//void AllocateDescriptorSets();
	//void CreateUniformBuffers();
	//void CreateCommandPool(uint32_t queueFamilyIndex);
	//void CreateCommandBuffer();
	//void CreateSyncObjects();
	//void CreateDescriptorPoolSets();
	//void CreateDescriptorPool();
	//void CreateUniformBuffer(VkDeviceSize bufferSize);
	//void CreateIndexBuffer(const std::vector<uint32_t>& indices);
	//void CreateVertexBuffer(const std::vector<Vertex>& vertices);

	PipelineHandle _cubemapPipelineHandle;
	//PipelineHandle _computePipelineHandle;

	std::shared_ptr<PolygonMesh> _cageMesh;
	std::shared_ptr<PolygonMesh> _deformableMesh;

	RenderResourceRef<Device> _device;
	RenderResourceRef<Instance> _instance;

	// The render pass we use for rendering the entire scene. It is created by the render subsystem and passed to the editor.
	VkRenderPass _renderPass = VK_NULL_HANDLE;

	//VkDescriptorSetLayout _descriptorSetLayout = VK_NULL_HANDLE;

	std::vector<VkImage> _cubemapImages = {};
	std::vector<std::array<VkImageView, 6>> _faceImageViews = {};
	std::vector<VkDeviceMemory> _cubemapImageMemory = {};
	std::vector<VkImageView> _cubemapViews = {};
	std::vector<VkFramebuffer> _faceFramebuffers = {};

	// The render pipeline manager to add the scene graphics pipelines.
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr;

	// Pointer to the resource manager.
	std::shared_ptr<RenderResourceManager> _resourceManager = nullptr; 

	// The Vulkan descriptor pool resource.
	RenderResourceRef<DescriptorPool> _descriptorPool;

	//RenderResourceRef<DescriptorSetLayout> _uboLayout; not needed
	RenderResourceRef<DescriptorSetLayout> _matricesLayout;
	//VkDescriptorSet _matricesDescriptorSet;
	//MemoryMappedBuffer _matricesUniformBuffer;
};
