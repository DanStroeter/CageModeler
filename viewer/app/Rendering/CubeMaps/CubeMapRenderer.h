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

struct CubemapMatricesUBO
{
	glm::mat4 proj;       // Projection matrix
	glm::mat4 views[6];   // View matrices for each cubemap face
};

class CubemapRenderer
{
public:
	CubemapRenderer() = delete;
	CubemapRenderer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance);
	void Initialize();
	void Deinitialize() {}; //TODO

	void ComputeCoordinates(); //Placeholder function to compute cubemap coordinates
	void RenderCubemaps();

	void SetCage(const std::shared_ptr<PolygonMesh> cage) { _cageMesh = cage; }
	void SetMesh(const std::shared_ptr<PolygonMesh> mesh) { _deformableMesh = mesh; }
private:
	CubemapMatricesUBO _computeMatricesUBO;
	SubsystemPtr<RenderSubsystem> _renderSubsystem;

	void CreateImageViews(uint32_t size, VkFormat format);
	void CreateRenderPass(VkFormat format);
	void CreateDescriptorSetLayouts();
	void CreateCubemapRenderPipeline();
	void CreateFramebuffer(uint32_t size);
	void CreateDepthImage(uint32_t size);
	void CreateCommandPool(uint32_t queueFamilyIndex);
	void CreateVertexBuffer(const std::vector<Vertex>& vertices);
	void CreateIndexBuffer(const std::vector<uint32_t>& indices);
	void CreateUniformBuffer(VkDeviceSize bufferSize);

	void AllocateMatricesDescriptorSet();
	void UpdateMatricesDescriptorSet();
	void CreateVertexBuffer(const std::vector < glm::vec3>& verticies);
	void CreateIndexBuffer(const std::vector<uint32_t>& indices);
	float ComputeNearPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices);
		float ComputeFarPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices);

	//createUBOBuffer?
	//void CreateComputePipeline();
	void CreateCommandBuffer();
	void CreateSyncObjects();
	glm::mat4 ComputeCubemapViewMatrix(uint32_t faceIndex, const glm::vec3& pos);


	PipelineHandle _cubemapPipelineHandle;
	//PipelineHandle _computePipelineHandle;

	std::shared_ptr<PolygonMesh> _cageMesh;
	std::shared_ptr<PolygonMesh> _deformableMesh;

	RenderResourceRef<Device> _device;
	RenderResourceRef<Instance> _instance;

	VkCommandPool _graphicCommandPool = VK_NULL_HANDLE;

	VkRenderPass _renderPass = VK_NULL_HANDLE;

	std::vector<VkImage> _cubemapImages = {};
	std::vector<std::array<VkImageView, 6>> _faceImageViews = {};
	std::vector<VkDeviceMemory> _cubemapImageMemory = {};
	std::vector<VkImageView> _cubemapViews = {};
	std::vector<VkFramebuffer> _faceFramebuffers = {};

	std::vector<VkImageView> _depthImageViews = {};
		VkDeviceMemory _depthImageMemory = VK_NULL_HANDLE;
		VkImage _depthImage	= VK_NULL_HANDLE;

	std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr;

	std::shared_ptr<RenderResourceManager> _resourceManager = nullptr; 

	RenderResourceRef<DescriptorPool> _descriptorPool;

	RenderResourceRef<DescriptorSetLayout> _matricesLayout;
	VkDescriptorSet _matricesDescriptorSet;
	MemoryMappedBuffer _matricesUniformBuffer;

	MemoryMappedBuffer _indexBuffer;
	MemoryMappedBuffer _vertexBuffer;

	std::vector<VkCommandBuffer> _commandBuffers;

	VkFence _renderFence;
};
