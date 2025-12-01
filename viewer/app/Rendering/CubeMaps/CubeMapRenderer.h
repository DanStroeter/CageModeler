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

class RenderSubsystem;

struct CubemapMatricesUBO
{
	glm::mat4 proj;       // Projection matrix
	glm::mat4 views[6];   // View matrices for each cubemap face
};

struct CubemapVertex
{
	glm::vec3 _position;  // Vertex position
	uint32_t _triangleID; // Triangle index
	uint32_t _vertexIndex; // 0,1,2 per triangle
};

class CubemapRenderer
{
public:
	CubemapRenderer() = delete;
	CubemapRenderer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance);
	
	void Initialize();
	void Deinitialize() {}; //TODO

	//void ComputeCoordinates(); //Placeholder function to compute cubemap coordinates
	void RenderCubemaps();

	void SetCage(const EigenMesh& mesh) {
		_cageMesh = mesh;
	}
	void SetMesh(const EigenMesh& mesh) {
		_deformableMesh = mesh;
	}
private:
	//void CreateComputePipeline();

	//init functions
	void CreateImageViews(uint32_t size, VkFormat format);
	void CreateRenderPass(VkFormat format);
	void CreateDescriptorSetLayouts();
	void CreateCubemapRenderPipeline();
	void CreateDepthImage(uint32_t size);
	void CreateFramebuffer(uint32_t size);
	void CreateCommandPool(uint32_t queueFamilyIndex);
	
	void CreateVertexBufferFromMesh();
	void CreateIndexBufferFromMesh();
	void CreateUniformBuffer(VkDeviceSize bufferSize);
	
	//void AllocateObjectDescriptorSet();
	//void UpdateObjectDescriptorSet();
	void AllocateMatricesDescriptorSet();
	void UpdateMatricesDescriptorSet();

	void CreateCommandBuffer();
	void CreateSyncObjects();

	//helper functions
	float ComputeNearPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices);
	float ComputeFarPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices);
	glm::mat4 ComputeCubemapViewMatrix(uint32_t faceIndex, const glm::vec3& pos);
	std::vector<CubemapVertex> CreateCubemapVertexBuffer(const PolygonMesh& mesh);

	//resources
	RenderResourceRef<Device> _device;
	RenderResourceRef<Instance> _instance;
	SubsystemPtr<RenderSubsystem> _renderSubsystem;
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr;
	std::shared_ptr<RenderResourceManager> _resourceManager = nullptr;

	//geodata
	EigenMesh _cageMesh;
	EigenMesh _deformableMesh;

	//pipeline
	VkCommandPool _graphicCommandPool = VK_NULL_HANDLE;
	VkRenderPass _renderPass = VK_NULL_HANDLE;
	PipelineHandle _cubemapPipelineHandle;
	//PipelineHandle _computePipelineHandle;

	//images
	std::vector<VkImage> _cubemapImages = {};
	std::vector<VkDeviceMemory> _cubemapImageMemory = {};
	std::vector<VkImageView> _cubemapViews = {};
	std::vector<std::array<VkImageView, 6>> _faceImageViews = {};
	std::vector<VkFramebuffer> _faceFramebuffers = {};

	//depth
	std::vector<VkImageView> _depthImageViews = {};
		VkDeviceMemory _depthImageMemory = VK_NULL_HANDLE;
		VkImage _depthImage	= VK_NULL_HANDLE;
	
	//descriptors
	RenderResourceRef<DescriptorPool> _descriptorPool;
	RenderResourceRef<DescriptorSetLayout> _matricesLayout;
	VkDescriptorSet _matricesDescriptorSet;
	MemoryMappedBuffer _matricesUniformBuffer;
	//RenderResourceRef<DescriptorSetLayout> _objectDataLayout;
	//VkDescriptorSet _objectDataDescriptorSet;
	//MemoryMappedBuffer _objectDataBuffer;

	//buffers
	MemoryMappedBuffer _indexBuffer;
	MemoryMappedBuffer _vertexBuffer;
	std::vector<VkCommandBuffer> _commandBuffers;
	
	//sync
	VkFence _renderFence;


	//---------------------Debugging print

};
