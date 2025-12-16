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

struct ComputePushConstants
{
	int   uNumCubemaps;
	int   uNumCageVertices;
	glm::ivec2 uFaceSize;
	int   uFacesPerCubemap;
	int uNumTriangles;
};


struct CubemapMatricesUBO
{
	glm::mat4 proj;       // Projection matrix
	glm::mat4 view;   // View matrices for each cubemap face
	float invNumTriangles;
	float _pad[3];
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

	void RenderCubemaps();

	void SetCage(const EigenMesh& mesh) {
		_cageMesh = mesh;
	}
	void SetMesh(const EigenMesh& mesh) {
		_deformableMesh = mesh;
	}
private:
	VkImage        _solidAngleImage = VK_NULL_HANDLE;
	VkDeviceMemory _solidAngleMemory = VK_NULL_HANDLE;
	VkImageView    _solidAngleArrayView = VK_NULL_HANDLE;
	VkSampler      _solidAngleSampler = VK_NULL_HANDLE;

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

	//buffers
	MemoryMappedBuffer _indexBuffer;
	MemoryMappedBuffer _vertexBuffer;
	std::vector<VkCommandBuffer> _commandBuffers;
	
	//sync
	VkFence _renderFence;


	//---------------------Debugging print(Keep to map approach by lipman to original method)
	
	VkCommandBuffer BeginOneTimeCommands();
	void ExportCubemapAsVerticalStrip(const std::string& filename);
	void TransitionImageToTransferSrc(VkCommandBuffer cmd, VkImage image);
	void EndOneTimeCommands(VkCommandBuffer cmd);
	uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);

	//----------------------Compute Stage
	PipelineHandle _computePipelineHandle;
	RenderResourceRef<DescriptorSetLayout> _computeLayout;
	VkDescriptorSet _computeDescriptorSet;
	Buffer _lambdaBuffer;
	Buffer _wsumBuffer;

	MemoryMappedBuffer _lambdaStagingBuffer;
	MemoryMappedBuffer _wsumStagingBuffer;
	Buffer _vertexListBuffer;

	std::vector<std::vector<float>> _lambdaResults; //cpu readback storage
	std::vector<float> _wsumResults;   //cpu readback storage

	VkCommandBuffer _computeCommandBuffer;
	VkSampler _sampler;
	VkImageView _baryTexImageView = VK_NULL_HANDLE;
	std::vector<VkImageView> _baryTexImageViews;

	VkCommandPool _computeCommandPool;

	void CreateComputeDescriptorSetLayout();
	void CreateComputeBuffers();
	void AllocateComputeDescriptorSet();
	void UpdateComputeDescriptorSet(); 
	void CreateComputePipeline();
	void CreateComputeCommandBuffer();
	void ComputeCoordinates(uint32_t cubeIndex);
	void CreateSampler();
	void CreateCubemapImageViews();
	void CreateComputeCommandPool(uint32_t queueFamilyIndex);

	void ReadbackCompute(uint32_t cubeIndex);
	void storeLambdaForVertex(uint32_t cubeIndex, const float* lambdaCPU);
	void storeWsumForVertex(uint32_t cubeIndex, const float* wsumCPU);

	void InsertImageMemoryBarrierToGeneral(VkCommandBuffer cmd, VkImage image, VkImageSubresourceRange subresourceRange);
	float ComputeSphereWeight(int px, int py, int faceSize);
	void CartesianToSpherical(float x, float y, float z, float& theta, float& phi);
	void CopyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);
	void WriteWeightsToFile(const std::string& filename);

	void SphereWeightInitialization(uint32_t size);
	glm::vec3 CubeFaceDir(int face, float x, float y);
};
