#pragma once
#include <Rendering/PMVC/CubemapManager.h>
#include <Rendering/PMVC/IComputeStrategy.h>
#include <Rendering/PMVC/CubemapRenderInstance.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Core/Buffer.h>
#include <vulkan/vulkan.h>

#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>


class CubemapManager;
struct CubemapWorkRange;

struct CubemapRenderTarget
{
	VkImage        cubemapImage;
	VkDeviceMemory cubemapMemory;
	VkImageView    cubemapView;
	std::array<VkImageView, 6> faceViews;

	VkImage        depthImage;
	VkDeviceMemory depthMemory;
	std::array<VkImageView, 6> depthViews;

	std::array<VkFramebuffer, 6> framebuffers;
};

struct CubemapRenderUnit
{
	std::vector<CubemapRenderTarget> targets;

	std::array<VkCommandBuffer, 6> graphicsCmd;

	MemoryMappedBuffer matricesUBO;
	VkDescriptorSet    matricesDescriptorSet;
};

enum class ComputeType {
	CPU,
	GPUATOMIC,
	GPUSORT,
	DEBUGCUBEMAPS, // for debugging writes cupemaps to disk no compute
	GPUSERIAL 
	// TODO: implement if time leftover
};


struct CubemapMatricesUBO
{
	float invNumTriangles;
	float _pad[3];
};


class CubemapRenderInstance {
public:
	//CubemapRenderInstance() = delete;

	CubemapRenderInstance(CubemapManager& cubemapManager);
	CubemapRenderInstance(CubemapManager& cubemapManager, int cubemapSize, VkFormat format, ComputeType computeType);
	~CubemapRenderInstance();
	//CubemapRenderInstance(CubemapRenderInstance&) = default;
	//CubemapRenderInstance& operator=(CubemapRenderInstance&) = default;

	void DebugCubemaps(const CubemapWorkRange& range);
	void DebugPMVC(const CubemapWorkRange& range);
	void ComputeCoordinates(const CubemapWorkRange& range);

private:
	CubemapManager& _cubemapManager;

	std::unique_ptr<ICubemapComputeStrategy> _computeStage;
	
	//parameters 
	unsigned int _cubemapSize;
	VkFormat _format;
	ComputeType _computeType;

	//init functions
	void Initalize();
	CubemapRenderTarget CreateCubemapRenderTarget() const;
	CubemapRenderUnit CreateCubemapRenderUnit() const;

	void CreateCommandPool(uint32_t queueFamilyIndex);
	void UpdateMatricesDescriptorSet();
	void CreateSyncObjects();

	void RecordAndSubmitCubemapRender(uint32_t cubemapIdx, const glm::vec3& camPos, 
		CubemapRenderTarget& target, uint64_t signalValue); //TODO submit cubemap at once not in 6 parts
	std::vector<glm::vec3> BuildDeformableVertexPositions() const;

	//render resources
	CubemapRenderUnit _cubemapRenderUnit;
	VkCommandPool _graphicCommandPool;

	//sync objects
	std::vector<uint64_t> _slotDoneValue;
	uint64_t _timelineValue = 0;
	VkSemaphore _timeline;

	friend class GpuSerialComputeStrategy;
};

class SphereWeightCalculator {
public:
	VkImage        _solidAngleImage = VK_NULL_HANDLE;
	VkDeviceMemory _solidAngleMemory = VK_NULL_HANDLE;
	VkImageView    _solidAngleArrayView = VK_NULL_HANDLE;
	VkSampler      _solidAngleSampler = VK_NULL_HANDLE;

	void SphereWeightInitialization(uint32_t size, RenderResourceRef<Device> device, std::shared_ptr<RenderResourceManager> resourceManager, VkCommandPool commandPool);
	float ComputeSphereWeight(int px, int py, int faceSize, int face);
};

/*
class CubemapRenderer
{
	/*
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
	//MemoryMappedBuffer _indexBuffer;
	//MemoryMappedBuffer _vertexBuffer;
	//std::vector<VkCommandBuffer> _commandBuffers;
	
	//sync
	//VkFence _renderFence;
	*/

	//---------------------Debugging print(Keep to map approach by lipman to original method)
	/*
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

	VkImage        _solidAngleImage = VK_NULL_HANDLE;
	VkDeviceMemory _solidAngleMemory = VK_NULL_HANDLE;
	VkImageView    _solidAngleArrayView = VK_NULL_HANDLE;
	VkSampler      _solidAngleSampler = VK_NULL_HANDLE;
};*/
