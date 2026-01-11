#pragma once
#include <Rendering/PMVC/CubemapRenderInstance.h>
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

/*
struct ComputePushConstants
{
	int   uNumCubemaps;
	int   uNumCageVertices;
	glm::ivec2 uFaceSize;
	int   uFacesPerCubemap;
	int uNumTriangles;
};
*/
struct CubemapPushConstants {
	glm::mat4 view;
	glm::mat4 proj;
};

struct CubemapWorkRange
{
	uint32_t first;   // start index in vertices list
	uint32_t count;   // how many cubemaps
};

struct CubemapVertex
{
	glm::vec3 _position;
	uint32_t _triangleID;
	uint32_t _vertexIndex;
};

class CubemapManager {
public:
	CubemapManager(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance, uint32_t cubemapSize, VkFormat format);
	~CubemapManager();
	void Initialize();

	void ComputeCoordinates(Eigen::MatrixXd& weights);
	void DebugRenderCubemaps();
	void DebugComputeCoordinates();

	void SetCage(const EigenMesh& mesh) { _cageMesh = mesh; }
	void SetMesh(const EigenMesh& mesh) { _deformableMesh = mesh; }

private: 
	//init functions:
	void CreateRenderPass(VkFormat format);
	void CreateCommandPool(uint32_t queueFamilyIndex);
	void CreateDescriptorSetLayouts();
	void CreateCubemapRenderPipeline();
	void CreateVertexBufferFromMesh();
	void CreateIndexBufferFromMesh();
	//void SphereWeightInitialization(uint32_t size);
	//float ComputeSphereWeight(int px, int py, int faceSize, int face);
	uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;
	//VkCommandBuffer BeginOneTimeCommands();
	//void EndOneTimeCommands(VkCommandBuffer cmd);

	//helper functions:
	float ComputeNearPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices);
	float ComputeFarPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices);
	glm::mat4 ComputeCubemapViewMatrix(uint32_t faceIndex, const glm::vec3& pos);
	std::vector<CubemapVertex> CreateCubemapVertexBuffer(const PolygonMesh& mesh);

	//resources:
	RenderResourceRef<Device> _device;
	RenderResourceRef<Instance> _instance;
	SubsystemPtr<RenderSubsystem> _renderSubsystem = nullptr;
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr;
	std::shared_ptr<RenderResourceManager> _resourceManager = nullptr;

	//geodata:
	EigenMesh _cageMesh;
	EigenMesh _deformableMesh;

	//pipeline
	VkCommandPool _graphicCommandPool = VK_NULL_HANDLE;
	VkRenderPass _renderPass = VK_NULL_HANDLE;
	PipelineHandle _cubemapPipelineHandle;

	//buffers
	MemoryMappedBuffer _indexBuffer;
	MemoryMappedBuffer _vertexBuffer;

	//descriptors
	RenderResourceRef<DescriptorPool> _descriptorPool;
	RenderResourceRef<DescriptorSetLayout> _matricesLayout;

	friend class CubemapRenderInstance; //TODO remove and fix dependencies!
	friend class GpuSerialComputeStrategy;

	uint32_t _cubemapSize;
	VkFormat _format;

	//VkImage        _solidAngleImage = VK_NULL_HANDLE;
	//VkDeviceMemory _solidAngleMemory = VK_NULL_HANDLE;
	//VkImageView    _solidAngleArrayView = VK_NULL_HANDLE;
	//VkSampler      _solidAngleSampler = VK_NULL_HANDLE;
};