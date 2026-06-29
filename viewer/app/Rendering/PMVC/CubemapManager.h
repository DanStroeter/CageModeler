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
	int faceIndex;
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
	void Initialize(uint32_t cubemapSize);
	void Cleanup();

	MeshOperationResult<MeshComputeWeightsOperationResult> ComputeCoordinates(
		PMVCComputeType computeType,
		bool useOffset,
		uint32_t targetCount = 64,
		uint32_t hitCount = 3,
		bool omitNegative = true,
		float alpha = 1.0f,
		float beta = -1.0f,
		float theta = 1.0f);
	void DebugRenderCubemaps();
	void DebugComputeCoordinates();

	void SetCage(const EigenMesh& mesh) { _cageMesh = mesh; }
	void SetMesh(const EigenMesh& mesh) { _deformableMesh = mesh; }

private: 
	//init functions:
	VkRenderPass CreateRenderPass(VkFormat format, bool cpuTransfer);
	void CreateCommandPool(uint32_t queueFamilyIndex);
	void CreateDescriptorSetLayouts();
	PipelineHandle CreateCubemapRenderPipeline(bool cpuTransfer, bool depthPeelPass = false);
	void CreateVertexBufferFromMesh();
	void CreateIndexBufferFromMesh();

	//helper functions:
	float ComputeNearPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices);
	float ComputeFarPlane(const glm::vec3& camPos, const std::vector<glm::vec3>& vertices);
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

	bool init = false;

	//pipeline
	VkCommandPool _graphicCommandPool;
	VkRenderPass _renderPass;
	VkRenderPass _renderPassCpu;
	PipelineHandle _cubemapPipelineHandle;
	PipelineHandle _cubemapPipelineHandleCpu;
	PipelineHandle _cubemapPipelineHitHandle;

	//buffers
	MemoryMappedBuffer _indexBuffer;
	MemoryMappedBuffer _vertexBuffer;

	//descriptors
	RenderResourceRef < DescriptorPool> _descriptorPool;
	RenderResourceRef < DescriptorSetLayout> _matricesLayout;
	RenderResourceRef < DescriptorSetLayout> _depthHistoryLayout;

	//friend class CubemapRenderInstance; //TODO remove and fix dependencies!
	//friend class GpuSerialComputeStrategy;

	uint32_t _cubemapSize;
	VkFormat _format;

};
