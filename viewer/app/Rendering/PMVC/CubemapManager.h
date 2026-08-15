#pragma once

#include <Rendering/Core/RenderProxy.h>
#include <Rendering/Core/Device.h>
#include <Rendering/Core/DescriptorPool.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/AlignedVector.h>
#include <Rendering/RenderPipelineManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/GeometryUtils.h>
#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>
#include <Editor/Light.h>
#include <Core/Subsystem.h>
#include <Rendering/RenderSubsystem.h>
#include <Eigen/Core>

class RenderSubsystem;
class PolygonMesh;

struct CubemapPushConstants
{
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

/**
 * Owns the resources shared by all PMVC weight computations (render pass, cubemap
 * pipelines, cage buffers and the memoized interior distance table) and runs a single
 * computation through the Ring pipeline.
 */
class CubemapManager
{
public:
	CubemapManager(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::shared_ptr<RenderResourceManager>& resourceManager,
		RenderResourceRef<Device> device,
		RenderResourceRef<Instance> instance,
		uint32_t cubemapSize,
		VkFormat format);
	~CubemapManager();

	void Initialize(uint32_t cubemapSize);
	void Cleanup();

	/**
	 * Computes the PMVC weights of the deformable mesh inside the cage.
	 *
	 * @param useOffset Use the offset variant (PMVCO), which weights by the solid angle
	 *                  of the texel alone.
	 * @param hitCount Number of depth peeling layers rendered per mesh vertex. The
	 *                 negative (every second) hit contributions are omitted, except for
	 *                 a hit count of three where alpha, beta and theta are applied to the
	 *                 first, second and third hit.
	 * @param useInteriorDistance Weight by heat-method interior distances instead of the
	 *                            rasterized Euclidean depth.
	 */
	MeshOperationResult<MeshComputeWeightsOperationResult> ComputeCoordinates(
		bool useOffset,
		uint32_t hitCount = 1,
		float alpha = 1.0f,
		float beta = -1.0f,
		float theta = 1.0f,
		bool useInteriorDistance = false);

	void SetCage(const EigenMesh& mesh) { _cageMesh = mesh; }
	void SetMesh(const EigenMesh& mesh) { _deformableMesh = mesh; }

private:
	//init functions:
	VkRenderPass CreateRenderPass(VkFormat format);
	void CreateCommandPool(uint32_t queueFamilyIndex);
	void CreateDescriptorSetLayouts();
	PipelineHandle CreateCubemapRenderPipeline(bool depthPeelPass = false);
	void CreateVertexBufferFromMesh();
	void CreateIndexBufferFromMesh();

	// Heat-method interior detour table (rows = cage vertices, cols = mesh vertices):
	// each entry is interior distance minus Euclidean distance (>= 0), zero wherever the
	// cage is convex from the mesh vertex. Computed lazily when interior-distance PMVC is
	// requested and memoized on the cage topology: cage vertex positions moving during
	// deformation do not invalidate it, only topology changes (vertex/face count, face
	// indices) trigger a recompute.
	void EnsureInteriorDistanceTable();

	//resources:
	RenderResourceRef<Device> _device;
	RenderResourceRef<Instance> _instance;
	SubsystemPtr<RenderSubsystem> _renderSubsystem = nullptr;
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr;
	std::shared_ptr<RenderResourceManager> _resourceManager = nullptr;

	//geodata:
	EigenMesh _cageMesh;
	EigenMesh _deformableMesh;

	Eigen::MatrixXf _interiorDetours;
	std::size_t _interiorDistanceTableHash = 0;

	bool init = false;

	//pipeline
	VkCommandPool _graphicCommandPool = VK_NULL_HANDLE;
	VkRenderPass _renderPass = VK_NULL_HANDLE;
	PipelineHandle _cubemapPipelineHandle;
	PipelineHandle _cubemapPipelineHitHandle;

	//buffers
	MemoryMappedBuffer _indexBuffer;
	MemoryMappedBuffer _vertexBuffer;

	//descriptors
	RenderResourceRef<DescriptorPool> _descriptorPool;
	RenderResourceRef<DescriptorSetLayout> _matricesLayout;
	RenderResourceRef<DescriptorSetLayout> _depthHistoryLayout;

	uint32_t _cubemapSize;
	VkFormat _format;
};
