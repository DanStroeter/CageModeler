#pragma once

#include <Rendering/PMVC/CubemapManager.h>
#include <Rendering/PMVC/RingComputeStrategy.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/DescriptorPool.h>
#include <vulkan/vulkan.h>
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <Mesh/GeometryUtils.h>
#include <Mesh/Operations/MeshWeightsParams.h>
#include <memory>
#include <optional>

class CubemapManager;
struct CubemapWorkRange;

/**
 * Colour and depth of every peeled hit of one cubemap.
 *
 * Both are single images with 6 * hitCount array layers, where hit k occupies the layers
 * [6k, 6k + 6). Every layer stays resident until the vertex is done because the compute
 * pass normalizes the hits of a texel against each other and therefore needs all of them
 * at once. Rendering a layer leaves it in a shader-read layout, which is what both the
 * peeling pass of the next hit and the final compute dispatch sample.
 */
struct CubemapRenderTarget
{
	VkImage        cubemapImage = VK_NULL_HANDLE;
	VkDeviceMemory cubemapMemory = VK_NULL_HANDLE;
	/// Array view over every colour layer, sampled by the compute shader.
	VkImageView    cubemapView = VK_NULL_HANDLE;
	/// Colour attachment view per hit and face, indexed [hit][face].
	std::vector<std::array<VkImageView, 6>> faceViews;

	VkImage        depthImage = VK_NULL_HANDLE;
	VkDeviceMemory depthMemory = VK_NULL_HANDLE;
	/// Array view over every depth layer, sampled by the compute shader.
	VkImageView    depthView = VK_NULL_HANDLE;
	/// Depth attachment view per hit and face, indexed [hit][face].
	std::vector<std::array<VkImageView, 6>> depthViews;
	/// Array view over the six depth layers of one hit, sampled by the next hit to peel.
	std::vector<VkImageView> depthHitViews;

	std::vector<std::array<VkFramebuffer, 6>> framebuffers;

	/// One per hit; the set of hit k binds the depth of hit k - 1. The first hit does not
	/// peel, so its set is allocated but never bound.
	std::vector<VkDescriptorSet> depthHistoryDescriptorSets;
};

struct CubemapRenderUnit
{
	std::vector<CubemapRenderTarget> targets;

	// One set of face command buffers per target and hit, indexed [target][hit][face]. The
	// hits of a vertex are submitted without a CPU wait between them, so each needs its own
	// buffers; batches are separated by a timeline wait that makes them reusable.
	std::vector<std::vector<std::array<VkCommandBuffer, 6>>> graphicsCmdPerTarget;

	MemoryMappedBuffer matricesUBO;
	VkDescriptorSet    matricesDescriptorSet = VK_NULL_HANDLE;
};

struct CubemapMatricesUBO
{
	float invNumTriangles;
	float _pad[3];
};

/**
 * Renders the cage into a cubemap around every deformable mesh vertex and turns the
 * rendered hits into PMVC weights using the Ring compute pipeline: the render targets
 * form a ring of slots that are filled, dispatched and read back one after the other by
 * a pool of worker threads.
 *
 * Every PMVC variant runs through this single pipeline:
 *  - the hit count controls how many depth peeling layers are rendered per vertex; all of
 *    them are rendered before a single dispatch consumes them together, because the
 *    weights of the hits along one ray are normalized against each other,
 *  - skipping every second hit drops the entry hits from that normalization,
 *  - interior distances bias the split between the hits of a ray when a detour table is
 *    set,
 *  - the offset variant (PMVCO) weights by the solid angle alone.
 */
class CubemapRenderInstance
{
public:
	CubemapRenderInstance(
		uint32_t cubemapSize,
		VkFormat format,
		bool useOffset,
		uint32_t targetCount,
		uint32_t hitCount,
		bool skipEvenHits,
		const Eigen::MatrixXf* interiorDetours,

		RenderResourceRef<Device> device,
		RenderResourceRef<DescriptorPool> descriptorPool,
		std::shared_ptr<RenderResourceManager> resourceManager,
		std::shared_ptr<RenderPipelineManager> renderPipelineManager,

		EigenMesh cageMesh,
		EigenMesh deformableMesh,

		VkRenderPass renderPass,
		PipelineHandle cubemapPipelineHandle,
		PipelineHandle cubemapPipelineHitHandle,

		RenderResourceRef<DescriptorSetLayout> matricesLayout,
		RenderResourceRef<DescriptorSetLayout> depthHistoryLayout,

		MemoryMappedBuffer indexBuffer,
		MemoryMappedBuffer vertexBuffer
	);
	~CubemapRenderInstance();

	CubemapRenderInstance(const CubemapRenderInstance&) = delete;
	CubemapRenderInstance& operator=(const CubemapRenderInstance&) = delete;

	void ComputeCoordinates(const CubemapWorkRange& range, Eigen::MatrixXd& weights);

	[[nodiscard]] std::optional<double> GetRenderMs() const { return _renderMs; }
	[[nodiscard]] std::optional<double> GetComputeMs() const { return _computeMs; }
	[[nodiscard]] std::optional<double> GetComputeTotalMs() const { return _computeTotalMs; }
	[[nodiscard]] std::optional<double> GetTransferMs() const { return _transferMs; }

	void Cleanup();

private:
	//init functions
	void Initialize();
	CubemapRenderTarget CreateCubemapRenderTarget() const;
	CubemapRenderUnit CreateCubemapRenderUnit() const;

	void CreateCommandPool(uint32_t queueFamilyIndex);
	void UpdateMatricesDescriptorSet();
	void CreateSyncObjects();
	void UpdateProjectionPlanes();

	void RecordAndSubmitCubemapRender(uint32_t targetIndex,
		const glm::vec3& camPos,
		const CubemapRenderTarget& target,
		VkSemaphore timeline,
		uint64_t waitValue,
		uint64_t signalValue,
		uint32_t hitIndex);

	[[nodiscard]] std::vector<glm::vec3> BuildDeformableVertexPositions() const;

	[[nodiscard]] glm::mat4 ComputeCubemapViewMatrix(uint32_t faceIndex, const glm::vec3& pos) const;
	[[nodiscard]] uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;

	std::unique_ptr<RingComputeStrategy> _computeStage;

	//offset
	bool _pmvcUseOffset = false;

	// Interior detour table (rows = cage vertices, cols = mesh vertices, entries are
	// interior minus Euclidean distance); when set the compute strategy uses it to bias the
	// weight split along a ray. Owned by the CubemapManager, must outlive this instance.
	const Eigen::MatrixXf* _interiorDetours = nullptr;

	//parameters
	uint32_t _cubemapSize = PMVCSettings::kCubemapSize;
	uint32_t _targetCount = PMVCSettings::kRingTargetCount;
	uint32_t _hitCount = 1;

	/// Drops the entry (every second) hits from the per-ray weight split.
	bool _skipEvenHits = false;
	VkFormat _format = VK_FORMAT_R32G32B32A32_SFLOAT;

	std::optional<double> _renderMs;
	std::optional<double> _computeMs;
	std::optional<double> _computeTotalMs;
	std::optional<double> _transferMs;

	float _projectionNearPlane = 0.001f;
	float _projectionFarPlane = 1000.0f;

	//render resources
	CubemapRenderUnit _cubemapRenderUnit;
	VkCommandPool _graphicCommandPool = VK_NULL_HANDLE;

	//sync objects
	std::vector<uint64_t> _slotDoneValue;
	std::vector<VkSemaphore> _timelines = {};

	//from manager
	RenderResourceRef<Device> _device;
	RenderResourceRef<DescriptorPool> _descriptorPool;
	std::shared_ptr<RenderResourceManager> _resourceManager;
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager;
	EigenMesh _cageMesh;
	EigenMesh _deformableMesh;
	VkRenderPass _renderPass = VK_NULL_HANDLE;
	PipelineHandle _cubemapPipelineHandle;
	PipelineHandle _cubemapPipelineHitHandle;
	RenderResourceRef<DescriptorSetLayout> _matricesLayout;
	RenderResourceRef<DescriptorSetLayout> _depthHistoryLayout;
	MemoryMappedBuffer _indexBuffer;
	MemoryMappedBuffer _vertexBuffer;
	VkSampler _depthHistorySampler = VK_NULL_HANDLE;
};
