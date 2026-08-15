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

struct CubemapRenderTarget
{
	VkImage        cubemapImage = VK_NULL_HANDLE;
	VkDeviceMemory cubemapMemory = VK_NULL_HANDLE;
	VkImageView    cubemapView = VK_NULL_HANDLE;
	std::array<VkImageView, 6> faceViews{};

	// Two depth images ping-ponged by the depth peeling passes: hit N renders into
	// depth image N % 2 while sampling depth image (N + 1) % 2, which holds the depth of
	// hit N - 1, to discard everything in front of the previous layer.
	std::array<VkImage, 2> depthImages{};
	std::array<VkDeviceMemory, 2> depthMemories{};
	std::array<VkImageView, 2> depthViewsArray{};
	std::array<std::array<VkImageView, 6>, 2> depthViews{};
	std::array<std::array<VkFramebuffer, 6>, 2> framebuffers{};
	std::array<VkDescriptorSet, 2> depthHistoryDescriptorSets{};
};

struct CubemapRenderUnit
{
	std::vector<CubemapRenderTarget> targets;

	std::vector<std::array<VkCommandBuffer, 6>> graphicsCmdPerTarget;

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
 *  - the hit count controls how many depth peeling layers are rendered per vertex,
 *  - the negative (every second) hit contributions are always omitted, except for the
 *    three-hit variant where the first, second and third hit are weighted by alpha, beta
 *    and theta instead,
 *  - interior distances replace the Euclidean hit distance when a detour table is set,
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
		float alpha,
		float beta,
		float theta,
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

	/**
	 * @return The weight of the hit, 0 for the hits that are rendered only so the next
	 * layer can be peeled and do not contribute to the weights.
	 */
	[[nodiscard]] float HitWeight(uint32_t hitIndex) const;

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
	// interior minus Euclidean distance); when set the compute strategy lengthens the
	// rasterized hit distance by the interpolated detour. Owned by the CubemapManager,
	// must outlive this instance.
	const Eigen::MatrixXf* _interiorDetours = nullptr;

	//parameters
	uint32_t _cubemapSize = PMVCSettings::kCubemapSize;
	uint32_t _targetCount = PMVCSettings::kRingTargetCount;
	uint32_t _hitCount = 1;
	float _alpha = 1.0f;
	float _beta = -1.0f;
	float _theta = 1.0f;
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
