#pragma once

#define VK_NO_PROTOTYPES

#include <Rendering/Core/DescriptorSetLayout.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/Pipeline.h>
#include <Mesh/GeometryUtils.h>
#include <Rendering/Core/DescriptorPool.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/RenderPipelineManager.h>
#include <Rendering/PMVC/SphereWeightCalculator.h>

#include <Eigen/Core>

#include <array>
#include <cassert>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>
#include <vulkan/vulkan.h>
#include <glm/ext/vector_int2.hpp>

/**
 * Push constants of the unified PMVC compute shader (PMVCCompute.comp). The layout has
 * to match the shader's push constant block exactly.
 */
struct ComputePushConstants
{
	/// Bit flags of the uFlags field.
	enum Flags : int32_t
	{
		/// Bias the per-ray weight split towards hits reachable without a detour.
		InteriorDistance = 1 << 0,
		/// Weight by the solid angle alone, without any distance term (PMVCO).
		SolidAngleOnly = 1 << 1,
		/// Drop the entry (every second) hits from the per-ray weight split.
		SkipEvenHits = 1 << 2
	};

	glm::ivec2 uFaceSize { 0, 0 };
	int32_t uNumTriangles = 0;
	int32_t uNumCageVertices = 0;
	int32_t uMeshVertexIdx = 0;
	float uNearPlane = 1e-4f;
	float uFarPlane = 1.0f;
	int32_t uHitCount = 1;
	int32_t uFlags = 0;
};

/**
 * Every rendered hit of one cubemap, handed to the compute dispatch in one go. Both views
 * cover all layers of their image, where hit k occupies the layers [6k, 6k + 6): the
 * dispatch normalizes the hits of a ray against each other and therefore needs all of them.
 */
struct RenderedHit
{
	/// Barycentric coordinates + triangle index of every hit.
	VkImageView colorView = VK_NULL_HANDLE;

	/// Depth of every hit.
	VkImageView depthView = VK_NULL_HANDLE;
};

/**
 * Configuration for the interior-distance PMVC variant: when a detour table is set the
 * compute strategy lengthens the rasterized (Euclidean) hit distance by the barycentric
 * interpolation of the precomputed heat-method interior detours (interior distance minus
 * Euclidean distance, zero wherever the cage is convex from the mesh vertex, so the
 * variant reduces exactly to the Euclidean weighting on locally convex cages). The table
 * has one row per cage vertex and one column per deformable mesh vertex.
 */
struct InteriorDistanceSettings
{
	const Eigen::MatrixXf* detours = nullptr;
	float nearPlane = 1e-4f;
	float farPlane = 1.0f;

	[[nodiscard]] bool IsEnabled() const
	{
		return detours != nullptr && detours->size() > 0;
	}
};

/**
 * The one and only PMVC compute pipeline ("Ring"): a ring of render targets is filled by
 * the cubemap renderer and every slot is consumed by a compute dispatch that accumulates
 * the barycentric coordinates of the rendered hit, weighted by the solid angle of the
 * texel and (unless the offset variant is used) by its distance, into a per-slot lambda
 * buffer that is copied back and added to the result row of the mesh vertex.
 *
 * A mesh vertex may be rendered several times (depth peeling), each of those hits is
 * dispatched with its own weight, and two hits that have to meet per texel (the first and
 * second hit of the three-hit variant) are dispatched together, so multi-hit PMVC, the
 * three-hit variant and the interior distance variant all share this single pipeline.
 */
class RingComputeStrategy final
{
public:
	RingComputeStrategy(
		RenderResourceRef<Device> device,
		uint32_t computeQueueFamily,
		uint32_t faceSize,
		VkFormat format,
		RenderResourceRef<DescriptorPool> descriptorPool,
		const std::shared_ptr<RenderResourceManager>& resourceManager,
		const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		EigenMesh& cageMesh,
		EigenMesh& deformableMesh,
		bool offset,
		bool skipEvenHits,
		uint32_t hitCount,
		uint32_t targetCount,
		const InteriorDistanceSettings& interiorDistance)
		: _device(device)
		, _computeQueueFamily(computeQueueFamily)
		, _faceSize(faceSize)
		, _format(format)
		, _descriptorPool(descriptorPool)
		, _resourceManager(resourceManager)
		, _renderPipelineManager(renderPipelineManager)
		, _cageMesh(cageMesh)
		, _deformableMesh(deformableMesh)
		, _offset(offset)
		, _skipEvenHits(skipEvenHits)
		, _hitCount(hitCount == 0 ? 1u : hitCount)
		, _targetCount(targetCount == 0 ? 1 : static_cast<int>(targetCount))
		, _interiorDistance(interiorDistance)
	{
	}

	~RingComputeStrategy()
	{
		Cleanup();
	}

	RingComputeStrategy(const RingComputeStrategy&) = delete;
	RingComputeStrategy& operator=(const RingComputeStrategy&) = delete;

	[[nodiscard]] uint32_t RequiredRenderTargetCount() const;

	void Initialize();
	void Cleanup();

	/// Clears the accumulators of a mesh vertex before its first hit is dispatched.
	void BeginVertex(uint32_t deformableIndex);

	/**
	 * Records and submits the compute dispatch consuming every rendered hit of one vertex.
	 *
	 * @param hits Colour and depth of all peeled layers, which all have to still be
	 *             resident: the weights of the hits along a ray are normalized against each
	 *             other, so none of them can be accumulated on its own.
	 */
	void DispatchAfterRender(
		uint32_t deformableIndex,
		uint32_t slot,
		VkSemaphore timeline,
		uint64_t waitValue,
		uint64_t signalValue,
		const RenderedHit& hits);

	void SubmitReadbackCopy(
		uint32_t slot,
		VkSemaphore timeline,
		uint64_t waitValue,
		uint64_t signalValue);

	/// Waits for the readback copy of a slot and adds its contribution to the results.
	void AccumulateSlot(
		uint32_t deformableIndex,
		uint32_t slot,
		VkSemaphore timeline,
		uint64_t waitValue);

	/// Normalizes the accumulated weights and returns them.
	[[nodiscard]] Eigen::MatrixXd Readback();

private:
	void CreatePipelineAndLayouts();
	void AllocateResources();
	void UpdateComputeDescriptorSet(uint32_t slotIndex, const RenderedHit& hits);
	void CreateSampler();
	void CreateDepthSampler();
	void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size);

	/// Reports the linear reproduction residual and the smallest weight of the rest pose.
	void LogWeightDiagnostics();

	/// Reports the validity counters the shader accumulated over every ray.
	void LogRayDiagnostics();

	[[nodiscard]] bool UseInteriorDistance() const
	{
		// The offset variant weights by solid angle only, there is no distance in its
		// formula that the interior distance could replace.
		return !_offset && _interiorDistance.IsEnabled();
	}

	static constexpr uint32_t kDispatchGroupSize = 8;

	/// Slots of the diagnostics buffer, mirroring the kDiag* constants of the shader.
	static constexpr uint32_t kDiagnosticsSlotCount = 3;

	RenderResourceRef<Device> _device;
	uint32_t _computeQueueFamily = 0;
	uint32_t _faceSize = 32;
	VkFormat _format = VK_FORMAT_R32G32B32A32_SFLOAT;

	// Compute pipeline and descriptors
	PipelineHandle _computePipeline;
	RenderResourceRef<DescriptorSetLayout> _computeLayout;
	std::vector<VkDescriptorSet> _computeDescriptorSets = {};

	// Command resources
	VkCommandPool _computeCommandPool = VK_NULL_HANDLE;
	std::vector<VkCommandBuffer> _computeCommandBuffers = {};
	std::vector<VkCommandBuffer> _copyCommandBuffers = {};

	Buffer _vertexListBuffer;

	struct SlotBuffers
	{
		Buffer lambda;
		Buffer wsum;
		MemoryMappedBuffer lambdaStaging;
		MemoryMappedBuffer wsumStaging;
	};
	std::vector<SlotBuffers> _slots;

	// CPU-side result storage
	Eigen::MatrixXd _lambdaResults;
	std::vector<double> _wsumResults;

	EigenMesh& _cageMesh;
	EigenMesh& _deformableMesh;

	RenderResourceRef<DescriptorPool> _descriptorPool;
	std::shared_ptr<RenderResourceManager> _resourceManager;
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager;

	SphereWeightCalculator _sphereWeightCalculator;

	int _targetCount = 64;

	VkSampler _barySampler = VK_NULL_HANDLE;
	VkSampler _depthSampler = VK_NULL_HANDLE;

	bool _offset = false;

	/// Drops the entry (every second) hits from the per-ray weight split.
	bool _skipEvenHits = false;

	/// Peeled layers rendered per vertex, which bounds the hits the shader walks.
	uint32_t _hitCount = 1;

	// Interior-distance PMVC variant: the detour table is uploaded once and read by the
	// compute shader, which uses it to bias the weight split along a ray. The shader always
	// declares the binding, so a dummy buffer is bound when the variant is disabled.
	InteriorDistanceSettings _interiorDistance;
	Buffer _interiorDistanceBuffer;

	// Validity counters written by the shader: rays with an even number of crossings (a
	// watertightness failure), rays that filled every rendered layer (the hit count is too
	// low for the model) and rays that never crossed the cage.
	Buffer _diagnosticsBuffer;
	MemoryMappedBuffer _diagnosticsStaging;
};
