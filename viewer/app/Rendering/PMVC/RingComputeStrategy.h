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
		/// Lengthen the rasterized hit distance by the interpolated interior detour.
		InteriorDistance = 1 << 0,
		/// Weight by the solid angle alone, without any distance term (PMVCO).
		SolidAngleOnly = 1 << 1
	};

	glm::ivec2 uFaceSize { 0, 0 };
	int32_t uNumTriangles = 0;
	int32_t uNumCageVertices = 0;
	int32_t uMeshVertexIdx = 0;
	float uNearPlane = 1e-4f;
	float uFarPlane = 1.0f;
	float uHitWeightA = 1.0f;
	float uHitWeightB = 0.0f;
	int32_t uFlags = 0;
};

/**
 * One rendered hit handed to the compute dispatch. Two of them are combined per texel by
 * the three-hit variant, where the second one is weighted by a negative beta and is
 * therefore subtracted from the first one on the ray it belongs to.
 */
struct RenderedHit
{
	/// Barycentric coordinates + triangle index of the hit.
	VkImageView colorView = VK_NULL_HANDLE;

	/// Depth layer the hit was rendered into.
	VkImageView depthView = VK_NULL_HANDLE;

	/// Scales the contribution of the hit (alpha / beta / theta, or 1). May be negative.
	float weight = 1.0f;
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
	 * Records and submits the compute dispatch consuming one or two rendered hits.
	 *
	 * @param first The hit to accumulate.
	 * @param second An optional second hit of the same cubemap, accumulated in the same
	 *               invocation so its (negative) contribution meets the first one per
	 *               texel. Both hits have to still be resident when this is called.
	 */
	void DispatchAfterRender(
		uint32_t deformableIndex,
		uint32_t slot,
		VkSemaphore timeline,
		uint64_t waitValue,
		uint64_t signalValue,
		const RenderedHit& first,
		const std::optional<RenderedHit>& second = std::nullopt);

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
	void UpdateComputeDescriptorSet(uint32_t slotIndex, const RenderedHit& first, const RenderedHit& second);
	void CreateSampler();
	void CreateDepthSampler();
	void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size);

	[[nodiscard]] bool UseInteriorDistance() const
	{
		// The offset variant weights by solid angle only, there is no distance in its
		// formula that the interior distance could replace.
		return !_offset && _interiorDistance.IsEnabled();
	}

	static constexpr uint32_t kDispatchGroupSize = 8;

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

	// Interior-distance PMVC variant: the detour table is uploaded once and read by the
	// compute shader on top of the rasterized depth. The shader always declares the
	// binding, so a dummy buffer is bound when the variant is disabled.
	InteriorDistanceSettings _interiorDistance;
	Buffer _interiorDistanceBuffer;
};
