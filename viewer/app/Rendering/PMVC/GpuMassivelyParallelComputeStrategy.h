#pragma once

#include <Rendering/PMVC/IComputeStrategy.h>
#include <Rendering/Core/DescriptorSetLayout.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/Pipeline.h>
#include <Mesh/GeometryUtils.h>
#include <Rendering/Core/DescriptorPool.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/PMVC/CubemapRenderInstance.h>
#include <Rendering/PMVC/SphereWeightCalculator.h>


class GpuMPComputeStrategy final : public ICubemapComputeStrategy
{
public:
    GpuMPComputeStrategy(
        RenderResourceRef<Device> device,
        uint32_t transferQueueFamily,
        uint32_t faceSize,
        VkFormat format,
        RenderResourceRef<DescriptorPool> descriptorPool,
        const std::shared_ptr<RenderResourceManager>& resourceManager,
        const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
        EigenMesh& cageMesh,
        EigenMesh& deformableMesh,
        bool offset,
        uint32_t targetCount,
        bool threeHitVariant = false,
        float alpha = 1.0f,
        float beta = -1.0f,
        float theta = 1.0f)
        : _device(device)
        , _transferQueueFamily(transferQueueFamily)
        , _faceSize(faceSize)
        , _format(format)
        , _descriptorPool(descriptorPool)
        , _resourceManager(resourceManager)
        , _renderPipelineManager(renderPipelineManager)
        , _cageMesh(cageMesh)
        , _deformableMesh(deformableMesh)
        , _offset(offset)
        , _targetCount(targetCount == 0 ? 1u : static_cast<int>(targetCount))
        , _threeHitVariant(threeHitVariant)
        , _alpha(alpha)
        , _beta(beta)
        , _theta(theta)
    {
    }

    uint32_t RequiredRenderTargetCount() const override;

    void Initialize() override;
    void Cleanup() override;

    void SetInteriorDistance(const InteriorDistanceSettings& settings) override
    {
        _interiorDistance = settings;
    }

    void DispatchAfterRender(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline,
        uint64_t waitValue,
        uint64_t signalValue,
        const CubemapRenderTarget& target);

    Eigen::MatrixXd Readback();

    void ConsumeSlot(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline,
        uint32_t waitValue);

    void SubmitReadbackCopy(
        uint32_t slot,
        VkSemaphore timeline,
        uint64_t waitValue,
        uint64_t signalValue);

    //new
    void RecordCompute(
        uint32_t slot,
        const CubemapRenderTarget& target,
        uint32_t deformableIndex);

    void SubmitAllComputes(
        VkSemaphore waitSemaphore,
        uint64_t waitValue,
        VkSemaphore signalSemaphore,
        uint64_t signalValue);

    void SubmitAllReadbackCopies(
        VkSemaphore waitSemaphore,
        uint64_t waitValue,
        VkSemaphore signalSemaphore,
        uint64_t signalValue);

    void ConsumeAllSlots(uint64_t numPass);

    // Three-hit variant: record a compute dispatch that combines two rendered
    // hits (color + depth) into the lambda/wsum buffers. Set A is weighted by
    // weightA, set B by weightB. Passing weightB == 0 (and any valid B views)
    // evaluates only set A, which is used for the theta-weighted third hit.
    void RecordCombinedCompute(
        uint32_t slot,
        uint32_t deformableIndex,
        VkImageView colorViewA,
        VkImageView depthViewA,
        VkImageView colorViewB,
        VkImageView depthViewB,
        float weightA,
        float weightB);

private:
    void CreatePipelineAndLayouts();
    void AllocateResources();
    void CreateCombinedPipelineAndLayout();
    void AllocateCombinedResources();
    void UpdateCombinedDescriptorSet(
        uint32_t slot,
        VkImageView colorViewA,
        VkImageView depthViewA,
        VkImageView colorViewB,
        VkImageView depthViewB);

    const uint32_t kDispatchGroupSize = 8;

    RenderResourceRef<Device> _device;
    uint32_t _transferQueueFamily = 0;
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
    std::vector<float> _wsumResults;

    EigenMesh& _cageMesh;
    EigenMesh& _deformableMesh;

    RenderResourceRef<DescriptorPool> _descriptorPool;
    std::shared_ptr<RenderResourceManager> _resourceManager;
    std::shared_ptr<RenderPipelineManager> _renderPipelineManager;

    void UpdateComputeDescriptorSet(uint32_t targetIndex, const CubemapRenderTarget& target);

    SphereWeightCalculator _sphereWeightCalculator;

    void CreateSampler();
    void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size);
    void WriteWeightsToFile(const std::string& filename);

    int _targetCount = 64;

    VkSampler _barySampler;

    bool _offset = false;

    // Interior-distance PMVC variant: table uploaded once, consumed by the
    // PMVCComputeInteriorDist compute shader instead of the rasterized depth. The
    // depth peeling hit passes keep working unchanged because the shader still uses
    // the depth texture for the no-hit test.
    [[nodiscard]] bool UseInteriorDistanceShader() const
    {
        return !_offset && _interiorDistance.IsEnabled();
    }
    InteriorDistanceSettings _interiorDistance;
    Buffer _interiorDistanceBuffer;

    void CreateDepthSampler();
    VkSampler _depthSampler;

    std::vector<uint32_t> _slotToDeformableIndex;

    // Three-hit variant resources.
    bool _threeHitVariant = false;
    float _alpha = 1.0f;
    float _beta = -1.0f;
    float _theta = 1.0f;

    PipelineHandle _combinedPipeline;
    RenderResourceRef<DescriptorSetLayout> _combinedLayout;
    std::vector<VkDescriptorSet> _combinedDescriptorSets = {};
};

struct ThreeHitPushConstants
{
    glm::ivec2 uFaceSize;
    int uNumTriangles;
    float uWeightA;
    float uWeightB;
};
