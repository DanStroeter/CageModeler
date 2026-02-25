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

class SphereWeightCalculator;

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
        EigenMesh& deformableMesh)
        : _device(device)
        , _transferQueueFamily(transferQueueFamily)
        , _faceSize(faceSize)
        , _format(format)
        , _descriptorPool(descriptorPool)
        , _resourceManager(resourceManager)
        , _renderPipelineManager(renderPipelineManager)
        , _cageMesh(cageMesh)
        , _deformableMesh(deformableMesh)
    {
		_targetCount = _deformableMesh._vertices.rows();
    }

    uint32_t RequiredRenderTargetCount() const override;
    void Initialize() override {};
    void Initialize(uint32_t targetCount);

    void DispatchAfterRender(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline,
        uint64_t waitValue,
        uint64_t signalValue,
        const CubemapRenderTarget& target);

    void SubmitAllComputes(
        VkSemaphore waitSemaphore,
        uint64_t waitValue,
        VkSemaphore signalSemaphore,
        uint64_t signalValue);

    Eigen::MatrixXd Readback();

    void RecordReadbackCopy(uint32_t slot);

    void ConsumeAllSlots();

    void SubmitAllReadbackCopies(
        VkSemaphore waitSemaphore,
        uint64_t waitValue,
        VkSemaphore signalSemaphore,
        uint64_t signalValue);

    void RecordCompute(
        uint32_t slot,
        const CubemapRenderTarget& target);

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

private:
    void CreatePipelineAndLayouts();
    void AllocateResources();

    const uint32_t kDispatchGroupSize = 16; // maybe 8

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

    int _targetCount = 6;

    VkSampler _barySampler;
};
