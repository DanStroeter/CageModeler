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

//class SphereWeightCalculator;

class CpuComputeStrategy final : public ICubemapComputeStrategy
{
public:
    CpuComputeStrategy(
        RenderResourceRef<Device> device,
        uint32_t transferQueueFamily,
        uint32_t faceSize,
        VkFormat format,
        RenderResourceRef<DescriptorPool> descriptorPool,
        const std::shared_ptr<RenderResourceManager>& resourceManager,
        const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
        EigenMesh& cageMesh,
        EigenMesh& deformableMesh,
        bool offset)
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
    {
    }

    uint32_t RequiredRenderTargetCount() const override;

    void Initialize() override;

    Eigen::MatrixXd Readback();

    //new
    void RecordReadback(
        uint32_t slot,
        const CubemapRenderTarget& target,
        uint32_t deformableIndex);

    void SubmitAllReadbacks(
        VkSemaphore waitSemaphore,
        uint64_t waitValue,
        VkSemaphore signalSemaphore,
        uint64_t signalValue);

private:
    struct SlotReadback
    {
        VkBuffer colorBuffer;
        VkDeviceMemory colorMemory;
        void* colorMapped;

        VkBuffer depthBuffer;
        VkDeviceMemory depthMemory;
        void* depthMapped;
    };

    //helpers for cpu compute
    float ComputeSolidAngle(uint32_t texelX, uint32_t texelY) const;
    float DecodeDepthSample(const uint8_t* texel) const;
    void CopyImagesToStaging(const CubemapRenderTarget& target, const SlotReadback& slot);
    void ComputeOnCpu(uint32_t deformableIndex, const SlotReadback& slot);

    void AllocateResources();

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

    // CPU-side result storage
    Eigen::MatrixXd _lambdaResults;
    std::vector<float> _wsumResults;

    EigenMesh& _cageMesh;
    EigenMesh& _deformableMesh;

    RenderResourceRef<DescriptorPool> _descriptorPool;
    std::shared_ptr<RenderResourceManager> _resourceManager;
    std::shared_ptr<RenderPipelineManager> _renderPipelineManager;

   SphereWeightCalculator _sphereWeightCalculator;

    void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size);
    void WriteWeightsToFile(const std::string& filename);

    int _targetCount = 3;

	std::vector<SlotReadback> _slots;

    bool _offset = false;

    std::vector<uint32_t> _slotToDeformableIndex;
};
