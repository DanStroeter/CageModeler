#pragma once

#include <Rendering/PMVC/IComputeStrategy.h>
#include <Rendering/Core/DescriptorSetLayout.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/Pipeline.h>
#include <Mesh/GeometryUtils.h>
#include <Rendering/Core/DescriptorPool.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/PMVC/CubemapRenderInstance.h>

class GpuAtomicComputeStrategy final : public ICubemapComputeStrategy
{
public:
    GpuAtomicComputeStrategy(
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
    }

    uint32_t RequiredRenderTargetCount() const override;

    void Initialize(uint32_t targetCount) override;

    void WaitForTargetReuse(uint32_t targetIndex,
        VkSemaphore timeline,
        uint64_t slotDoneValue) override;

    void DispatchAfterRender(uint32_t cubemapIdx,
        uint32_t targetIndex,
        VkSemaphore timeline,
        uint64_t renderDoneValue,
        uint64_t copyDoneValue,
        const CubemapRenderTarget& target) override;

    uint64_t GetSlotCompletionValue(uint32_t targetIndex) const override;

    void Readback(uint32_t cubemapIdx,
        uint32_t targetIndex,
        const std::string& filename) override;

    void WaitAll(VkSemaphore timeline) override;

    std::vector<VkCommandBuffer> _computeCommandBuffers;

private:
    void CreatePipelineAndLayouts();
    void AllocateResources();

    const uint32_t kCubemapFaceCount = 6;
    const uint32_t kFaceSize = 512;
    const uint32_t kDispatchGroupSize = 8;

    RenderResourceRef<Device> _device;
	uint32_t _transferQueueFamily = 0;
	uint32_t _faceSize = 512;
	VkFormat _format = VK_FORMAT_R32G32B32A32_SFLOAT;

    // Compute pipeline and descriptors
    PipelineHandle _computePipeline;
    RenderResourceRef<DescriptorSetLayout> _computeLayout;
    VkDescriptorSet _computeDescriptorSet = VK_NULL_HANDLE;

    // Command resources
    VkCommandPool _computeCommandPool = VK_NULL_HANDLE;
    VkCommandBuffer _computeCommandBuffer = VK_NULL_HANDLE;

    // Sampler and textures
    VkSampler _sampler = VK_NULL_HANDLE;
    VkImageView _baryTexImageView = VK_NULL_HANDLE;
    std::vector<VkImageView> _baryTexImageViews;

    // Buffers
    Buffer _lambdaBuffer;
    Buffer _wsumBuffer;
    Buffer _vertexListBuffer;

    // Staging buffers (CPU-visible)
    MemoryMappedBuffer _lambdaStagingBuffer;
    MemoryMappedBuffer _wsumStagingBuffer;

    // CPU-side result storage
    std::vector<std::vector<float>> _lambdaResults;
    std::vector<float> _wsumResults;

    EigenMesh& _cageMesh;
    EigenMesh& _deformableMesh;

    RenderResourceRef<DescriptorPool> _descriptorPool;
    std::shared_ptr<RenderResourceManager> _resourceManager;
    std::shared_ptr<RenderPipelineManager> _renderPipelineManager;

    uint64_t _timelineValue = 0;
    std::vector<uint64_t> _slotDoneValue;

    void storeLambdaForVertex(uint32_t cubeIndex, const float* lambdaCPU);
    void storeWsumForVertex(uint32_t cubeIndex, const float* wsumCPU);
    void UpdateComputeDescriptorSet();

	SphereWeightCalculator _sphereWeightCalculator;

    void InsertImageMemoryBarrierToGeneral(
        VkCommandBuffer cmd,
        VkImage image,
        VkImageSubresourceRange subresourceRange);
    void CreateSampler();
    void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size);

};
