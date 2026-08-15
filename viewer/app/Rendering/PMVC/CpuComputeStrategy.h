#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include <Rendering/PMVC/IComputeStrategy.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/DescriptorPool.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/PMVC/CubemapRenderInstance.h>
#include <Thread/ThreadPool.h>
#include <Mesh/GeometryUtils.h>

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
        bool offset,
        uint32_t targetCount)
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
        , _maxTargetCount(targetCount == 0 ? 1u : targetCount)
    {
    }

    uint32_t RequiredRenderTargetCount() const override;
    void Initialize() override;
    void Cleanup() override;

    void RecordReadback(uint32_t slot, const CubemapRenderTarget& target, uint32_t deformableIndex);
    void SubmitAllReadbacks(VkSemaphore waitSemaphore, uint64_t waitValue, VkSemaphore signalSemaphore, uint64_t signalValue);
    void ConsumeAllSlots();
    Eigen::MatrixXd Readback();

private:
    struct SlotReadback
    {
        VkBuffer colorBuffer = VK_NULL_HANDLE;
        VkDeviceMemory colorMemory = VK_NULL_HANDLE;
        void* colorMapped = nullptr;

        VkBuffer depthBuffer = VK_NULL_HANDLE;
        VkDeviceMemory depthMemory = VK_NULL_HANDLE;
        void* depthMapped = nullptr;
    };

    void AllocateResources();
    void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size);

    float ComputeSolidAngle(uint32_t texelX, uint32_t texelY) const;
    float DecodeDepthSample(const uint8_t* texel) const;
    void ComputeOnCpu(uint32_t deformableIndex, const SlotReadback& slot);

    RenderResourceRef<Device> _device;
    uint32_t _transferQueueFamily = 0;
    uint32_t _faceSize = 32;
    VkFormat _format = VK_FORMAT_R32G32B32A32_SFLOAT;

    VkFormat _depthFormat = VK_FORMAT_D32_SFLOAT;
    VkDeviceSize _depthBytesPerTexel = sizeof(float);

    VkCommandPool _computeCommandPool = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> _computeCommandBuffers = {};

    Buffer _vertexListBuffer;
    std::vector<uint32_t> _vertexList;
    std::vector<float> _solidAngles;

    Eigen::MatrixXd _lambdaResults;
    std::vector<float> _wsumResults;

    EigenMesh& _cageMesh;
    EigenMesh& _deformableMesh;

    RenderResourceRef<DescriptorPool> _descriptorPool;
    std::shared_ptr<RenderResourceManager> _resourceManager;
    std::shared_ptr<RenderPipelineManager> _renderPipelineManager;

    int _targetCount = 0;
    std::vector<SlotReadback> _slots;
    std::vector<uint32_t> _slotToDeformableIndex;
    std::unique_ptr<ThreadPool> _threadPool;
    uint32_t _cpuWorkerCount = 1;
    bool _offset = false;

    uint32_t _maxTargetCount = 64;
};
