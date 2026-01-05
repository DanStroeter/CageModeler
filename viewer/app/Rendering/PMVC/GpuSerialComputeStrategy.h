#pragma once

#include <Rendering/PMVC/IComputeStrategy.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/PMVC/CubemapRenderInstance.h>

struct ComputePushConstants {
	int uNumCubemaps;
	int uNumCageVertices;
	glm::ivec2 uFaceSize;
	int uFacesPerCubemap;
	int uNumTriangles;
};

class GpuSerialComputeStrategy final : public ICubemapComputeStrategy
{
public:
    GpuSerialComputeStrategy(
        VkDevice device,
        VkPhysicalDevice physicalDevice,
        VkQueue transferQueue,
        uint32_t transferQueueFamily,
        uint32_t faceSize,
        VkFormat format, 
        CubemapRenderInstance& cubemaprenderinstance)
        : _device(device)
        , _physicalDevice(physicalDevice)
        , _transferQueue(transferQueue)
        , _transferQueueFamily(transferQueueFamily)
        , _faceSize(faceSize)
        , _format(format)
		, _cubemapRenderInstance(cubemaprenderinstance)
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
private:
    void CreateComputeCommandPool(uint32_t queueFamilyIndex);
    void CreateComputeDescriptorSetLayout();
    void CreateComputeBuffers();
    void AllocateComputeDescriptorSet();
    void UpdateComputeDescriptorSet();
    void CreateComputePipeline();
    void CreateComputeCommandBuffer();
    void ComputeCoordinates(uint32_t cubeIndex);
    void ReadbackCompute(uint32_t cubeIndex);
    void storeLambdaForVertex(uint32_t cubeIndex, const float* lambdaCPU);
	void storeWsumForVertex(uint32_t cubeIndex, const float* wsumCPU);
    void WriteWeightsToFile(const std::string& filename);
    void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size);
    void InsertImageMemoryBarrierToGeneral(VkCommandBuffer cmd, VkImage image, VkImageSubresourceRange subresourceRange);

    CubemapRenderInstance& _cubemapRenderInstance;
    Buffer _lambdaBuffer;
    Buffer _wsumBuffer;
    Buffer _vertexListBuffer;
    MemoryMappedBuffer _lambdaStagingBuffer;
    MemoryMappedBuffer _wsumStagingBuffer;
    VkDescriptorSet _computeDescriptorSet;
    VkCommandPool _computeCommandPool;
    VkCommandBuffer _computeCommandBuffer;
    VkSampler _sampler;
    VkImageView _baryTexImageView = VK_NULL_HANDLE;
    std::vector<VkImageView> _baryTexImageViews;
    PipelineHandle _computePipelineHandle;
    RenderResourceRef<DescriptorSetLayout> _computeLayout;
    std::vector<std::vector<float>> _lambdaResults;
    std::vector<float> _wsumResults;

    VkDevice _device;
	VkPhysicalDevice _physicalDevice = VK_NULL_HANDLE;
	VkQueue _transferQueue = VK_NULL_HANDLE;
	uint32_t _transferQueueFamily = 0;
	uint32_t _faceSize = 512;
	VkFormat _format = VK_FORMAT_R32G32B32A32_SFLOAT;
};
