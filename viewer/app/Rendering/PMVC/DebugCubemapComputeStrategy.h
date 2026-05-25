#pragma once
#include <Rendering/PMVC/IComputeStrategy.h>
#include <vulkan/vulkan.h>
#include <Rendering/PMVC/CubemapRenderInstance.h>
#include <Rendering/Utils/VulkanUtils.h>


class DebugCubemapComputeStrategy final : public ICubemapComputeStrategy
{
public:
    DebugCubemapComputeStrategy(
        VkDevice device,
        VkPhysicalDevice physicalDevice,
        VkQueue transferQueue,
        uint32_t transferQueueFamily,
        uint32_t faceSize,
        VkFormat format)
        : _device(device)
        , _physicalDevice(physicalDevice)
        , _transferQueue(transferQueue)
        , _transferQueueFamily(transferQueueFamily)
        , _faceSize(faceSize)
        , _format(format)
    {
    }

    uint32_t RequiredRenderTargetCount() const override
    {
        // Triple-buffering is sensible for CPU readback to overlap render/copy/map.
        return 8;
    }

    void Initialize() override;
    void WaitForTargetReuse(uint32_t targetIndex, VkSemaphore timeline, uint64_t slotDoneValue);
    void DispatchAfterRender(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline,
		const CubemapRenderTarget& target);

    uint64_t GetSlotCompletionValue(uint32_t targetIndex) const
    {
        return _slotCopyDoneValue[targetIndex];
    }

    void Readback(uint32_t cubemapIdx, uint32_t targetIndex, const std::string& filename);
    void WaitAll(VkSemaphore timeline);

    void ConsumeSlot(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline) {
    }

private:
    struct Slot
    {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        VkDeviceSize sizeBytes = 0;
    };

    VkDevice _device = VK_NULL_HANDLE;
    VkPhysicalDevice _physicalDevice = VK_NULL_HANDLE;
    VkQueue _transferQueue = VK_NULL_HANDLE;
    uint32_t _transferQueueFamily = 0;

    VkCommandPool _cmdPool = VK_NULL_HANDLE;

    uint32_t _faceSize = 512;
    VkFormat _format = VK_FORMAT_R32G32B32A32_SFLOAT;

    std::vector<Slot> _slots;
    std::vector<uint64_t> _slotCopyDoneValue; // timeline values guarding reuse

    // Helper functions
    uint32_t FindMemoryType(uint32_t typeBits, VkMemoryPropertyFlags props) const;
    VkDeviceSize BytesPerTexel(VkFormat fmt) const;

    void RecordCopyCmd(VkCommandBuffer cmd,
        VkImage cubemapImage,
        VkDeviceSize imageSizePerFace,
        VkDeviceSize totalSize);
};
