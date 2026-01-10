#include <Rendering/PMVC/DebugCubemapComputeStrategy.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../../external/stb_image_write.h"

void DebugCubemapComputeStrategy::Initialize()
{
	int targetCount = RequiredRenderTargetCount();
    // Command pool for transfer command buffers
    VkCommandPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    poolInfo.queueFamilyIndex = _transferQueueFamily;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    VK_CHECK(vkCreateCommandPool(_device, &poolInfo, nullptr, &_cmdPool));

    _slots.resize(targetCount);
    _slotCopyDoneValue.assign(targetCount, 0);

    const VkDeviceSize bpt = BytesPerTexel(_format); // e.g. R32G32B32A32_SFLOAT -> 16 bytes
    const VkDeviceSize imageSizePerFace = VkDeviceSize(_faceSize) * _faceSize * bpt;
    const VkDeviceSize totalSize = imageSizePerFace * 6;

    // Allocate command buffers
    std::vector<VkCommandBuffer> cmds(targetCount);

    VkCommandBufferAllocateInfo cmdAlloc{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cmdAlloc.commandPool = _cmdPool;
    cmdAlloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdAlloc.commandBufferCount = targetCount;
    VK_CHECK(vkAllocateCommandBuffers(_device, &cmdAlloc, cmds.data()));

    for (uint32_t i = 0; i < targetCount; ++i)
    {
        _slots[i].cmd = cmds[i];
        _slots[i].sizeBytes = totalSize;

        // Create staging buffer
        VkBufferCreateInfo bufInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bufInfo.size = totalSize;
        bufInfo.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VK_CHECK(vkCreateBuffer(_device, &bufInfo, nullptr, &_slots[i].buffer));

        VkMemoryRequirements req{};
        vkGetBufferMemoryRequirements(_device, _slots[i].buffer, &req);

        VkMemoryAllocateInfo alloc{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
        alloc.allocationSize = req.size;
        alloc.memoryTypeIndex = FindMemoryType(req.memoryTypeBits,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
            VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

        VK_CHECK(vkAllocateMemory(_device, &alloc, nullptr, &_slots[i].memory));
        VK_CHECK(vkBindBufferMemory(_device, _slots[i].buffer, _slots[i].memory, 0));
    }
    for (int i = 0; i < _slotCopyDoneValue.size(); i++) {
		_slotCopyDoneValue[i] = 0;
    }
}

void DebugCubemapComputeStrategy::WaitForTargetReuse(
    uint32_t targetIndex,
    VkSemaphore timeline,
    uint64_t slotDoneValue)
{
    if (slotDoneValue == 0)
        return; // never used

    VkSemaphoreWaitInfo waitInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO };
    waitInfo.semaphoreCount = 1;
    waitInfo.pSemaphores = &timeline;
    waitInfo.pValues = &slotDoneValue;

    VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
}

void DebugCubemapComputeStrategy::DispatchAfterRender(
    uint32_t deformableIndex,
    uint32_t slot,
    VkSemaphore timeline,
    const CubemapRenderTarget& target)
{
  /*  const Slot& slot = _slots[targetIndex];

    const VkDeviceSize bpt = BytesPerTexel(_format);
    const VkDeviceSize imageSizePerFace = VkDeviceSize(_faceSize) * _faceSize * bpt;
    const VkDeviceSize totalSize = imageSizePerFace * 6;

    VkCommandBuffer cmd = slot.cmd;
    // Wait for prior usage of this command buffer to finish (using timeline semaphore)
    if (_slotCopyDoneValue[targetIndex] > 0)
    {
        VkSemaphoreWaitInfo waitInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO };
        waitInfo.semaphoreCount = 1;
        waitInfo.pSemaphores = &timeline;
        waitInfo.pValues = &_slotCopyDoneValue[targetIndex];

        VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
    }

    VK_CHECK(vkResetCommandBuffer(cmd, 0));

    VkCommandBufferBeginInfo begin{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(cmd, &begin));

    // Transition cubemap image to TRANSFER_SRC_OPTIMAL (from COLOR_ATTACHMENT_OPTIMAL is typical)
    // Use a conservative barrier if you do not track exact layouts:
    VkImageMemoryBarrier2 barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    barrier.dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
    barrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    barrier.image = target.cubemapImage;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 6;

    VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    dep.imageMemoryBarrierCount = 1;
    dep.pImageMemoryBarriers = &barrier;
    vkCmdPipelineBarrier2(cmd, &dep);

    // Copy 6 layers into contiguous regions
    std::array<VkBufferImageCopy, 6> regions{};
    for (uint32_t face = 0; face < 6; ++face)
    {
        regions[face].bufferOffset = imageSizePerFace * face;
        regions[face].bufferRowLength = 0;
        regions[face].bufferImageHeight = 0;
        regions[face].imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        regions[face].imageSubresource.mipLevel = 0;
        regions[face].imageSubresource.baseArrayLayer = face;
        regions[face].imageSubresource.layerCount = 1;
        regions[face].imageOffset = { 0, 0, 0 };
        regions[face].imageExtent = { _faceSize, _faceSize, 1 };
    }

    vkCmdCopyImageToBuffer(
        cmd,
        target.cubemapImage,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        slot.buffer,
        (uint32_t)regions.size(),
        regions.data());

    // Optional: transition back if you will use the image again on GPU without re-rendering.
    // For pure debug readback + overwrite next time, you can skip transitioning back.

    VK_CHECK(vkEndCommandBuffer(cmd));

    // Timeline: wait on renderDoneValue, signal copyDoneValue
    //const uint64_t copyDoneValue = renderDoneValue + 1; // simple scheme; you can also use a global counter

    VkSemaphoreSubmitInfo waitInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO };
    waitInfo.semaphore = timeline;
    waitInfo.value = renderDoneValue;
    waitInfo.stageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;

    VkSemaphoreSubmitInfo signalInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO };
    signalInfo.semaphore = timeline;
    signalInfo.value = copyDoneValue;
    signalInfo.stageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;

    VkCommandBufferSubmitInfo cmdInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO };
    cmdInfo.commandBuffer = cmd;

    VkSubmitInfo2 submit{ VK_STRUCTURE_TYPE_SUBMIT_INFO_2 };
    submit.waitSemaphoreInfoCount = 1;
    submit.pWaitSemaphoreInfos = &waitInfo;
    submit.commandBufferInfoCount = 1;
    submit.pCommandBufferInfos = &cmdInfo;
    submit.signalSemaphoreInfoCount = 1;
    submit.pSignalSemaphoreInfos = &signalInfo;

    VK_CHECK(vkQueueSubmit2(_transferQueue, 1, &submit, VK_NULL_HANDLE));

    _slotCopyDoneValue[targetIndex] = copyDoneValue;*/
}

void DebugCubemapComputeStrategy::Readback(uint32_t cubemapIdx, uint32_t targetIndex, const std::string& filename)
{
    const Slot& slot = _slots[targetIndex];

    void* raw = nullptr;
    VK_CHECK(vkMapMemory(_device, slot.memory, 0, slot.sizeBytes, 0, &raw));

    const uint32_t channels = 4;
    const uint32_t faceSize = _faceSize;
    const size_t texelCountPerFace = size_t(faceSize) * faceSize;

    const float* src = reinterpret_cast<const float*>(raw);

    // Vertical strip: (faceSize x faceSize*6), RGBA8
    std::vector<uint8_t> strip(faceSize * faceSize * 6 * 4);

    for (uint32_t face = 0; face < 6; ++face)
    {
        for (size_t i = 0; i < texelCountPerFace; ++i)
        {
            size_t srcIndex = (face * texelCountPerFace + i) * channels;
            size_t dstIndex = (face * texelCountPerFace + i) * 4;

            for (int c = 0; c < 4; ++c)
            {
                float f = src[srcIndex + c];
                f = std::max(0.0f, std::min(1.0f, f));
                strip[dstIndex + c] = static_cast<uint8_t>(f * 255.0f);
            }
        }
    }

    vkUnmapMemory(_device, slot.memory);

    stbi_write_png(
        filename.c_str(),
        (int)faceSize,
        (int)(faceSize * 6),
        4,
        strip.data(),
        (int)(faceSize * 4)
    );
    std::string c = "DebugCubemap: Saved cubemap " + cubemapIdx + filename;
    LOG_DEBUG(c);
}

void DebugCubemapComputeStrategy::WaitAll(VkSemaphore timeline)
{
    uint64_t maxValue = 0;
    for (uint64_t v : _slotCopyDoneValue) maxValue = std::max(maxValue, v);
    if (maxValue == 0) return;

    VkSemaphoreWaitInfo waitInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO };
    waitInfo.semaphoreCount = 1;
    waitInfo.pSemaphores = &timeline;
    waitInfo.pValues = &maxValue;

    VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
}

uint32_t DebugCubemapComputeStrategy::FindMemoryType(uint32_t typeBits, VkMemoryPropertyFlags props) const
{
    VkPhysicalDeviceMemoryProperties memProps{};
    vkGetPhysicalDeviceMemoryProperties(_physicalDevice, &memProps);

    for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i)
    {
        if ((typeBits & (1u << i)) && ((memProps.memoryTypes[i].propertyFlags & props) == props))
            return i;
    }
    throw std::runtime_error("No suitable memory type found for staging buffer");
}

VkDeviceSize DebugCubemapComputeStrategy::BytesPerTexel(VkFormat fmt) const
{
    // For your debug path, you stated VK_FORMAT_R32G32B32A32_SFLOAT
    // which is 4 * 32-bit floats = 16 bytes.
    switch (fmt)
    {
    case VK_FORMAT_R32G32B32A32_SFLOAT: return 16;
    default:
        throw std::runtime_error("BytesPerTexel: unsupported format in debug strategy");
    }
}