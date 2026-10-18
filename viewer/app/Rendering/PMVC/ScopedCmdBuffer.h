#pragma once

#include <Rendering/Core/Device.h>
#include <vulkan/vulkan.h>

class ScopedCmdBuffer
{
public:
    ScopedCmdBuffer(
        RenderResourceRef<Device> device,
        VkCommandPool pool)
        : _device(device), _pool(pool)
    {
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = _pool;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;

        VK_CHECK(vkAllocateCommandBuffers(_device, &allocInfo, &_cmd));

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        VK_CHECK(vkBeginCommandBuffer(_cmd, &beginInfo));
    }

    VkCommandBuffer Get() const { return _cmd; }

    void SubmitAndWait(VkQueue queue)
    {
        //VK_ASSERT(!_submitted);

        VK_CHECK(vkEndCommandBuffer(_cmd));

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &_cmd;

        VK_CHECK(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));
        VK_CHECK(vkQueueWaitIdle(queue));

        _submitted = true;
    }

    void Submit(VkQueue queue, VkFence fence = VK_NULL_HANDLE)
    {
        //VK_ASSERT(!_submitted);

        VK_CHECK(vkEndCommandBuffer(_cmd));

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &_cmd;

        VK_CHECK(vkQueueSubmit(queue, 1, &submitInfo, fence));

        _submitted = true;
    }

    ~ScopedCmdBuffer()
    {
        if (_cmd != VK_NULL_HANDLE)
            vkFreeCommandBuffers(_device, _pool, 1, &_cmd);
    }

private:
    RenderResourceRef<Device> _device;
    VkCommandPool _pool;
    VkCommandBuffer _cmd = VK_NULL_HANDLE;
    bool _submitted = false;
};