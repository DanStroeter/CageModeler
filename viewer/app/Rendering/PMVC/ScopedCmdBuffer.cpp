#include<Rendering/PMVC/ScopedCmdBuffer.h>

/*ScopedCmdBuffer::ScopedCmdBuffer(RenderResourceRef<Device> device, VkCommandPool pool)
    : _device(device), _pool(pool), _cmd(VK_NULL_HANDLE)
{
    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = _pool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1;

    vkAllocateCommandBuffers(_device, &allocInfo, &_cmd);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    vkBeginCommandBuffer(_cmd, &beginInfo);
}

ScopedCmdBuffer::~ScopedCmdBuffer()
{
    if (_cmd != VK_NULL_HANDLE) {
        vkEndCommandBuffer(_cmd);

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &_cmd;

        VkQueue graphicsQueue;
        // NOTE: this assumes queue index 0 and graphics family is known — adjust if needed
        vkGetDeviceQueue(_device, 0 /* graphics family index , 0, &graphicsQueue);
        vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(graphicsQueue);

        vkFreeCommandBuffers(_device, _pool, 1, &_cmd);
    }
}*/

