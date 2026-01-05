#pragma once

#include <vulkan/vulkan.h>

class ScopedCmdBuffer {
public:
    ScopedCmdBuffer(VkDevice device, VkCommandPool pool);
    ~ScopedCmdBuffer();

    VkCommandBuffer Get() const { return _cmd; }

private:
    VkDevice _device;
    VkCommandPool _pool;
    VkCommandBuffer _cmd;
};