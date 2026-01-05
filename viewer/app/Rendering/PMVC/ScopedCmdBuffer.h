#pragma once

#include <Rendering/Core/Device.h>
#include <vulkan/vulkan.h>

class ScopedCmdBuffer {
public:
    ScopedCmdBuffer(RenderResourceRef<Device> device, VkCommandPool pool);
    ~ScopedCmdBuffer();

    VkCommandBuffer Get() const { return _cmd; }

private:
    VkDevice _device;
    VkCommandPool _pool;
    VkCommandBuffer _cmd;
};