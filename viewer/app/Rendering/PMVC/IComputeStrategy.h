#pragma once
#define VK_NO_PROTOTYPES
#include <cstdint>    
#include <string>   
#include <vulkan/vulkan.h> 
#include <glm/ext/vector_int2.hpp>

struct CubemapRenderTarget;

struct ComputePushConstants {
    glm::ivec2 uFaceSize;
    int uNumTriangles;
};

class ICubemapComputeStrategy
{
public:
    virtual ~ICubemapComputeStrategy() = default;

    virtual uint32_t RequiredRenderTargetCount() const = 0;

    virtual void Initialize() { };

    virtual void Cleanup() { };

    /*virtual void DispatchAfterRender(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline,
        const CubemapRenderTarget& target) = 0;

    virtual void Readback() {};

    virtual void ConsumeSlot(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline) = 0;

    virtual void SubmitReadbackCopy(
        uint32_t slot,
        VkSemaphore timeline) {
    };*/
};
