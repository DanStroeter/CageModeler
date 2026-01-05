#pragma once
#define VK_NO_PROTOTYPES
#include <cstdint>    
#include <string>   
#include <vulkan/vulkan.h> 
#include <glm/ext/vector_int2.hpp>

struct CubemapRenderTarget;

struct ComputePushConstants {
    int uNumCubemaps;
    int uNumCageVertices;
    glm::ivec2 uFaceSize;
    int uFacesPerCubemap;
    int uNumTriangles;
};

class ICubemapComputeStrategy
{
public:
    virtual ~ICubemapComputeStrategy() = default;

    virtual uint32_t RequiredRenderTargetCount() const = 0;

    virtual void Initialize(uint32_t targetCount) = 0;

    virtual void WaitForTargetReuse(uint32_t targetIndex,
        VkSemaphore timeline,
        uint64_t slotDoneValue) = 0;

    virtual void DispatchAfterRender(uint32_t cubemapIdx,
        uint32_t targetIndex,
        VkSemaphore timeline,
        uint64_t renderDoneValue, uint64_t copyDoneValue,
        const CubemapRenderTarget& target) = 0;

    virtual uint64_t GetSlotCompletionValue(uint32_t targetIndex) const = 0;

    virtual void Readback(uint32_t cubemapIdx, uint32_t targetIndex, const std::string& filename) = 0;

    virtual void WaitAll(VkSemaphore timeline) = 0;
};