#pragma once

#include <Rendering/PMVC/IComputeStrategy.h>

class GpuSerialComputeStrategy final : public ICubemapComputeStrategy
{
public:
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

};
