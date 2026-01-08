#pragma once

#include <Rendering/PMVC/IComputeStrategy.h>

class GpuSortComputeStrategy final : public ICubemapComputeStrategy
{
public:
    uint32_t RequiredRenderTargetCount() const override;

    void Initialize(uint32_t targetCount) override;
    void WaitForTargetReuse(uint32_t targetIndex,
        VkSemaphore timeline,
        uint64_t slotDoneValue) override;

    void DispatchAfterRender(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline,
		const CubemapRenderTarget& target) override;

    void ConsumeSlot(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline) override {
    }

    uint64_t GetSlotCompletionValue(uint32_t targetIndex) const override;
    void Readback(uint32_t cubemapIdx,
        uint32_t targetIndex,
        const std::string& filename) override;

    void WaitAll(VkSemaphore timeline) override;
};
