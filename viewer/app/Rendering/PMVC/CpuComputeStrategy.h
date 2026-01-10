#pragma once

#include <Rendering/PMVC/IComputeStrategy.h>

class CpuComputeStrategy final : public ICubemapComputeStrategy
{
public:
    uint32_t RequiredRenderTargetCount() const override;

    void Initialize() override;

    void WaitForTargetReuse(uint32_t targetIndex,
        VkSemaphore timeline,
        uint64_t slotDoneValue);

    void DispatchAfterRender(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline,
		const CubemapRenderTarget& target);

    uint64_t GetSlotCompletionValue(uint32_t targetIndex) const;

    void Readback(uint32_t cubemapIdx,
        uint32_t targetIndex,
        const std::string& filename);

    void WaitAll(VkSemaphore timeline);

    void ConsumeSlot(
        uint32_t deformableIndex,
        uint32_t slot,
        VkSemaphore timeline) { }

private:
    uint32_t _targetCount = 0;
};
