#include <Rendering/PMVC/CpuComputeStrategy.h>

uint32_t CpuComputeStrategy::RequiredRenderTargetCount() const
{
    // CPU path typically single-buffered or small ring
    return 3;
}

void CpuComputeStrategy::Initialize(uint32_t targetCount)
{
    _targetCount = targetCount;
}

void CpuComputeStrategy::WaitForTargetReuse(uint32_t,
    VkSemaphore,
    uint64_t)
{
    // CPU path: nothing to wait on
}

void CpuComputeStrategy::DispatchAfterRender(uint32_t,
    uint32_t,
    VkSemaphore,
    uint64_t,
    const CubemapRenderTarget&)
{
    // CPU path: no GPU compute dispatch
    // Rendering completion implicitly means data is ready
}

uint64_t CpuComputeStrategy::GetSlotCompletionValue(uint32_t) const
{
    // CPU work completes immediately
    return 0;
}

void CpuComputeStrategy::Readback(uint32_t,
    uint32_t,
    const std::string&)
{
    // Optional: hook for debug readback
    // Intentionally empty for placeholder
}

void CpuComputeStrategy::WaitAll(VkSemaphore)
{
    // CPU path: nothing outstanding
}