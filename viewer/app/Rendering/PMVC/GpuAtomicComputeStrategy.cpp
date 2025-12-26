#include <Rendering/PMVC/GpuAtomicComputeStrategy.h>

uint32_t GpuAtomicComputeStrategy::RequiredRenderTargetCount() const
{
    return 1;
}

void GpuAtomicComputeStrategy::Initialize(uint32_t)
{
}

void GpuAtomicComputeStrategy::WaitForTargetReuse(
    uint32_t,
    VkSemaphore,
    uint64_t)
{
}

void GpuAtomicComputeStrategy::DispatchAfterRender(
    uint32_t,
    uint32_t,
    VkSemaphore,
    uint64_t,
    const CubemapRenderTarget&)
{
}

uint64_t GpuAtomicComputeStrategy::GetSlotCompletionValue(uint32_t) const
{
    return 0;
}

void GpuAtomicComputeStrategy::Readback(
    uint32_t,
    uint32_t,
    const std::string&)
{
}

void GpuAtomicComputeStrategy::WaitAll(VkSemaphore)
{
}