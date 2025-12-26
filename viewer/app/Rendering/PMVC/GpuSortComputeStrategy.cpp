#include <Rendering/PMVC/GpuSortComputeStrategy.h>

uint32_t GpuSortComputeStrategy::RequiredRenderTargetCount() const
{
    return 2;
}

void GpuSortComputeStrategy::Initialize(uint32_t) {}
void GpuSortComputeStrategy::WaitForTargetReuse(uint32_t, VkSemaphore, uint64_t) {}
void GpuSortComputeStrategy::DispatchAfterRender(uint32_t, uint32_t, VkSemaphore, uint64_t, const CubemapRenderTarget&) {}
uint64_t GpuSortComputeStrategy::GetSlotCompletionValue(uint32_t) const { return 0; }
void GpuSortComputeStrategy::Readback(uint32_t, uint32_t, const std::string&) {}
void GpuSortComputeStrategy::WaitAll(VkSemaphore) {}
