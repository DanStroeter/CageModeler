#include <Rendering/PMVC/GpuSerialComputeStrategy.h>

uint32_t GpuSerialComputeStrategy::RequiredRenderTargetCount() const
{
    return 2;
}

void GpuSerialComputeStrategy::Initialize(uint32_t) {}
void GpuSerialComputeStrategy::WaitForTargetReuse(uint32_t, VkSemaphore, uint64_t) {}
void GpuSerialComputeStrategy::DispatchAfterRender(uint32_t, uint32_t, VkSemaphore, uint64_t, uint64_t, const CubemapRenderTarget&) {}
uint64_t GpuSerialComputeStrategy::GetSlotCompletionValue(uint32_t) const { return 0; }
void GpuSerialComputeStrategy::Readback(uint32_t, uint32_t, const std::string&) {}
void GpuSerialComputeStrategy::WaitAll(VkSemaphore) {}
