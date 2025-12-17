class GpuSortComputeStrategy final : public ICubemapComputeStrategy
{
public:
    uint32_t RequiredRenderTargetCount() const override
    {
        return 2;
    }
};