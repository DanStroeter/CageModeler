class ICubemapComputeStrategy
{
public:
    virtual ~ICubemapComputeStrategy() = default;
    
    virtual uint32_t RequiredRenderTargetCount() const = 0;
    virtual void Initialize(CubemapRenderInstance& owner) = 0;
    virtual void Dispatch(uint32_t cubemapIndex) = 0;
    virtual void Wait() = 0;
    virtual void Readback(uint32_t cubemapIndex) = 0;
};