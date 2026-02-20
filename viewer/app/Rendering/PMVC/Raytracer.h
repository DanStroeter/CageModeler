#pragma once 
#include <Rendering/PMVC/CubemapRenderInstance.h> 
#include <Rendering/Core/RenderProxy.h> 
#include <Rendering/Core/Device.h> 
#include <Rendering/Core/DescriptorPool.h> 
#include <Rendering/Core/Buffer.h> 
#include <Rendering/Core/AlignedVector.h>
#include <Rendering/RenderPipelineManager.h> 
#include <Rendering/Scene/SceneData.h> 
#include <Mesh/GeometryUtils.h>
#include <Editor/Light.h> 
#include <Core/Subsystem.h> 
#include <Eigen/Core> 
#include <vector> 
#include <cstdint> 
#include <filesystem>

class RenderPipelineManager; 
class RenderResourceManager; 

class Raytracer final
{
public:
    Raytracer(
        const std::shared_ptr<RenderPipelineManager>& pipelineManager,
        const std::shared_ptr<RenderResourceManager>& resourceManager,
        const RenderResourceRef<Device> device);

    ~Raytracer();

    void Initialize();
    void SetCage(const EigenMesh& mesh) { _cageMesh = mesh; } 
    void SetMesh(const EigenMesh& mesh) { _deformableMesh = mesh; }
    MeshOperationResult<MeshComputeWeightsOperationResult> ComputeCoordinates();

private:
    // --- Start trace ---
    void StartRayTrace();

    // --- Device and Managers --- 
    RenderResourceRef<Device>                _device;
    std::shared_ptr<RenderResourceManager>   _resourceManager;
    std::shared_ptr<RenderPipelineManager>   _pipelineManager;

	// --- Device Properties ---
    VkPhysicalDeviceRayTracingPipelinePropertiesKHR _rtProperties{
        .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR
    };
    VkPhysicalDeviceAccelerationStructurePropertiesKHR _asProperties{
        .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR
    };

    void GetRaytracingComponents();

	// --- Queue and Command Pool ---
    uint32_t _queueFamilyIndex = 0;
    VkQueue  _queue = VK_NULL_HANDLE;
    VkCommandPool _commandPool = VK_NULL_HANDLE;

    void CreateCommandPool();

    // --- Scene Data ---
    struct CageBuffers
    {
        Buffer vertexBuffer;       
        Buffer indexBuffer;         
        uint32_t vertexCount = 0;
        uint32_t indexCount = 0;
    };

    struct RayOriginBuffer
    {
        Buffer vertexBuffer;
        uint32_t vertexCount = 0;
    };

    EigenMesh _cageMesh;
    EigenMesh _deformableMesh;
    CageBuffers _cageGeometry;
    RayOriginBuffer _deformableGeometry;

    void CreateCageBuffers(EigenMesh& geometry, CageBuffers& geometryBuffers);
    void CreateRayOriginBuffer(EigenMesh& geometry, RayOriginBuffer& geometryBuffers);

    // --- Acceleration Structures ---
    struct AccelerationStructure
    {
        VkAccelerationStructureKHR handle = VK_NULL_HANDLE;
        Buffer buffer;                     
        VkDeviceAddress deviceAddress = 0;
    };

    AccelerationStructure _blas;
    AccelerationStructure _tlas;

    void CreateAccelerationStructures();
    void CreateBLAS();
    void CreateTLAS();
    void BuildAccelerationStructure(
        VkAccelerationStructureGeometryKHR& geometry,
        uint32_t primitiveCount,
        VkAccelerationStructureTypeKHR type,
        AccelerationStructure& outAS);

    // --- Ray Tracing Pipeline ---
    struct RTPipeline
    {
        VkPipeline       pipeline = VK_NULL_HANDLE;
        VkPipelineLayout layout = VK_NULL_HANDLE;

        VkDescriptorSetLayout descriptorLayout = VK_NULL_HANDLE;
        VkDescriptorSet       descriptorSet = VK_NULL_HANDLE;
        VkDescriptorPool      descriptorPool = VK_NULL_HANDLE;
    };

    RTPipeline _rtPipeline;

    void CreateRayTracingPipeline();
    void CreateRayTracingDescriptorSet();
    void UpdateDescriptorSet();

    // --- Shaders and Shader Binding Table ---
    struct ShaderBindingTable
    {
        Buffer buffer;          
        VkStridedDeviceAddressRegionKHR raygen{};
        VkStridedDeviceAddressRegionKHR miss{};
        VkStridedDeviceAddressRegionKHR hit{};
        VkStridedDeviceAddressRegionKHR callable{};
    };

    std::vector<uint8_t> _shaderHandles;
    VkShaderModule _raygenShader = VK_NULL_HANDLE;
    VkShaderModule _missShader = VK_NULL_HANDLE;
    VkShaderModule _hitShader = VK_NULL_HANDLE;
    VkStridedDeviceAddressRegionKHR _raygenRegion{};
    VkStridedDeviceAddressRegionKHR _missRegion{};
    VkStridedDeviceAddressRegionKHR _hitRegion{};
    VkStridedDeviceAddressRegionKHR _callableRegion{};
    ShaderBindingTable _sbt;

    void CreateShaderBindingTable(const VkRayTracingPipelineCreateInfoKHR& pipelineInfo);
    void LoadShaders();
    std::vector<uint32_t> LoadSPIRV(const std::string& filename);
    VkShaderModule CreateShaderModule(const std::vector<uint32_t>& code);

    // Ray Tracing Buffers
    struct RayBuffers
    {
        Buffer rayDirections;             
        Buffer hitBuffer;  
        Buffer atomicCounter;
    };
    struct HitBufferData {
        uint32_t rayIndex;
    };

    RayBuffers _rayBuffers;

    void CreateRayBuffers();

    // --- Push Constants ---
    struct PushConstants
    {
        uint32_t vertexCount;
        uint32_t raysPerVertex;
        uint32_t maxHitsPerRay;
        uint32_t padding;
    };

    PushConstants _pushConstants{0, 6, 1, 0};

    // --- Sync ---
    struct TraceSync {
        VkFence fence = VK_NULL_HANDLE;
        VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
        bool isTracing = false;
    };

    TraceSync _traceSync;

    void WaitForTrace();

    // --- Readback ---
    struct ReadbackData {
        MemoryMappedBuffer stagingBuffer; 
        VkDeviceSize size = 0;
        HitBufferData* mappedData;

        VkCommandBuffer copyCmd = VK_NULL_HANDLE;
        VkFence copyCompleteFence = VK_NULL_HANDLE;
    };

    ReadbackData _readback;

    void CreateReadbackResources();
    void SubmitReadback();
    void WaitForReadbackComplete();
	std::vector<HitBufferData> GetHitResults();
    void WriteHitsToFile(const std::string& filename, const std::vector<HitBufferData>& hits);

    // --- Helper Functions ---
    void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size);
    VkDeviceAddress GetBufferAddress(const Buffer& buffer) const;

    // Cleanup
    //void Cleanup();
    //void CleanupShaders();
};