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

// Forward declarations
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

    // ============================================================
    // === Core Device / Managers
    // ============================================================

    RenderResourceRef<Device>                _device;
    std::shared_ptr<RenderResourceManager>   _resourceManager;
    std::shared_ptr<RenderPipelineManager>   _pipelineManager;

    uint32_t _queueFamilyIndex = 0;
    VkQueue  _queue = VK_NULL_HANDLE;
    VkCommandPool _commandPool = VK_NULL_HANDLE;

    // ============================================================
    // === Scene Data
    // ============================================================

    EigenMesh _cageMesh;
    EigenMesh _deformableMesh;

    // ============================================================
    // === Geometry Buffers (GPU-only)
    // ============================================================

    struct GeometryBuffers
    {
        Buffer vertexBuffer;        // DEVICE_LOCAL
        Buffer indexBuffer;         // DEVICE_LOCAL
        uint32_t vertexCount = 0;
        uint32_t indexCount = 0;
    };

    GeometryBuffers _cageGeometry;
    GeometryBuffers _deformableGeometry;

    // ============================================================
    // === Acceleration Structures
    // ============================================================

    struct AccelerationStructure
    {
        VkAccelerationStructureKHR handle = VK_NULL_HANDLE;
        Buffer buffer;                     // DEVICE_LOCAL
        VkDeviceAddress deviceAddress = 0;
    };

    AccelerationStructure _blas;
    AccelerationStructure _tlas;

    // ============================================================
    // === Ray Tracing Pipeline
    // ============================================================

    struct RTPipeline
    {
        VkPipeline       pipeline = VK_NULL_HANDLE;
        VkPipelineLayout layout = VK_NULL_HANDLE;

        VkDescriptorSetLayout descriptorLayout = VK_NULL_HANDLE;
        VkDescriptorSet       descriptorSet = VK_NULL_HANDLE;
        VkDescriptorPool      descriptorPool = VK_NULL_HANDLE;
    };

    RTPipeline _rtPipeline;

    // ============================================================
    // === Shader Binding Table (CPU-written, GPU-read)
    // ============================================================

    struct ShaderBindingTable
    {
        Buffer buffer;          // HOST_VISIBLE + DEVICE_ADDRESS
        VkStridedDeviceAddressRegionKHR raygen{};
        VkStridedDeviceAddressRegionKHR miss{};
        VkStridedDeviceAddressRegionKHR hit{};
        VkStridedDeviceAddressRegionKHR callable{};
    };

    VkPhysicalDeviceRayTracingPipelinePropertiesKHR _rtProperties{
        .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR
    };
    VkPhysicalDeviceAccelerationStructurePropertiesKHR _asProperties{
        .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR
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

    // ============================================================
    // === Ray Tracing Working Buffers
    // ============================================================

    struct RayBuffers
    {
        Buffer rayDirections;               // DEVICE_LOCAL
        Buffer hitBuffer;       // GPU write -> CPU read
        //Buffer mvcWeights;      // GPU write -> CPU read
        //Buffer atomicCounterBuffer;
    };

    RayBuffers _rayBuffers;

    // ============================================================
    // === Push Constants
    // ============================================================

    struct PushConstants
    {
        uint32_t vertexCount = 0;
        uint32_t raysPerVertex = 256;
        uint32_t maxHitsPerRay = 8;
        uint32_t padding = 0;
    };

    PushConstants _pushConstants;

    // ============================================================
    // === Initialization Steps
    // ============================================================

    void CreateCommandPool();
    void CreateGeometryBuffers(EigenMesh& geometry, GeometryBuffers& geometryBuffers);

    void CreateAccelerationStructures();
    void CreateBLAS();
    void CreateTLAS();
    void BuildAccelerationStructure(
        VkAccelerationStructureGeometryKHR& geometry,
        uint32_t primitiveCount,
        VkAccelerationStructureTypeKHR type,
        AccelerationStructure& outAS);

    //void CreateRayTracingPipelineLayout();
    void CreateRayTracingPipeline();
    void CreateRayBuffers();
    void GetRaytracingComponents();

    void CreateRayTracingDescriptorSet();
    void UpdateDescriptorSet();

    //Shaders
    void CreateShaderBindingTable(const VkRayTracingPipelineCreateInfoKHR& pipelineInfo);
    void LoadShaders();
    std::vector<uint32_t> LoadSPIRV(const std::string& filename);
    VkShaderModule CreateShaderModule(const std::vector<uint32_t>& code);
   
    void StartRayTrace();
    void ResetHitBuffer();

    struct SimpleHit {
        uint32_t faceIndex;        // Which cage triangle was hit
        float barycentricU;        // Barycentric coordinate U
        float barycentricV;        // Barycentric coordinate V  
        float distance;            // Hit distance
        uint32_t sourceVertex;     // Which deformable vertex
        uint32_t rayIndex;         // Which ray from this vertex
        uint32_t padding[2];       // Ensure 32-byte alignment
    };

    // ============================================================
    // === Helpers
    // ============================================================

    void SubmitReadbackCopy(uint32_t slot, VkSemaphore timeline, uint64_t waitValue, uint64_t signalValue);
    void CopyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size);

    VkDeviceAddress GetBufferAddress(const Buffer& buffer) const;
    uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags props) const;

    //void Cleanup();
    //void CleanupShaders();

    struct TraceSync {
        VkFence fence = VK_NULL_HANDLE;
        VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
        bool isTracing = false;
        uint64_t frameNumber = 0;
    };

    TraceSync _traceSync;
    VkFence _traceCompleteFence = VK_NULL_HANDLE;
    bool IsTraceComplete();
    void WaitForTrace();

    struct ReadbackData {
        Buffer hitBuffer;        // Device-local for GPU writes
        Buffer stagingBuffer;    // Host-visible for CPU read
        VkDeviceSize size = 0;
        void* mappedData = nullptr;
        bool isMapped = false;

        VkCommandBuffer copyCmd = VK_NULL_HANDLE;
        VkFence copyCompleteFence = VK_NULL_HANDLE;
    };

    // Single slot instead of array
    ReadbackData _readback;
    bool _hasPendingResults = false;

    void CreateReadbackResources();
    void SubmitReadback();
    void WaitForReadbackComplete();
	std::vector<SimpleHit> GetHitResults();
    void WriteHitsToFile(const std::string& filename, const std::vector<SimpleHit>& hits);
};