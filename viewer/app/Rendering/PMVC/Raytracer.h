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
#include <Rendering/RenderSubsystem.h> 
#include <Eigen/Core> 
#include <vector> 
#include <cstdint> 
#include <filesystem>

// Forward declarations
class RenderPipelineManager; 
class RenderResourceManager; 
class RenderSubsystem;

class Raytracer final
{
public:
    Raytracer(
        const std::shared_ptr<RenderPipelineManager>& pipelineManager,
        const std::shared_ptr<RenderResourceManager>& resourceManager,
        const RenderResourceRef<Device> device,
        uint32_t cubemapSize,
        VkFormat format);

    ~Raytracer();

    void Initialize();

    //void SetCage(const EigenMesh& mesh);
    //void SetDeformableMesh(const EigenMesh& mesh);

    void SetCage(const EigenMesh& mesh) { _cageMesh = mesh; } 
    void SetMesh(const EigenMesh& mesh) { _deformableMesh = mesh; }
    MeshOperationResult<MeshComputeWeightsOperationResult> ComputeCoordinates();

    void Trace();
    Eigen::MatrixXd ComputeMVC();

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
        MemoryMappedBuffer buffer;          // HOST_VISIBLE + DEVICE_ADDRESS
        VkStridedDeviceAddressRegionKHR raygen{};
        VkStridedDeviceAddressRegionKHR miss{};
        VkStridedDeviceAddressRegionKHR hit{};
        VkStridedDeviceAddressRegionKHR callable{};
    };

    VkShaderModule _raygenShader = VK_NULL_HANDLE;
    VkShaderModule _missShader = VK_NULL_HANDLE;
    VkShaderModule _hitShader = VK_NULL_HANDLE;

    ShaderBindingTable _sbt;

    // ============================================================
    // === Ray Tracing Working Buffers
    // ============================================================

    struct RayBuffers
    {
        Buffer rayDirections;               // DEVICE_LOCAL
        Buffer hitBuffer;       // GPU write -> CPU read
        Buffer mvcWeights;      // GPU write -> CPU read
        Buffer atomicCounterBuffer;
    };

    RayBuffers _rayBuffers;

    // ============================================================
    // === Output Image
    // ============================================================

    struct OutputImage
    {
        VkImage image = VK_NULL_HANDLE;
        VkImageView view = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
    };

    OutputImage _output;

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
    void CreateGeometryBuffers();
    void CreateAccelerationStructures();
    void CreateBLAS();
    void CreateTLAS();
    void BuildAccelerationStructure(
        VkAccelerationStructureGeometryKHR& geometry,
        uint32_t primitiveCount,
        VkAccelerationStructureTypeKHR type,
        AccelerationStructure& outAS);
    void CopyBuffer(const MemoryMappedBuffer& src, Buffer& dst, VkDeviceSize size);
    
    void CreateRayTracingPipeline();
    void CreateShaderBindingTable();
    void CreateRayBuffers();
    void CreateOutputImage();
    void UpdateDescriptorSet();

    void CreateRaytraceDescriptorLayout();
    void CreateRayTracingDescriptorSet();
    void SetupRayDirections();

    void LoadShaders();
    std::vector<uint32_t> LoadSPIRV(const std::string& filename);
    VkShaderModule CreateShaderModule(const std::vector<uint32_t>& code);
    //void GetRaytracingComponents();

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

    VkDeviceAddress GetBufferAddress(const Buffer& buffer) const;
    uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags props) const;

    void Cleanup();
    void CleanupShaders();
};