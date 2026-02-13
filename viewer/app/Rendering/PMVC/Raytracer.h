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
        RenderResourceRef<Device> device,
        uint32_t queueFamilyIndex,
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

    ShaderBindingTable _sbt;

    // ============================================================
    // === Ray Tracing Working Buffers
    // ============================================================

    struct RayBuffers
    {
        Buffer rayDirections;               // DEVICE_LOCAL
        MemoryMappedBuffer hitBuffer;       // GPU write -> CPU read
        MemoryMappedBuffer mvcWeights;      // GPU write -> CPU read
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
    void CreateRayTracingPipeline();
    void CreateShaderBindingTable();
    void CreateRayBuffers();
    void CreateOutputImage();
    void UpdateDescriptorSet();

    // ============================================================
    // === Helpers
    // ============================================================

    VkDeviceAddress GetBufferAddress(const Buffer& buffer) const;
    uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags props) const;

    void Cleanup();
};