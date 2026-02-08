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

class RenderPipelineManager;
class RenderResourceManager;
class RenderSubsystem;

struct alignas(16) RTVertex {
    glm::vec3 pos;
};

class Raytracer {
public:
    Raytracer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
        const std::shared_ptr<RenderResourceManager>& resourceManager,
        const RenderResourceRef<Device> device,
        const RenderResourceRef<Instance> instance,
        uint32_t cubemapSize,
        VkFormat format);
    Raytracer();
    ~Raytracer();

    void Initialize();
    MeshOperationResult<MeshComputeWeightsOperationResult> ComputeCoordinates();
    //void TraceRays();

    void SetCage(const EigenMesh& mesh) { _cageMesh = mesh; }
    void SetMesh(const EigenMesh& mesh) { _deformableMesh = mesh; }

    void TraceRays();
    void ComputeMVCCoordinates();

private:
    enum BindingPoints
    {
        eOutImage = 0,
        eAccelStruct = 1,
    };
    // init functions
    void GetRaytracingComponents();
    void CreateRaytracingPipeline();
    void PrimitiveToGeometry(Buffer& vertexBuffer,
        Buffer& indexBuffer,
        uint32_t vertexCount,
        uint32_t triangleCount,
        VkAccelerationStructureGeometryKHR& geometry,
        VkAccelerationStructureBuildRangeInfoKHR& rangeInfo);
    void CopyBuffer(const Buffer& src, Buffer& dst, VkDeviceSize size);

    void CreateVertexBufferFromMesh();
    void CreateIndexBufferFromMesh();
    void CreateRaytraceDescriptorLayout();
    void CreateShaderBindingTable();

    void OnDetach();

    VkDescriptorSetLayout _rtDescSetLayout;

    // resources
    RenderResourceRef<Device> _device;
    RenderResourceRef<Instance> _instance;
    SubsystemPtr<RenderSubsystem> _renderSubsystem = nullptr;
    std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr;
    std::shared_ptr<RenderResourceManager> _resourceManager = nullptr;
    VkQueue _queue;

    VkCommandPool _commandPool;

    // geodata
    EigenMesh _cageMesh;
    EigenMesh _deformableMesh;

    // pipeline
    PipelineHandle _raytracingPipelineHandle;

    // buffers
    Buffer _indexBuffer;
    Buffer _vertexBuffer;

    // descriptors
    // RenderResourceRef<DescriptorPool> _descriptorPool;
    // RenderResourceRef<DescriptorSetLayout> _matricesLayout;

    // --- Acceleration Structures ---

    void CreateAccelerationStructure(VkAccelerationStructureTypeKHR asType,
        VkAccelerationStructureKHR& accelStruct,
        VkDeviceMemory& accelMemory,
        Buffer& vertexBuffer,
        Buffer& indexBuffer,
        uint32_t vertexCount,
        uint32_t triangleCount,
        VkBuildAccelerationStructureFlagsKHR flags);

    void CreateCommandPool(uint32_t queueFamilyIndex);
    void CreateBottomLevelAS();
    //void CreateTopLevelAS();
    void UploadVertexAndIndexBuffers();

    VkAccelerationStructureKHR m_blasAccel; // Bottom-level acceleration structures
    VkDeviceMemory m_blasMemory;            // Memory for BLAS
    //VkAccelerationStructureKHR m_tlasAccel = VK_NULL_HANDLE; // Top-level acceleration structure
    //VkDeviceMemory m_tlasMemory = VK_NULL_HANDLE;            // TLAS memory

    // Ray Tracing Pipeline Components
    VkPipeline       m_rtPipeline{};             // Ray tracing pipeline
    VkPipelineLayout m_rtPipelineLayout{};       // Ray tracing pipeline layout
    VkDescriptorSet  m_rtDescriptorSet{};        // Descriptor set

    // Direct SBT management
    MemoryMappedBuffer _sbtBuffer;              // Shader Binding Table buffer
    std::vector<uint8_t> _shaderHandles;        // Shader handles
    VkStridedDeviceAddressRegionKHR m_raygenRegion{};
    VkStridedDeviceAddressRegionKHR m_missRegion{};
    VkStridedDeviceAddressRegionKHR m_hitRegion{};
    VkStridedDeviceAddressRegionKHR m_callableRegion{};

    // Ray Tracing Properties
    VkPhysicalDeviceRayTracingPipelinePropertiesKHR m_rtProperties{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR
    };
    VkPhysicalDeviceAccelerationStructurePropertiesKHR m_asProperties{
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR
    };
    uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;



    // New members for ray tracing setup
    struct PushConstants {
        uint32_t maxHitsPerRay = 8;
        uint32_t skipEveryNthHit = 0; // 0 = no skip, 1 = skip every other
        uint32_t vertexCount = 0;
        uint32_t raysPerVertex = 256; // Number of rays per vertex for Monte Carlo
    };

    // Buffers for ray tracing
    Buffer _deformableVertexBuffer;
    Buffer _rayDirectionsBuffer;
    Buffer _hitBuffer;
    Buffer _cageIndexBuffer;

    // For MVC calculation
    Buffer _mvcWeightsBuffer;

    // Shader modules
    VkShaderModule _raygenShader = VK_NULL_HANDLE;
    VkShaderModule _missShader = VK_NULL_HANDLE;
    VkShaderModule _hitShader = VK_NULL_HANDLE;

    // Shader file paths
    std::filesystem::path _shaderDir = "assets/shaders/";

    // Shader loading functions
    std::vector<uint32_t> LoadSPIRV(const std::string& filename);
    VkShaderModule CreateShaderModule(const std::vector<uint32_t>& code);
    void LoadShaders();
    void CleanupShaders();

    void CreateRayTracingBuffers();
    void CreateRayTracingDescriptorSet();
    void SetupRayDirections();

    void SetupRayDirections();

    void CreateRayTracingBuffers();

    void CreateRayTracingDescriptorSet();

    void ReadHitData();

    // MVC calculation
    void ProcessHitsForMVC();

    // Configuration
    PushConstants _pushConstants;

    struct GLSLHitRecord {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec2 barycentric;
        uint32_t triangleId;
        uint32_t vertexIndices[3];
        float distance;
        uint32_t rayIndex;
        uint32_t hitSequence;

        // For CPU-side alignment
        static_assert(sizeof(GLSLHitRecord) == 64, "GLSLHitRecord must be tightly packed");
    };

    struct RayPayload {
        uint32_t vertexIndex;
        uint32_t rayIndex;
        uint32_t hitCount;
        float tMax;
    };

    struct PushConstants {
        uint32_t maxHitsPerRay = 8;
        uint32_t skipEveryNthHit = 0; // 0 = no skip, 1 = skip every other
        uint32_t vertexCount = 0;
        uint32_t raysPerVertex = 256;

        // Ensure 16-byte alignment for GLSL
        static_assert(sizeof(PushConstants) % 16 == 0, "PushConstants must be 16-byte aligned");
    };
    std::vector<GLSLHitRecord> _hits;
    Eigen::MatrixXd _mvcWeights;

    // Helper functions
    std::vector<GLSLHitRecord> ReadHitData();
    void ProcessHitsForMVC();
};