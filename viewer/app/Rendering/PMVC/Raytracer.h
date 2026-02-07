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
    void TraceRays();

    void SetCage(const EigenMesh& mesh) { _cageMesh = mesh; }
    void SetMesh(const EigenMesh& mesh) { _deformableMesh = mesh; }

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
};