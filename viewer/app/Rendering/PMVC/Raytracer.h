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
#include <external/nvvk/acceleration_structures.hpp>

class Raytracer {
public:
	Raytracer(const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::shared_ptr<RenderResourceManager>& resourceManager, const RenderResourceRef<Device> device, const RenderResourceRef<Instance> instance, uint32_t cubemapSize, VkFormat format);
	~Raytracer();
		
	void Initialize();

	MeshOperationResult<MeshComputeWeightsOperationResult> ComputeCoordinates();
	void TraceRays();

	void SetCage(const EigenMesh& mesh) { _cageMesh = mesh; }
	void SetMesh(const EigenMesh& mesh) { _deformableMesh = mesh; }

private:
	//init functions:
	void GetRaytracingComponents();
	void CreateRaytracingPipeline();
	void PrimitiveToGeometry(EigenMesh& eigenMesh,
		VkAccelerationStructureGeometryKHR& geometry,
		VkAccelerationStructureBuildRangeInfoKHR& rangeInfo);

	void CreateVertexBufferFromMesh();
	void CreateIndexBufferFromMesh()


	//resources:
	RenderResourceRef<Device> _device;
	RenderResourceRef<Instance> _instance;
	SubsystemPtr<RenderSubsystem> _renderSubsystem = nullptr;
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr;
	std::shared_ptr<RenderResourceManager> _resourceManager = nullptr;

	//geodata:
	EigenMesh _cageMesh;
	EigenMesh _deformableMesh;

	//pipeline
	PipelineHandle _raytracingPipelineHandle;

	//buffers
	MemoryMappedBuffer _indexBuffer;
	MemoryMappedBuffer _vertexBuffer;

	//descriptors
	//RenderResourceRef < DescriptorPool> _descriptorPool;
	//RenderResourceRef < DescriptorSetLayout> _matricesLayout;

	void CreateAccelerationStructure(VkAccelerationStructureTypeKHR asType,  // The type of acceleration structure (BLAS or TLAS)
		nvvk::AccelerationStructure& accelStruct,  // The acceleration structure to create
		VkAccelerationStructureGeometryKHR& asGeometry,  // The geometry to build the acceleration structure from
		VkAccelerationStructureBuildRangeInfoKHR& asBuildRangeInfo,  // The range info for building the acceleration structure
		VkBuildAccelerationStructureFlagsKHR flags  // Build flags (e.g. prefer fast trace)
	);

	void CreateBottomLevelAS();  // Set up BLAS infrastructure
	void CreateTopLevelAS();

	// Ray Tracing Pipeline Components
	nvvk::DescriptorPack m_rtDescPack;               // Ray tracing descriptor bindings
	VkPipeline           m_rtPipeline{};             // Ray tracing pipeline
	VkPipelineLayout     m_rtPipelineLayout{};       // Ray tracing pipeline layout

	// Acceleration Structure Components
	std::vector<nvvk::AccelerationStructure> m_blasAccel;     // Bottom-level acceleration structures
	nvvk::AccelerationStructure              m_tlasAccel;     // Top-level acceleration structure

	// Direct SBT management
	nvvk::Buffer                    m_sbtBuffer;         // Buffer for shader binding table
	std::vector<uint8_t>            m_shaderHandles;     // Storage for shader group handles
	VkStridedDeviceAddressRegionKHR m_raygenRegion{};    // Ray generation shader region
	VkStridedDeviceAddressRegionKHR m_missRegion{};      // Miss shader region
	VkStridedDeviceAddressRegionKHR m_hitRegion{};       // Hit shader region
	VkStridedDeviceAddressRegionKHR m_callableRegion{};  // Callable shader region

	// Ray Tracing Properties
	VkPhysicalDeviceRayTracingPipelinePropertiesKHR m_rtProperties{
		VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR };
	VkPhysicalDeviceAccelerationStructurePropertiesKHR m_asProperties{
		VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_PROPERTIES_KHR };
};