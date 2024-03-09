#pragma once

#include <Rendering/Core/RenderProxy.h>
#include <Rendering/Core/Device.h>
#include <Rendering/Core/DescriptorPool.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/AlignedVector.h>
#include <Rendering/RenderPipelineManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/GeometryUtils.h>
#include <Editor/Light.h>

#include <Eigen/Core>

struct ViewInfo;
class ScreenPass;
class PolygonMesh;
class RenderProxyCollector;
class RenderResourceManager;
class RenderCommandScheduler;

class SceneRenderer
{
public:
	SceneRenderer() = delete;
	SceneRenderer(const RenderResourceRef<Device>& device,
		const RenderResourceRef<DescriptorPool>& descriptorPool,
		const std::shared_ptr<RenderPipelineManager>& renderPipelineManager,
		const std::shared_ptr<RenderProxyCollector>& renderProxyCollector,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
		const std::shared_ptr<RenderResourceManager>& resourceManager,
		const VkRenderPass& renderPass);

	void Initialize();

	[[nodiscard]] std::shared_ptr<PolygonMesh> AddCage(const Eigen::MatrixXd& vertices,
		const Eigen::MatrixXi& indices);

	[[nodiscard]] std::shared_ptr<PolygonMesh> AddMesh(const Eigen::MatrixXd& vertices,
		const Eigen::MatrixXi& indices);

	[[nodiscard]] std::shared_ptr<PolygonMesh> AddGizmo(const MeshGeometry& geom);

	void RemoveMesh(const std::shared_ptr<PolygonMesh>& mesh);

	void AddLightSource(const PointLightGPU& light);

	void CreateResources();

	void Render(const double deltaTime, const uint32_t currentFrameIndex, const ViewInfo& viewInfo);

private:
	void CreateScreenPasses();
	void CreateDescriptorSetLayouts();
	void CreateRenderPipelines();

	void CreateBackgroundPipeline();
	void CreateStaticMeshPipeline();
	void CreateCagePipeline();
	void CreateWireframePipeline();
	void CreatePointsPipeline();
	void CreateGizmoPipeline();
	void CreateViewportGridPipeline();

	void AllocateDescriptorSets();
	void CreateUniformBuffers();

private:
	/// The logical render device.
	RenderResourceRef<Device> _device;

	/// The render command scheduler.
	std::shared_ptr<RenderCommandScheduler> _renderCommandScheduler = nullptr;

	/// The proxy collector where we register render proxies.
	std::shared_ptr<RenderProxyCollector> _renderProxyCollector = nullptr;

	/// The render pipeline manager to add the scnee graphics pipelines.
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager = nullptr;

	/// Pointer to the resource manager.
	std::shared_ptr<RenderResourceManager> _resourceManager = nullptr;

	/// The Vulkan descriptor pool resource.
	RenderResourceRef<DescriptorPool> _descriptorPool;

	/// The render pass we use for rendering the entire scene. It is created by the render subsystem and passed to the editor.
	VkRenderPass _renderPass = VK_NULL_HANDLE;

	RenderResourceRef<DescriptorSetLayout> _matricesLayout;
	std::vector<VkDescriptorSet> _matricesDescriptorSets;
	std::vector<MemoryMappedBuffer> _matricesUniformBuffers;

	RenderResourceRef<DescriptorSetLayout> _objectDataLayout;
	std::vector<VkDescriptorSet> _objectDataDescriptorSets;
	std::vector<MemoryMappedBuffer> _objectsDynamicUniformBuffers;

	/// A buffer of all objects data.
	AlignedDeviceVector<ModelInfo> _objectsBufferData;

	RenderResourceRef<DescriptorSetLayout> _lightsLayout;
	std::vector<VkDescriptorSet> _lightsDescriptorSets;
	std::vector<MemoryMappedBuffer> _lightsUniformBuffers;

	uint32_t _objectsBufferDynamicAlignment;

	/// Pipeline to use to render the background gradient.
	PipelineHandle _backgroundPipelineHandle;

	/// Pipeline to use for the grid rendering.
	PipelineHandle _gridPipelineHandle;

	/// Pipeline for all static meshes.
	PipelineHandle _staticMeshPipelineHandle;

	/// Pipeline for meshes which are rendered with their influence map as vertex color.
	PipelineHandle _staticMeshInfluenceMapPipelineHandle;

	/// Pipeline for all polygon meshes with wireframe.
	PipelineHandle _cagePipelineHandle;

	/// Pipeline for wireframe rendering of meshes.
	PipelineHandle _edgesPipelineHandle;

	/// Pipeline for rendering of points of the mesh.
	PipelineHandle _pointsPipelineHandle;

	/// Pipeline for rendering of gizmos in the scene.
	PipelineHandle _gizmoPipelineHandle;

	std::vector<std::shared_ptr<ScreenPass>> _screenPasses;

	std::vector<PointLightGPU> _lightSources;
};
