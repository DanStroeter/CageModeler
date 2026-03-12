#pragma once
#include <Rendering/PMVC/CubemapManager.h>
#include <Rendering/PMVC/IComputeStrategy.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/DescriptorPool.h>
#include <vulkan/vulkan.h>
#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <Mesh/GeometryUtils.h>
#include <Rendering/PMVC/CubemapManager.h>
//#include <Rendering/PMVC/CubemapRenderInstance.h>
#include <Rendering/PMVC/SphereWeightCalculator.h>
#include <Mesh/Operations/MeshWeightsParams.h>
#include <optional>


class CubemapManager;
struct CubemapWorkRange;

struct CubemapRenderTarget
{
	VkImage        cubemapImage;
	VkDeviceMemory cubemapMemory;
	VkImageView    cubemapView;
	std::array<VkImageView, 6> faceViews;

	VkImage        depthImage = VK_NULL_HANDLE;
	VkDeviceMemory depthMemory = VK_NULL_HANDLE;
	VkImageView    depthView = VK_NULL_HANDLE;
	std::array<VkImage, 2> depthImages{};
	std::array<VkDeviceMemory, 2> depthMemories{};
	std::array<VkImageView, 2> depthViewsArray{};
	std::array<std::array<VkImageView, 6>, 2> depthViews{};
	std::array<std::array<VkFramebuffer, 6>, 2> framebuffers{};
	std::array<VkDescriptorSet, 2> depthHistoryDescriptorSets{};
};

struct CubemapRenderUnit
{
	std::vector<CubemapRenderTarget> targets;

	std::vector<std::array<VkCommandBuffer, 6>> graphicsCmdPerTarget;

	MemoryMappedBuffer matricesUBO;
	VkDescriptorSet    matricesDescriptorSet;
};

struct CubemapMatricesUBO
{
	float invNumTriangles;
	float _pad[3];
};


class CubemapRenderInstance {
public:
	//CubemapRenderInstance() = delete;

	CubemapRenderInstance();//CubemapManager& cubemapManager);
	CubemapRenderInstance(
		CubemapManager& cubemapManager,
		int cubemapSize,
		VkFormat format,
		PMVCComputeType computeType,
		bool useOffset,
		uint32_t targetCount,

		RenderResourceRef<Device> device,
		RenderResourceRef<DescriptorPool> descriptorPool,
		std::shared_ptr<RenderResourceManager> resourceManager,
		std::shared_ptr<RenderPipelineManager> renderPipelineManager,

		EigenMesh cageMesh,
		EigenMesh deformableMesh,

		VkCommandPool graphicsCommandPool,
		VkRenderPass renderPass,
		VkRenderPass renderPassCpu,
		PipelineHandle cubemapPipelineHandle,
		PipelineHandle cubemapPipelineHandleCpu,
		PipelineHandle cubemapPipelineHitHandle,

		RenderResourceRef<DescriptorSetLayout> matricesLayout,
		RenderResourceRef<DescriptorSetLayout> depthHistoryLayout,

		MemoryMappedBuffer indexBuffer,
		MemoryMappedBuffer vertexBuffer
	);
	~CubemapRenderInstance();
	//CubemapRenderInstance(CubemapRenderInstance&) = default;
	//CubemapRenderInstance& operator=(CubemapRenderInstance&) = default;

	void ComputeCoordinatesGPUSerial(const CubemapWorkRange& range, Eigen::MatrixXd& weights);
	void ComputeCoordinatesGPUAtomic(const CubemapWorkRange& range, Eigen::MatrixXd& weights);
	void ComputeCoordinatesGPUMP(
		const CubemapWorkRange& range,
		Eigen::MatrixXd& weights);
	void ComputeCoordinates(const CubemapWorkRange& range, Eigen::MatrixXd& weights);
	[[nodiscard]] std::optional<double> GetRenderMs() const { return _renderMs; }
	[[nodiscard]] std::optional<double> GetComputeMs() const { return _computeMs; }
	[[nodiscard]] std::optional<double> GetComputeTotalMs() const { return _computeTotalMs; }
	[[nodiscard]] std::optional<double> GetTransferMs() const { return _transferMs; }
	void Cleanup();

private:
	//CubemapManager& _cubemapManager;

	std::unique_ptr<ICubemapComputeStrategy> _computeStage;
	
	//offset
	bool _pmvcUseOffset;

	//parameters 
	unsigned int _cubemapSize;
	uint32_t _targetCount = 64;
	VkFormat _format;
	PMVCComputeType _computeType;
	std::optional<double> _renderMs;
	std::optional<double> _computeMs;
	std::optional<double> _computeTotalMs;
	std::optional<double> _transferMs;

	//init functions
	void Initialize();
	CubemapRenderTarget CreateCubemapRenderTarget() const;
	CubemapRenderUnit CreateCubemapRenderUnit() const;

	void CreateCommandPool(uint32_t queueFamilyIndex);
	void UpdateMatricesDescriptorSet();
	void CreateSyncObjects();

	void RecordAndSubmitCubemapRender(uint32_t cubemapIdx, uint32_t targetIndex, const glm::vec3& camPos,
		CubemapRenderTarget& target, VkSemaphore timeline, uint64_t signalValue, uint32_t hitIndex); //TODO submit cubemap at once not in 6 parts
	std::vector<glm::vec3> BuildDeformableVertexPositions() const;

	void ComputeCoordinatesCpu(
		const CubemapWorkRange& range, Eigen::MatrixXd& weights);

	//render resources
	CubemapRenderUnit _cubemapRenderUnit;
	VkCommandPool _graphicCommandPool;

	//sync objects
	std::vector<uint64_t> _slotDoneValue;
	std::vector <VkSemaphore> _timelines = {};
	glm::mat4 ComputeCubemapViewMatrix(uint32_t faceIndex, const glm::vec3& pos);
	uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const;
	//friend class GpuSerialComputeStrategy;
	SphereWeightCalculator _sphereWeightCalculator;

	//from manager
	RenderResourceRef<Device> _device;
	RenderResourceRef < DescriptorPool> _descriptorPool;
	std::shared_ptr<RenderResourceManager> _resourceManager;
	std::shared_ptr<RenderPipelineManager> _renderPipelineManager;
	EigenMesh _cageMesh;
	EigenMesh _deformableMesh;
	VkCommandPool _graphicsCommandPool;
	VkRenderPass _renderPass;
	VkRenderPass _renderPassCpu;
	PipelineHandle _cubemapPipelineHandle;
	PipelineHandle _cubemapPipelineHandleCpu;
	PipelineHandle _cubemapPipelineHitHandle;
	RenderResourceRef<DescriptorSetLayout>  _matricesLayout;
	RenderResourceRef<DescriptorSetLayout>  _depthHistoryLayout;
	MemoryMappedBuffer _indexBuffer;
	MemoryMappedBuffer _vertexBuffer;
	VkSampler _depthHistorySampler = VK_NULL_HANDLE;
	uint32_t _allModeHitCount = 3;
	bool _omitEverySecondHit = false;
};