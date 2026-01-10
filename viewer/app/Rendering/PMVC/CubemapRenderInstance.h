#pragma once
#include <Rendering/PMVC/CubemapManager.h>
#include <Rendering/PMVC/IComputeStrategy.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Core/Buffer.h>
#include <vulkan/vulkan.h>

#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>


class CubemapManager;
struct CubemapWorkRange;

class SphereWeightCalculator {
public:
	VkImage        _solidAngleImage = VK_NULL_HANDLE;
	VkDeviceMemory _solidAngleMemory = VK_NULL_HANDLE;
	VkImageView    _solidAngleArrayView = VK_NULL_HANDLE;
	VkSampler      _solidAngleSampler = VK_NULL_HANDLE;
	SphereWeightCalculator() = default;
	void SphereWeightInitialization(uint32_t size, RenderResourceRef<Device> device, std::shared_ptr<RenderResourceManager> resourceManager, VkCommandPool commandPool);
	float ComputeSphereWeight(int px, int py, int faceSize, int face);
};

struct CubemapRenderTarget
{
	VkImage        cubemapImage;
	VkDeviceMemory cubemapMemory;
	VkImageView    cubemapView;
	std::array<VkImageView, 6> faceViews;

	VkImage        depthImage;
	VkDeviceMemory depthMemory;
	std::array<VkImageView, 6> depthViews;

	std::array<VkFramebuffer, 6> framebuffers;
};

struct CubemapRenderUnit
{
	std::vector<CubemapRenderTarget> targets;

	std::array<VkCommandBuffer, 6> graphicsCmd;

	MemoryMappedBuffer matricesUBO;
	VkDescriptorSet    matricesDescriptorSet;
};

enum class ComputeType {
	CPU,
	GPUATOMIC,
	//GPUSORT,
	DEBUGCUBEMAPS, // for debugging writes cupemaps to disk no compute
	GPUSERIAL 
	// TODO: implement if time leftover
};


struct CubemapMatricesUBO
{
	float invNumTriangles;
	float _pad[3];
};


class CubemapRenderInstance {
public:
	//CubemapRenderInstance() = delete;

	CubemapRenderInstance(CubemapManager& cubemapManager);
	CubemapRenderInstance(CubemapManager& cubemapManager, int cubemapSize, VkFormat format, ComputeType computeType);
	~CubemapRenderInstance();
	//CubemapRenderInstance(CubemapRenderInstance&) = default;
	//CubemapRenderInstance& operator=(CubemapRenderInstance&) = default;

	void ComputeCoordinatesGPUSerial(const CubemapWorkRange& range);

private:
	CubemapManager& _cubemapManager;

	std::unique_ptr<ICubemapComputeStrategy> _computeStage;
	
	//parameters 
	unsigned int _cubemapSize;
	VkFormat _format;
	ComputeType _computeType;

	//init functions
	void Initalize();
	CubemapRenderTarget CreateCubemapRenderTarget() const;
	CubemapRenderUnit CreateCubemapRenderUnit() const;

	void CreateCommandPool(uint32_t queueFamilyIndex);
	void UpdateMatricesDescriptorSet();
	void CreateSyncObjects();

	void RecordAndSubmitCubemapRender(uint32_t cubemapIdx, const glm::vec3& camPos, 
		CubemapRenderTarget& target, VkSemaphore timeline, uint64_t signalValue); //TODO submit cubemap at once not in 6 parts
	std::vector<glm::vec3> BuildDeformableVertexPositions() const;

	//render resources
	CubemapRenderUnit _cubemapRenderUnit;
	VkCommandPool _graphicCommandPool;

	//sync objects
	std::vector<uint64_t> _slotDoneValue;
	std::vector <VkSemaphore> _timelines = {};

	friend class GpuSerialComputeStrategy;
	SphereWeightCalculator _sphereWeightCalculator;
};
