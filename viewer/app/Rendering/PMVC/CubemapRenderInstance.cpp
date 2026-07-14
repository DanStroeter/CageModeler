#include <Rendering/PMVC/CubemapRenderInstance.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Scene/SceneData.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/ScreenPass.h>
#include <Editor/Light.h>
#include <cstddef>
#include <Rendering/PMVC/CpuComputeStrategy.h>
#include <Rendering/PMVC/GpuAtomicComputeStrategy.h>
#include <Rendering/PMVC/CubemapManager.h>
#include <Rendering/PMVC/DebugCubemapComputeStrategy.h>
#include <Rendering/PMVC/ScopedCmdBuffer.h>
#include <Rendering/PMVC/GpuSerialComputeStrategy.h>
#include <Rendering/PMVC/GpuMassivelyParallelComputeStrategy.h>
#include <Mesh/Operations/MeshWeightsParams.h>
#include <Thread/ThreadPool.h>

#include <algorithm>
#include <exception>
#include <future>
#include <mutex>
#include <thread>
#include <chrono>
#include <limits>
#include <cmath>

namespace
{
	[[nodiscard]] glm::vec3 ToGlmVec3(const Eigen::Ref<const Eigen::RowVectorXd>& row)
	{
		return glm::vec3(
			static_cast<float>(row(0)),
			static_cast<float>(row(1)),
			static_cast<float>(row(2))
		);
	}

	[[nodiscard]] float PointToTriangleDistance(
		const glm::vec3& point,
		const glm::vec3& a,
		const glm::vec3& b,
		const glm::vec3& c)
	{
		const glm::vec3 ab = b - a;
		const glm::vec3 ac = c - a;
		const glm::vec3 ap = point - a;
		const float d1 = glm::dot(ab, ap);
		const float d2 = glm::dot(ac, ap);
		if (d1 <= 0.0f && d2 <= 0.0f)
			return glm::length(ap);

		const glm::vec3 bp = point - b;
		const float d3 = glm::dot(ab, bp);
		const float d4 = glm::dot(ac, bp);
		if (d3 >= 0.0f && d4 <= d3)
			return glm::length(bp);

		const float vc = d1 * d4 - d3 * d2;
		if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
		{
			const float v = d1 / (d1 - d3);
			return glm::length(point - (a + v * ab));
		}

		const glm::vec3 cp = point - c;
		const float d5 = glm::dot(ab, cp);
		const float d6 = glm::dot(ac, cp);
		if (d6 >= 0.0f && d5 <= d6)
			return glm::length(cp);

		const float vb = d5 * d2 - d1 * d6;
		if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
		{
			const float w = d2 / (d2 - d6);
			return glm::length(point - (a + w * ac));
		}

		const float va = d3 * d6 - d5 * d4;
		if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
		{
			const glm::vec3 bc = c - b;
			const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			return glm::length(point - (b + w * bc));
		}

		const glm::vec3 normal = glm::cross(ab, ac);
		const float normalLen = glm::length(normal);
		if (normalLen <= std::numeric_limits<float>::epsilon())
		{
			return std::min({ glm::length(ap), glm::length(bp), glm::length(cp) });
		}

		return std::abs(glm::dot(point - a, normal / normalLen));
	}
}

CubemapRenderInstance::~CubemapRenderInstance() {
	Cleanup();
}

CubemapRenderInstance::CubemapRenderInstance()
{
	_cubemapSize = 32;
	_format = VK_FORMAT_R32G32B32A32_SFLOAT;
	_computeType = PMVCComputeType::Serial;
	_pmvcUseOffset = false;
	_targetCount = 64;
	_hitCount = 3;
	_omitNegative = true;
	Initialize();
}

CubemapRenderInstance::CubemapRenderInstance(
	CubemapManager& cubemapManager,
	int cubemapSize,
	VkFormat format,
	PMVCComputeType computeType,
	bool useOffset,
	uint32_t targetCount,
	uint32_t hitCount,
	bool omitNegative,
	float alpha,
	float beta,
	float theta,
	const Eigen::MatrixXf* interiorDetours,

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
): _cubemapSize(cubemapSize)
, _format(format)
, _computeType(computeType)
, _pmvcUseOffset(useOffset)
, _targetCount(targetCount == 0 ? 1u : targetCount)
, _hitCount(hitCount == 0 ? 1u : hitCount)
, _omitNegative(omitNegative)
, _threeHitVariant(computeType == PMVCComputeType::ThreeHit)
, _alpha(alpha)
, _beta(beta)
, _theta(theta)
, _interiorDetours(interiorDetours)
, _device(std::move(device))
, _descriptorPool(std::move(descriptorPool))
, _resourceManager(std::move(resourceManager))
, _renderPipelineManager(std::move(renderPipelineManager))
, _cageMesh(std::move(cageMesh))
, _deformableMesh(std::move(deformableMesh))
, _graphicsCommandPool(graphicsCommandPool)
, _renderPass(renderPass)
, _renderPassCpu(renderPassCpu)
, _cubemapPipelineHandle(cubemapPipelineHandle)
, _cubemapPipelineHitHandle(cubemapPipelineHitHandle)
, _matricesLayout(std::move(matricesLayout))
, _depthHistoryLayout(std::move(depthHistoryLayout))
, _indexBuffer(std::move(indexBuffer))
, _vertexBuffer(std::move(vertexBuffer))
, _cubemapPipelineHandleCpu(cubemapPipelineHandleCpu)
{
	UpdateProjectionPlanes();
	Initialize();
}


void CubemapRenderInstance::Cleanup()
{
	if (!_device)
		return;

	vkDeviceWaitIdle(_device);

	if (_computeStage)
	{
		_computeStage->Cleanup();
		_computeStage.reset();
	}

	for (auto timeline : _timelines)
	{
		if (timeline != VK_NULL_HANDLE)
			vkDestroySemaphore(_device, timeline, nullptr);
	}
	_timelines.clear();

	if (_graphicCommandPool != VK_NULL_HANDLE)
	{
		vkDestroyCommandPool(_device, _graphicCommandPool, nullptr);
		_graphicCommandPool = VK_NULL_HANDLE;
	}

	if (_cubemapRenderUnit.matricesUBO._deviceBuffer != VK_NULL_HANDLE)
	{
		_cubemapRenderUnit.matricesUBO.ReleaseResource(_device);
		_cubemapRenderUnit.matricesUBO = MemoryMappedBuffer();
	}

	for (auto& target : _cubemapRenderUnit.targets)
	{
		for (const auto& framebufferSet : target.framebuffers)
		{
			for (auto framebuffer : framebufferSet)
			{
				if (framebuffer != VK_NULL_HANDLE)
					vkDestroyFramebuffer(_device, framebuffer, nullptr);
			}
		}
		for (auto view : target.faceViews)
		{
			if (view != VK_NULL_HANDLE)
				vkDestroyImageView(_device, view, nullptr);
		}
		for (auto view : target.faceViewsSecond)
		{
			if (view != VK_NULL_HANDLE)
				vkDestroyImageView(_device, view, nullptr);
		}
		for (const auto& depthViewSet : target.depthViews)
		{
			for (auto view : depthViewSet)
			{
				if (view != VK_NULL_HANDLE)
					vkDestroyImageView(_device, view, nullptr);
			}
		}

		if (target.cubemapView != VK_NULL_HANDLE)
			vkDestroyImageView(_device, target.cubemapView, nullptr);

		if (target.cubemapViewSecond != VK_NULL_HANDLE)
			vkDestroyImageView(_device, target.cubemapViewSecond, nullptr);

		for (auto depthViewArray : target.depthViewsArray)
		{
			if (depthViewArray != VK_NULL_HANDLE)
				vkDestroyImageView(_device, depthViewArray, nullptr);
		}

		if (target.cubemapImage != VK_NULL_HANDLE)
			vkDestroyImage(_device, target.cubemapImage, nullptr);
		if (target.cubemapMemory != VK_NULL_HANDLE)
			vkFreeMemory(_device, target.cubemapMemory, nullptr);

		if (target.cubemapImageSecond != VK_NULL_HANDLE)
			vkDestroyImage(_device, target.cubemapImageSecond, nullptr);
		if (target.cubemapMemorySecond != VK_NULL_HANDLE)
			vkFreeMemory(_device, target.cubemapMemorySecond, nullptr);

		for (auto depthImage : target.depthImages)
		{
			if (depthImage != VK_NULL_HANDLE)
				vkDestroyImage(_device, depthImage, nullptr);
		}
		for (auto depthMemory : target.depthMemories)
		{
			if (depthMemory != VK_NULL_HANDLE)
				vkFreeMemory(_device, depthMemory, nullptr);
		}
	}

	if (_depthHistorySampler != VK_NULL_HANDLE)
	{
		vkDestroySampler(_device, _depthHistorySampler, nullptr);
		_depthHistorySampler = VK_NULL_HANDLE;
	}

	_cubemapRenderUnit.targets.clear();
	_cubemapRenderUnit.graphicsCmdPerTarget.clear();
}

void CubemapRenderInstance::Initialize() {

	if (_computeType == PMVCComputeType::Serial) {
		_computeStage = std::make_unique<GpuSerialComputeStrategy>(
			_device,
			_device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_descriptorPool,
			_resourceManager,
			_renderPipelineManager,
			_cageMesh,
			_deformableMesh, 
			_pmvcUseOffset,
			_targetCount
		);
	}
	else if (_computeType == PMVCComputeType::Ring) {
		_computeStage = std::make_unique<GpuAtomicComputeStrategy>(
			_device,
			_device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_descriptorPool,
			_resourceManager,
			_renderPipelineManager,
			_cageMesh,
			_deformableMesh,
			_pmvcUseOffset,
			_targetCount
		);
	}
	else if (_computeType == PMVCComputeType::All) {
		_computeStage = std::make_unique<GpuMPComputeStrategy>(
			_device,
			_device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_descriptorPool,
			_resourceManager,
			_renderPipelineManager,
			_cageMesh,
			_deformableMesh,
			_pmvcUseOffset,
			_targetCount
		);
	}
	else if (_computeType == PMVCComputeType::ThreeHit) {
		_computeStage = std::make_unique<GpuMPComputeStrategy>(
			_device,
			_device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_descriptorPool,
			_resourceManager,
			_renderPipelineManager,
			_cageMesh,
			_deformableMesh,
			_pmvcUseOffset,
			_targetCount,
			true,
			_alpha,
			_beta,
			_theta
		);
	}
	else if (_computeType == PMVCComputeType::Cpu) {
		_computeStage = std::make_unique<CpuComputeStrategy>(
			_device,
			_device->GetQueueFamilies()._graphics.value(),
			_cubemapSize,
			_format,
			_descriptorPool,
			_resourceManager,
			_renderPipelineManager,
			_cageMesh,
			_deformableMesh,
			_pmvcUseOffset,
			_targetCount
		);
	}

	uint32_t graphicsQueueFamilyIndex = _device->GetQueueFamilies()._graphics.value();
	CreateCommandPool(graphicsQueueFamilyIndex);

	if (_interiorDetours != nullptr)
	{
		InteriorDistanceSettings interiorSettings{};
		interiorSettings.detours = _interiorDetours;
		interiorSettings.nearPlane = _projectionNearPlane;
		interiorSettings.farPlane = _projectionFarPlane;
		_computeStage->SetInteriorDistance(interiorSettings);
	}

	const uint32_t targetCount = _computeStage->RequiredRenderTargetCount();
	_computeStage->Initialize();

	VkSamplerCreateInfo depthSamplerInfo{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
	depthSamplerInfo.magFilter = VK_FILTER_NEAREST;
	depthSamplerInfo.minFilter = VK_FILTER_NEAREST;
	depthSamplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
	depthSamplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	depthSamplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	depthSamplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
	depthSamplerInfo.maxAnisotropy = 1.0f;
	VK_CHECK(vkCreateSampler(_device, &depthSamplerInfo, nullptr, &_depthHistorySampler));

	_cubemapRenderUnit.targets.reserve(targetCount);
	for (uint32_t i = 0; i < targetCount; ++i)
	{
		_cubemapRenderUnit.targets.push_back(CreateCubemapRenderTarget());
	}

	_cubemapRenderUnit = CreateCubemapRenderUnit();
	UpdateMatricesDescriptorSet();
	CreateSyncObjects();
	//_sphereWeightCalculator = SphereWeightCalculator();
	//_sphereWeightCalculator.SphereWeightInitialization(_cubemapSize, _cubemapManager._device, _cubemapManager._resourceManager, _graphicCommandPool);
}

void CubemapRenderInstance::UpdateProjectionPlanes()
{
	if (_cageMesh._vertices.rows() == 0 || _cageMesh._vertices.cols() < 3)
	{
		return;
	}

	float maxCageVertexDistance = 0.0f;
	for (int i = 0; i < _cageMesh._vertices.rows(); ++i)
	{
		const glm::vec3 a = ToGlmVec3(_cageMesh._vertices.row(i));
		for (int j = i + 1; j < _cageMesh._vertices.rows(); ++j)
		{
			const glm::vec3 b = ToGlmVec3(_cageMesh._vertices.row(j));
			maxCageVertexDistance = std::max(maxCageVertexDistance, glm::distance(a, b));
		}
	}

	const float computedFarPlane = std::max(maxCageVertexDistance, 1e-3f);
	_projectionFarPlane = computedFarPlane;

	if (_deformableMesh._vertices.rows() == 0 || _deformableMesh._vertices.cols() < 3 || _cageMesh._faces.cols() < 3)
	{
		_projectionNearPlane = 1e-4f;
		return;
	}

	float minMeshToCageTriangleDistance = std::numeric_limits<float>::max();
	for (int meshVertexIdx = 0; meshVertexIdx < _deformableMesh._vertices.rows(); ++meshVertexIdx)
	{
		const glm::vec3 point = ToGlmVec3(_deformableMesh._vertices.row(meshVertexIdx));
		for (int faceIdx = 0; faceIdx < _cageMesh._faces.rows(); ++faceIdx)
		{
			const int v0Idx = _cageMesh._faces(faceIdx, 0);
			const int v1Idx = _cageMesh._faces(faceIdx, 1);
			const int v2Idx = _cageMesh._faces(faceIdx, 2);
			if (v0Idx < 0 || v1Idx < 0 || v2Idx < 0 ||
				v0Idx >= _cageMesh._vertices.rows() ||
				v1Idx >= _cageMesh._vertices.rows() ||
				v2Idx >= _cageMesh._vertices.rows())
			{
				continue;
			}

			const glm::vec3 a = ToGlmVec3(_cageMesh._vertices.row(v0Idx));
			const glm::vec3 b = ToGlmVec3(_cageMesh._vertices.row(v1Idx));
			const glm::vec3 c = ToGlmVec3(_cageMesh._vertices.row(v2Idx));
			minMeshToCageTriangleDistance = std::min(
				minMeshToCageTriangleDistance,
				PointToTriangleDistance(point, a, b, c));
		}
	}

	if (!std::isfinite(minMeshToCageTriangleDistance))
	{
		_projectionNearPlane = 1e-4f;
		return;
	}

	const float epsilon = 1e-4f;
	_projectionNearPlane = std::max(minMeshToCageTriangleDistance, epsilon);
	if (_projectionNearPlane >= _projectionFarPlane)
	{
		_projectionNearPlane = std::max(_projectionFarPlane * 0.001f, epsilon);
	}
}

CubemapRenderTarget CubemapRenderInstance::CreateCubemapRenderTarget() const
{
	CubemapRenderTarget target{};
	//target.currentLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	// ---------------------------------------------------------------------
	// Create cubemap color image
	// ---------------------------------------------------------------------
	VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
	imageInfo.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
	imageInfo.imageType = VK_IMAGE_TYPE_2D;
	imageInfo.format = _format;
	imageInfo.extent = { _cubemapSize, _cubemapSize, 1 };
	imageInfo.mipLevels = 1;
	imageInfo.arrayLayers = 6;
	imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	imageInfo.usage =
		VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
		VK_IMAGE_USAGE_SAMPLED_BIT |
		VK_IMAGE_USAGE_STORAGE_BIT |
		VK_IMAGE_USAGE_TRANSFER_SRC_BIT; //TODO: only added for debugging prints for image remove after done(needed for CPU compute?)
	imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	VK_CHECK(vkCreateImage(_device, &imageInfo, nullptr, &target.cubemapImage));

	VkMemoryRequirements memReq{};
	vkGetImageMemoryRequirements(_device, target.cubemapImage, &memReq);

	VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
	allocInfo.allocationSize = memReq.size;
	allocInfo.memoryTypeIndex = FindMemoryType(memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	VK_CHECK(vkAllocateMemory(_device, &allocInfo, nullptr, &target.cubemapMemory));
	VK_CHECK(vkBindImageMemory(_device, target.cubemapImage, target.cubemapMemory, 0));


	/*VkCommandBufferAllocateInfo allocInfoCB{
	.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
	.commandPool = _graphicCommandPool, // or graphics pool
	.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
	.commandBufferCount = 1
	};

	VkCommandBuffer cmd;
	VK_CHECK(vkAllocateCommandBuffers(_device, &allocInfoCB, &cmd));

	VkCommandBufferBeginInfo beginInfo{
	.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
	.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
	};

	VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));
	*/
	// ---------------------------------------------------------------------
	// Create per-face color views
	// ---------------------------------------------------------------------
	for (uint32_t face = 0; face < 6; ++face)
	{
		VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
		viewInfo.image = target.cubemapImage;
		viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		viewInfo.format = _format;
		viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		viewInfo.subresourceRange.levelCount = 1;
		viewInfo.subresourceRange.baseArrayLayer = face;
		viewInfo.subresourceRange.layerCount = 1;

		VK_CHECK(vkCreateImageView(_device, &viewInfo, nullptr, &target.faceViews[face]));
	}

	// ---------------------------------------------------------------------
	// Create cubemap view (for sampling)
	// ---------------------------------------------------------------------
	/*VkImageViewCreateInfo cubeViewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
	cubeViewInfo.image = target.cubemapImage;
	cubeViewInfo.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
	cubeViewInfo.format = _format;
	cubeViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	cubeViewInfo.subresourceRange.levelCount = 1;
	cubeViewInfo.subresourceRange.layerCount = 6;

	VK_CHECK(vkCreateImageView(_cubemapManager._device, &cubeViewInfo, nullptr, &target.cubemapView));*/
	VkImageViewCreateInfo cubeViewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
	cubeViewInfo.image = target.cubemapImage;
	cubeViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
	cubeViewInfo.format = _format;
	cubeViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	cubeViewInfo.subresourceRange.baseMipLevel = 0;
	cubeViewInfo.subresourceRange.levelCount = 1;
	cubeViewInfo.subresourceRange.baseArrayLayer = 0;
	cubeViewInfo.subresourceRange.layerCount = 6;

	VK_CHECK(vkCreateImageView(_device, &cubeViewInfo, nullptr, &target.cubemapView));

	// ---------------------------------------------------------------------
	// Create the second color cubemap (three-hit variant only).
	// The first hit color must survive while the second hit is rendered, so we
	// render the two hits into two distinct color images and combine them in the
	// compute shader afterwards.
	// ---------------------------------------------------------------------
	if (_threeHitVariant)
	{
		VK_CHECK(vkCreateImage(_device, &imageInfo, nullptr, &target.cubemapImageSecond));

		VkMemoryRequirements memReqSecond{};
		vkGetImageMemoryRequirements(_device, target.cubemapImageSecond, &memReqSecond);

		VkMemoryAllocateInfo allocInfoSecond{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
		allocInfoSecond.allocationSize = memReqSecond.size;
		allocInfoSecond.memoryTypeIndex = FindMemoryType(memReqSecond.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

		VK_CHECK(vkAllocateMemory(_device, &allocInfoSecond, nullptr, &target.cubemapMemorySecond));
		VK_CHECK(vkBindImageMemory(_device, target.cubemapImageSecond, target.cubemapMemorySecond, 0));

		for (uint32_t face = 0; face < 6; ++face)
		{
			VkImageViewCreateInfo viewInfoSecond{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
			viewInfoSecond.image = target.cubemapImageSecond;
			viewInfoSecond.viewType = VK_IMAGE_VIEW_TYPE_2D;
			viewInfoSecond.format = _format;
			viewInfoSecond.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			viewInfoSecond.subresourceRange.levelCount = 1;
			viewInfoSecond.subresourceRange.baseArrayLayer = face;
			viewInfoSecond.subresourceRange.layerCount = 1;
			VK_CHECK(vkCreateImageView(_device, &viewInfoSecond, nullptr, &target.faceViewsSecond[face]));
		}

		VkImageViewCreateInfo cubeViewInfoSecond{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
		cubeViewInfoSecond.image = target.cubemapImageSecond;
		cubeViewInfoSecond.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
		cubeViewInfoSecond.format = _format;
		cubeViewInfoSecond.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		cubeViewInfoSecond.subresourceRange.baseMipLevel = 0;
		cubeViewInfoSecond.subresourceRange.levelCount = 1;
		cubeViewInfoSecond.subresourceRange.baseArrayLayer = 0;
		cubeViewInfoSecond.subresourceRange.layerCount = 6;
		VK_CHECK(vkCreateImageView(_device, &cubeViewInfoSecond, nullptr, &target.cubemapViewSecond));
	}

	// ---------------------------------------------------------------------
	// Create depth images (ping-pong)
	// ---------------------------------------------------------------------
	VkFormat depthFormat = _device->FindDepthFormat();

	VkImageCreateInfo depthInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
	depthInfo.imageType = VK_IMAGE_TYPE_2D;
	depthInfo.format = depthFormat;
	depthInfo.extent = { _cubemapSize, _cubemapSize, 1 };
	depthInfo.mipLevels = 1;
	depthInfo.arrayLayers = 6;
	depthInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	depthInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	depthInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT |
		VK_IMAGE_USAGE_SAMPLED_BIT |
		VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
	depthInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	for (uint32_t pingPong = 0; pingPong < 2; ++pingPong)
	{
		VK_CHECK(vkCreateImage(_device, &depthInfo, nullptr, &target.depthImages[pingPong]));

		vkGetImageMemoryRequirements(_device, target.depthImages[pingPong], &memReq);
		allocInfo.allocationSize = memReq.size;
		allocInfo.memoryTypeIndex = FindMemoryType(memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
		VK_CHECK(vkAllocateMemory(_device, &allocInfo, nullptr, &target.depthMemories[pingPong]));
		VK_CHECK(vkBindImageMemory(_device, target.depthImages[pingPong], target.depthMemories[pingPong], 0));

		for (uint32_t face = 0; face < 6; ++face)
		{
			VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
			viewInfo.image = target.depthImages[pingPong];
			viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
			viewInfo.format = depthFormat;
			viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
			viewInfo.subresourceRange.levelCount = 1;
			viewInfo.subresourceRange.baseArrayLayer = face;
			viewInfo.subresourceRange.layerCount = 1;
			VK_CHECK(vkCreateImageView(_device, &viewInfo, nullptr, &target.depthViews[pingPong][face]));
		}

		VkImageViewCreateInfo depthViewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
		depthViewInfo.image = target.depthImages[pingPong];
		depthViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
		depthViewInfo.format = depthFormat;
		depthViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
		depthViewInfo.subresourceRange.baseMipLevel = 0;
		depthViewInfo.subresourceRange.levelCount = 1;
		depthViewInfo.subresourceRange.baseArrayLayer = 0;
		depthViewInfo.subresourceRange.layerCount = 6;
		VK_CHECK(vkCreateImageView(_device, &depthViewInfo, nullptr, &target.depthViewsArray[pingPong]));
	}

	std::array<VkDescriptorSetLayout, 2> depthLayouts{
		_depthHistoryLayout->GetReference(),
		_depthHistoryLayout->GetReference()
	};
	VkDescriptorSetAllocateInfo depthAllocInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	depthAllocInfo.descriptorPool = _descriptorPool;
	depthAllocInfo.descriptorSetCount = static_cast<uint32_t>(depthLayouts.size());
	depthAllocInfo.pSetLayouts = depthLayouts.data();
	VK_CHECK(vkAllocateDescriptorSets(_device, &depthAllocInfo, target.depthHistoryDescriptorSets.data()));

	for (uint32_t pingPong = 0; pingPong < 2; ++pingPong)
	{
		const uint32_t historyIndex = 1u - pingPong;
		VkDescriptorImageInfo imageInfo{};
		imageInfo.sampler = _depthHistorySampler;
		imageInfo.imageView = target.depthViewsArray[historyIndex];
		imageInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

		VkWriteDescriptorSet write{};
		write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		write.dstSet = target.depthHistoryDescriptorSets[pingPong];
		write.dstBinding = 0;
		write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		write.descriptorCount = 1;
		write.pImageInfo = &imageInfo;
		vkUpdateDescriptorSets(_device, 1, &write, 0, nullptr);
	}

	target.depthImage = target.depthImages[0];
	target.depthMemory = target.depthMemories[0];
	target.depthView = target.depthViewsArray[0];

	for (uint32_t pingPong = 0; pingPong < 2; ++pingPong)
	{
		for (uint32_t face = 0; face < 6; ++face)
		{
			// In the three-hit variant the second ping-pong slot renders into the
			// dedicated second color image so the first hit is not overwritten.
			VkImageView colorAttachment =
				(_threeHitVariant && pingPong == 1)
				? target.faceViewsSecond[face]
				: target.faceViews[face];

			VkImageView attachments[2] = {
				colorAttachment,
				target.depthViews[pingPong][face]
			};

			VkFramebufferCreateInfo fbInfo{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
			fbInfo.renderPass = (_computeType == PMVCComputeType::Cpu) ? _renderPassCpu : _renderPass;
			fbInfo.attachmentCount = 2;
			fbInfo.pAttachments = attachments;
			fbInfo.width = _cubemapSize;
			fbInfo.height = _cubemapSize;
			fbInfo.layers = 1;

			VK_CHECK(vkCreateFramebuffer(_device, &fbInfo, nullptr, &target.framebuffers[pingPong][face]));
		}
	}

	return target;
}

CubemapRenderUnit CubemapRenderInstance::CreateCubemapRenderUnit() const
{
	VkDeviceSize matricesUBOSize = sizeof(CubemapMatricesUBO);
	CubemapRenderUnit unit{};
	unit.targets = _cubemapRenderUnit.targets;
	// ------------------------------------------------------------
	// Create uniform buffer
	// ------------------------------------------------------------
	std::span<std::byte> sizeSpan(
		static_cast<std::byte*>(nullptr),
		matricesUBOSize
	);

	unit.matricesUBO =
		_resourceManager->CreateBufferAndMapMemory(
			sizeSpan,
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		);

	// ------------------------------------------------------------
	// Allocate descriptor set
	// ------------------------------------------------------------
	VkDescriptorSetLayout layout = _matricesLayout->GetReference();

	VkDescriptorSetAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	allocInfo.descriptorPool = _descriptorPool;
	allocInfo.descriptorSetCount = 1;
	allocInfo.pSetLayouts = &layout;

	VK_CHECK(vkAllocateDescriptorSets(
		_device,
		&allocInfo,
		&unit.matricesDescriptorSet
	));

	// ------------------------------------------------------------
	// Allocate command buffers (one per face)
	// ------------------------------------------------------------
	/*std::array<VkCommandBuffer, 6> buffers{};

	VkCommandBufferAllocateInfo alloc{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		.commandPool = _graphicCommandPool,
		.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
		.commandBufferCount = (uint32_t)buffers.size()
	};

	VK_CHECK(vkAllocateCommandBuffers(
		_device,
		&alloc,
		buffers.data()));

	//unit.beginCmd = buffers[0];*/
	const uint32_t targetCount =
		static_cast<uint32_t>(unit.targets.size());
	unit.graphicsCmdPerTarget.resize(targetCount);

	/*for (uint32_t i = 0; i < 6; ++i)
		unit.graphicsCmd[i] = buffers[i];
		*/
	//unit.endCmd = buffers[7];

	if (targetCount > 0)
	{
		std::vector<VkCommandBuffer> flatBuffers(targetCount * 6);

		VkCommandBufferAllocateInfo alloc{
			.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
			.commandPool = _graphicCommandPool,
			.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
			.commandBufferCount = static_cast<uint32_t>(flatBuffers.size())
		};
		VK_CHECK(vkAllocateCommandBuffers(
			_device,
			&alloc,
			flatBuffers.data()));
		for (uint32_t target = 0; target < targetCount; ++target)
		{
			for (uint32_t face = 0; face < 6; ++face)
			{
				unit.graphicsCmdPerTarget[target][face] =
					flatBuffers[target * 6 + face];
			}
		}
	}
	/*VkCommandBufferAllocateInfo cmdAllocInfo{
		VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO
	};
	cmdAllocInfo.commandPool = _graphicCommandPool;
	cmdAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	cmdAllocInfo.commandBufferCount = 6;

	VK_CHECK(vkAllocateCommandBuffers(
		_cubemapManager._device,
		&cmdAllocInfo,
		unit.graphicsCmd.data()
	));*/

	return unit;
}

void CubemapRenderInstance::CreateCommandPool(uint32_t queueFamilyIndex) {
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_graphicCommandPool) != VK_SUCCESS) {
		throw std::runtime_error("Failed to create command pool!");
	}
}

void CubemapRenderInstance::UpdateMatricesDescriptorSet()
{
	VkDescriptorBufferInfo bufferInfo{};
	bufferInfo.buffer = _cubemapRenderUnit.matricesUBO._deviceBuffer;
	bufferInfo.offset = 0;
	bufferInfo.range = _cubemapRenderUnit.matricesUBO._allocatedSize;

	VkWriteDescriptorSet write{};
	write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
	write.dstSet = _cubemapRenderUnit.matricesDescriptorSet;
	write.dstBinding = 0;
	write.dstArrayElement = 0;
	write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	write.descriptorCount = 1;
	write.pBufferInfo = &bufferInfo;

	vkUpdateDescriptorSets(_device, 1, &write, 0, nullptr);
}

void CubemapRenderInstance::RecordAndSubmitCubemapRender(
	uint32_t cubemapIdx,
	uint32_t targetIndex,
	const glm::vec3& camPos,
	CubemapRenderTarget& target,
	VkSemaphore timeline,
	uint64_t signalValue,
	uint32_t hitIndex = 0)
{
	assert(targetIndex < _cubemapRenderUnit.graphicsCmdPerTarget.size());

	VkClearValue clearValues[2]{};
	clearValues[0].color = { {0.f, 0.f, 0.f, 1.f} };
	clearValues[1].depthStencil = { 1.f, 0 };

	VkCommandBufferBeginInfo beginInfo{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
		.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
	};
	
	PipelineHandle pipelineHandle = _cubemapPipelineHandle;
	if (_computeType == PMVCComputeType::Cpu)
		pipelineHandle = _cubemapPipelineHandleCpu;
	else if (hitIndex > 0)
		pipelineHandle = _cubemapPipelineHitHandle;

	const PipelineObject& pipelineObj =
		_renderPipelineManager->GetPipelineObject(
			pipelineHandle);

	// ---------------------------------------------------------------------
	// Update shared UBO (outside render loop)
	// ---------------------------------------------------------------------
	const uint32_t numTriangles =
		static_cast<uint32_t>(_cageMesh._faces.size() / 3);

	CubemapMatricesUBO ubo{};
	ubo.invNumTriangles = 1.0f / float(numTriangles);
	std::memcpy(
		_cubemapRenderUnit.matricesUBO._mappedData,
		&ubo,
		sizeof(ubo));

	// =====================================================================
	// GRAPHICS CMDS — one per face
	// =====================================================================
	for (uint32_t face = 0; face < 6; ++face)
	{
		VkCommandBuffer cmd = _cubemapRenderUnit.graphicsCmdPerTarget[targetIndex][face];
		VK_CHECK(vkResetCommandBuffer(cmd, 0));
		VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo));

		CubemapPushConstants push{};
		push.proj = glm::perspective(glm::radians(90.0f), 1.0f, _projectionNearPlane, _projectionFarPlane);
		push.proj[1][1] *= -1.0f;
		push.view = ComputeCubemapViewMatrix(face, camPos);
		push.faceIndex = static_cast<int>(face);

		vkCmdPushConstants(
			cmd,
			pipelineObj._pipelineLayout,
			VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
			0,
			sizeof(push),
			&push);

		VkRenderPass renderPass = _renderPass;
		if (_computeType == PMVCComputeType::Cpu)
			renderPass = _renderPassCpu;

		VkRenderPassBeginInfo rpInfo{
			.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO,
			.renderPass = renderPass,
			.framebuffer = target.framebuffers[hitIndex % 2][face],
			.renderArea = {{0, 0}, {_cubemapSize, _cubemapSize}},
			.clearValueCount = 2,
			.pClearValues = clearValues
		};

		vkCmdBeginRenderPass(cmd, &rpInfo, VK_SUBPASS_CONTENTS_INLINE);
		vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineObj._handle);

		VkBuffer vb[] = { _vertexBuffer._deviceBuffer };
		VkDeviceSize offs[] = { 0 };
		vkCmdBindVertexBuffers(cmd, 0, 1, vb, offs);
		vkCmdBindIndexBuffer(
			cmd,
			_indexBuffer._deviceBuffer,
			0,
			VK_INDEX_TYPE_UINT32);

		vkCmdBindDescriptorSets(
			cmd,
			VK_PIPELINE_BIND_POINT_GRAPHICS,
			pipelineObj._pipelineLayout,
			0, 1,
			&_cubemapRenderUnit.matricesDescriptorSet,
			0, nullptr);

		if (hitIndex > 0)
		{
			vkCmdBindDescriptorSets(
				cmd,
				VK_PIPELINE_BIND_POINT_GRAPHICS,
				pipelineObj._pipelineLayout,
				1, 1,
				&target.depthHistoryDescriptorSets[hitIndex % 2],
				0, nullptr);
		}

		vkCmdDrawIndexed(
			cmd,
			static_cast<uint32_t>(_cageMesh._faces.size()),
			1, 0, 0, 0);

		vkCmdEndRenderPass(cmd);
		VK_CHECK(vkEndCommandBuffer(cmd));
	}

	// =====================================================================
	// SUBMIT
	// =====================================================================
	std::array<VkCommandBufferSubmitInfo, 6> cmdInfos{};

	for (uint32_t i = 0; i < 6; ++i) {
		cmdInfos[i] = {
			VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
			nullptr,
			_cubemapRenderUnit.graphicsCmdPerTarget[targetIndex][i]
		};
	}

	VkSemaphoreSubmitInfo signalInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = timeline,
		.value = signalValue,
		.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT
	};

	VkSubmitInfo2 submit{
		.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2,
		.commandBufferInfoCount = uint32_t(cmdInfos.size()),
		.pCommandBufferInfos = cmdInfos.data(),
		.signalSemaphoreInfoCount = 1,
		.pSignalSemaphoreInfos = &signalInfo
	};

	VkQueue graphicsQueue;
	vkGetDeviceQueue(
		_device,
		_device->GetQueueFamilies()._graphics.value(),
		0,
		&graphicsQueue);

	VK_CHECK(vkQueueSubmit2(graphicsQueue, 1, &submit, VK_NULL_HANDLE));
}

void CubemapRenderInstance::CreateSyncObjects()
{
	const uint32_t slotCount =
		static_cast<uint32_t>(_cubemapRenderUnit.targets.size());

	_timelines.resize(slotCount);
	_slotDoneValue.assign(slotCount, 0);

	for (uint32_t i = 0; i < slotCount; ++i)
	{
		VkSemaphoreTypeCreateInfo typeInfo{
			.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO,
			.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE,
			.initialValue = 0
		};

		VkSemaphoreCreateInfo semInfo{
			.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO,
			.pNext = &typeInfo
		};

		VK_CHECK(vkCreateSemaphore(
			_device,
			&semInfo,
			nullptr,
			&_timelines[i]
		));
	}
}

std::vector<glm::vec3> CubemapRenderInstance::BuildDeformableVertexPositions() const
{
	std::vector<glm::vec3> vertices;
	vertices.reserve(_deformableMesh._vertices.rows());

	for (int i = 0; i < _deformableMesh._vertices.rows(); ++i)
	{
		const auto& v = _deformableMesh._vertices.row(i);

		vertices.emplace_back(
			static_cast<float>(v(0)),
			static_cast<float>(v(1)),
			static_cast<float>(v(2))
		);
	}

	return vertices;
}

glm::mat4 CubemapRenderInstance::ComputeCubemapViewMatrix(uint32_t faceIndex, const glm::vec3& pos)
{
	switch (faceIndex)
	{
	case 0: return glm::lookAt(pos, pos + glm::vec3(1, 0, 0), glm::vec3(0, -1, 0)); // +X
	case 1: return glm::lookAt(pos, pos + glm::vec3(-1, 0, 0), glm::vec3(0, -1, 0)); // -X
	case 2: return glm::lookAt(pos, pos + glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));  // +Y
	case 3: return glm::lookAt(pos, pos + glm::vec3(0, -1, 0), glm::vec3(0, 0, -1)); // -Y
	case 4: return glm::lookAt(pos, pos + glm::vec3(0, 0, 1), glm::vec3(0, -1, 0)); // +Z
	case 5: return glm::lookAt(pos, pos + glm::vec3(0, 0, -1), glm::vec3(0, -1, 0)); // -Z
	default: return glm::mat4(1.0f);
	}
}

uint32_t CubemapRenderInstance::FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const
{
	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(_device->GetPhysicalDeviceHandle(), &memProperties);

	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
		if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
			return i;
	}
	throw std::runtime_error("Failed to find suitable memory type!");
}

void CubemapRenderInstance::ComputeCoordinatesGPUMP(
	const CubemapWorkRange& range,
	Eigen::MatrixXd& weights)
{
	const auto totalStart = std::chrono::steady_clock::now();
	auto waitStart = std::chrono::high_resolution_clock::now();
	auto waitTemp = std::chrono::high_resolution_clock::now();
	LOG_DEBUG("ComputeCoordinatesGPUAtomic (batched, no slots)");
  
	auto* computeStage = static_cast<GpuMPComputeStrategy*>(_computeStage.get());
	const auto vertices = BuildDeformableVertexPositions();

	const uint32_t end = std::min<uint32_t>(
		range.first + range.count,
		static_cast<uint32_t>(vertices.size())
	);
	const uint32_t cubemapCount = end - range.first;
	const uint32_t slotCount = static_cast<uint32_t>(_cubemapRenderUnit.targets.size());
	double renderAccumulatedMs = 0.0;
	double computeAccumulatedMs = 0.0;

	if (cubemapCount == 0 || slotCount == 0)
	{
		weights = computeStage->Readback();
		return;
	}

	VkSemaphore timeline = _timelines[0];
	uint64_t& timelineValue = _slotDoneValue[0];

	for (uint32_t batchStart = 0; batchStart < cubemapCount; batchStart += slotCount)
	{
		const uint32_t batchCount = std::min(slotCount, cubemapCount - batchStart);

		if (timelineValue > 0)
		{
			VkSemaphoreWaitInfo waitInfo{};
			waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
			waitInfo.semaphoreCount = 1;
			waitInfo.pSemaphores = &timeline;
			waitInfo.pValues = &timelineValue;
			VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
		}

		for (uint32_t hit = 0; hit < _hitCount; ++hit)
		{
			const auto renderStart = std::chrono::steady_clock::now();
			uint64_t renderDone = timelineValue;
			for (uint32_t slot = 0; slot < batchCount; ++slot)
			{
				const uint32_t cubemapIdx = range.first + batchStart + slot;
				CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];

				renderDone = ++timelineValue;
				RecordAndSubmitCubemapRender(
					cubemapIdx,
					slot,
					vertices[cubemapIdx],
					target,
					timeline,
					renderDone,
					hit);

				target.depthView = target.depthViewsArray[hit % 2];
				computeStage->RecordCompute(slot, target, cubemapIdx);
			}
			const auto renderEnd = std::chrono::steady_clock::now();
			renderAccumulatedMs += std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();

			if (_omitNegative && ((hit + 1) % 2u == 0u))
			{
				continue;
			}

			const auto computeStart = std::chrono::steady_clock::now();
			const uint64_t computeDone = ++timelineValue;
			computeStage->SubmitAllComputes(timeline, renderDone, timeline, computeDone);

			const uint64_t copyDone = ++timelineValue;
			computeStage->SubmitAllReadbackCopies(timeline, computeDone, timeline, copyDone);

			VkSemaphoreWaitInfo waitInfo{};
			waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
			waitInfo.semaphoreCount = 1;
			waitInfo.pSemaphores = &timeline;
			waitInfo.pValues = &copyDone;
			VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));

			timelineValue = copyDone;

			computeStage->ConsumeAllSlots(hit);
			
			const auto computeEnd = std::chrono::steady_clock::now();
			computeAccumulatedMs += std::chrono::duration<double, std::milli>(computeEnd - computeStart).count();
			
		}
	}

	const auto readbackStart = std::chrono::steady_clock::now();
	weights = computeStage->Readback();
	const auto readbackEnd = std::chrono::steady_clock::now();
	computeAccumulatedMs += std::chrono::duration<double, std::milli>(readbackEnd - readbackStart).count();

	_renderMs = renderAccumulatedMs;
	_computeMs = computeAccumulatedMs;
	_computeTotalMs = std::chrono::duration<double, std::milli>(readbackEnd - totalStart).count();
}

void CubemapRenderInstance::ComputeCoordinatesGPUMPThreeHit(
	const CubemapWorkRange& range,
	Eigen::MatrixXd& weights)
{
	const auto totalStart = std::chrono::steady_clock::now();
	LOG_DEBUG("ComputeCoordinatesGPUMPThreeHit (alpha={}, beta={}, theta={})",
		_alpha, _beta, _theta);

	auto* computeStage = static_cast<GpuMPComputeStrategy*>(_computeStage.get());
	const auto vertices = BuildDeformableVertexPositions();

	const uint32_t end = std::min<uint32_t>(
		range.first + range.count,
		static_cast<uint32_t>(vertices.size())
	);
	const uint32_t cubemapCount = end > range.first ? end - range.first : 0;
	const uint32_t slotCount = static_cast<uint32_t>(_cubemapRenderUnit.targets.size());
	double renderAccumulatedMs = 0.0;
	double computeAccumulatedMs = 0.0;

	if (cubemapCount == 0 || slotCount == 0)
	{
		weights = computeStage->Readback();
		return;
	}

	VkSemaphore timeline = _timelines[0];
	uint64_t& timelineValue = _slotDoneValue[0];

	const bool hasSecondHit = _hitCount >= 2;

	const auto waitTimeline = [&](uint64_t value)
	{
		VkSemaphoreWaitInfo waitInfo{};
		waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		waitInfo.semaphoreCount = 1;
		waitInfo.pSemaphores = &timeline;
		waitInfo.pValues = &value;
		VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
	};

	for (uint32_t batchStart = 0; batchStart < cubemapCount; batchStart += slotCount)
	{
		const uint32_t batchCount = std::min(slotCount, cubemapCount - batchStart);

		if (timelineValue > 0)
		{
			waitTimeline(timelineValue);
		}

		// -------------------------------------------------------------
		// Render the first hit (and the second hit) into their dedicated
		// color/depth targets so both are available to the compute shader.
		// -------------------------------------------------------------
		const auto renderStart = std::chrono::steady_clock::now();
		uint64_t renderDone = timelineValue;
		for (uint32_t slot = 0; slot < batchCount; ++slot)
		{
			const uint32_t cubemapIdx = range.first + batchStart + slot;
			CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];
			renderDone = ++timelineValue;
			RecordAndSubmitCubemapRender(
				cubemapIdx, slot, vertices[cubemapIdx], target, timeline, renderDone, 0);
		}
		if (hasSecondHit)
		{
			for (uint32_t slot = 0; slot < batchCount; ++slot)
			{
				const uint32_t cubemapIdx = range.first + batchStart + slot;
				CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];
				renderDone = ++timelineValue;
				RecordAndSubmitCubemapRender(
					cubemapIdx, slot, vertices[cubemapIdx], target, timeline, renderDone, 1);
			}
		}
		const auto renderEnd = std::chrono::steady_clock::now();
		renderAccumulatedMs += std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();

		// -------------------------------------------------------------
		// Combined compute: alpha * firstHit + beta * secondHit, evaluated
		// per ray by sampling both rendered textures in a single dispatch.
		// -------------------------------------------------------------
		const auto computeStart = std::chrono::steady_clock::now();
		// SubmitAllComputes always submits every slot's command buffer, so record
		// all of them. Slots beyond the current batch are marked with the
		// UINT32_MAX sentinel so ConsumeAllSlots skips their (stale) results.
		for (uint32_t slot = 0; slot < slotCount; ++slot)
		{
			CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];
			const uint32_t cubemapIdx = (slot < batchCount)
				? range.first + batchStart + slot
				: UINT32_MAX;
			computeStage->RecordCombinedCompute(
				slot,
				cubemapIdx,
				target.cubemapView,
				target.depthViewsArray[0],
				hasSecondHit ? target.cubemapViewSecond : target.cubemapView,
				hasSecondHit ? target.depthViewsArray[1] : target.depthViewsArray[0],
				_alpha,
				hasSecondHit ? _beta : 0.0f);
		}

		uint64_t computeDone = ++timelineValue;
		computeStage->SubmitAllComputes(timeline, renderDone, timeline, computeDone);

		uint64_t copyDone = ++timelineValue;
		computeStage->SubmitAllReadbackCopies(timeline, computeDone, timeline, copyDone);

		waitTimeline(copyDone);
		timelineValue = copyDone;
		computeStage->ConsumeAllSlots(0);

		// -------------------------------------------------------------
		// Remaining hits (the third hit and beyond) are weighted by theta
		// and accumulated with their own single-texture compute pass.
		// -------------------------------------------------------------
		for (uint32_t hit = 2; hit < _hitCount; ++hit)
		{
			uint64_t hitRenderDone = timelineValue;
			for (uint32_t slot = 0; slot < batchCount; ++slot)
			{
				const uint32_t cubemapIdx = range.first + batchStart + slot;
				CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];
				hitRenderDone = ++timelineValue;
				RecordAndSubmitCubemapRender(
					cubemapIdx, slot, vertices[cubemapIdx], target, timeline, hitRenderDone, hit);
			}

			for (uint32_t slot = 0; slot < slotCount; ++slot)
			{
				CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];
				const uint32_t cubemapIdx = (slot < batchCount)
					? range.first + batchStart + slot
					: UINT32_MAX;
				const VkImageView colorView = (hit % 2u == 0u) ? target.cubemapView : target.cubemapViewSecond;
				const VkImageView depthView = target.depthViewsArray[hit % 2u];
				computeStage->RecordCombinedCompute(
					slot,
					cubemapIdx,
					colorView,
					depthView,
					colorView,
					depthView,
					_theta,
					0.0f);
			}

			uint64_t hitComputeDone = ++timelineValue;
			computeStage->SubmitAllComputes(timeline, hitRenderDone, timeline, hitComputeDone);

			uint64_t hitCopyDone = ++timelineValue;
			computeStage->SubmitAllReadbackCopies(timeline, hitComputeDone, timeline, hitCopyDone);

			waitTimeline(hitCopyDone);
			timelineValue = hitCopyDone;
			computeStage->ConsumeAllSlots(0);
		}
		const auto computeEnd = std::chrono::steady_clock::now();
		computeAccumulatedMs += std::chrono::duration<double, std::milli>(computeEnd - computeStart).count();
	}

	const auto readbackStart = std::chrono::steady_clock::now();
	weights = computeStage->Readback();
	const auto readbackEnd = std::chrono::steady_clock::now();
	computeAccumulatedMs += std::chrono::duration<double, std::milli>(readbackEnd - readbackStart).count();

	_renderMs = renderAccumulatedMs;
	_computeMs = computeAccumulatedMs;
	_computeTotalMs = std::chrono::duration<double, std::milli>(readbackEnd - totalStart).count();
}

void CubemapRenderInstance::ComputeCoordinatesGPUAtomic(
	const CubemapWorkRange& range, Eigen::MatrixXd& weights)
{
	const auto totalStart = std::chrono::steady_clock::now();
	LOG_DEBUG("ComputeCoordinatesGPUSerial (pipelined): range.first={}, range.count={}",
		range.first, range.count);

	auto* computeStage =
		static_cast<GpuAtomicComputeStrategy*>(_computeStage.get());

	const auto vertices = BuildDeformableVertexPositions();

	const uint32_t end = std::min<uint32_t>(
		range.first + range.count,
		static_cast<uint32_t>(vertices.size())
	);
	const uint32_t cubemapCount = end > range.first ? end - range.first : 0;
	const uint32_t slotCount = static_cast<uint32_t>(_cubemapRenderUnit.targets.size());

	if (cubemapCount == 0 || slotCount == 0)
	{
		weights = computeStage->Readback();
		return;
	}

	const uint32_t hardwareThreads = std::max(1u, std::thread::hardware_concurrency());
	const uint32_t workerCount = std::max(1u, std::min({ hardwareThreads, slotCount, cubemapCount }));

	std::vector<std::vector<uint32_t>> slotPools(workerCount);
	for (uint32_t slot = 0; slot < slotCount; ++slot)
	{
		slotPools[slot % workerCount].push_back(slot);
	}

	ThreadPool workerPool(workerCount);
	std::vector<std::promise<void>> completionPromises(workerCount);
	std::vector<std::future<void>> completionFutures;
	completionFutures.reserve(workerCount);
	for (auto& completionPromise : completionPromises)
	{
		completionFutures.emplace_back(completionPromise.get_future());
	}

	std::mutex submitMutex;

	for (uint32_t workerId = 0; workerId < workerCount; ++workerId)
	{
		workerPool.Submit([&, workerId]() mutable
		{
			try
			{
				VkSemaphore timeline = _timelines[workerId];
				uint64_t& timelineValue = _slotDoneValue[workerId];
				const auto& pool = slotPools[workerId];

				for (uint32_t batchStart = 0; batchStart < cubemapCount; batchStart += slotCount)
				{
					const uint32_t batchCount = std::min(slotCount, cubemapCount - batchStart);

					if (timelineValue > 0)
					{
						VkSemaphoreWaitInfo waitInfo{};
						waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
						waitInfo.semaphoreCount = 1;
						waitInfo.pSemaphores = &timeline;
						waitInfo.pValues = &timelineValue;
						VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
					}

					for (const uint32_t slot : pool)
					{
						if (slot >= batchCount)
						{
							continue;
						}

						const uint32_t cubemapIdx = range.first + batchStart + slot;
						CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];

						const uint64_t renderDone = ++timelineValue;
						{
							std::lock_guard lock(submitMutex);
							RecordAndSubmitCubemapRender(
								cubemapIdx,
								slot,
								vertices[cubemapIdx],
								target,
								timeline,
								renderDone);
						}

						const uint64_t computeDone = ++timelineValue;
						{
							std::lock_guard lock(submitMutex);
							computeStage->DispatchAfterRender(
								cubemapIdx,
								slot,
								timeline,
								renderDone,
								computeDone,
								target
							);
						}

						const uint64_t copyDone = ++timelineValue;
						{
							std::lock_guard lock(submitMutex);
							computeStage->SubmitReadbackCopy(
								slot,
								timeline,
								computeDone,
								copyDone
							);
						}

						computeStage->ConsumeSlot(
							cubemapIdx,
							slot,
							timeline,
							copyDone
						);
					}
				}

				completionPromises[workerId].set_value();
			}
			catch (...)
			{
				completionPromises[workerId].set_exception(std::current_exception());
			}
		});
	}

	for (auto& completion : completionFutures)
	{
		completion.get();
	}

	weights = computeStage->Readback();
	const auto totalEnd = std::chrono::steady_clock::now();
	_computeTotalMs = std::chrono::duration<double, std::milli>(totalEnd - totalStart).count();

	LOG_DEBUG("ComputeCoordinatesGPUSerial (pipelined): done");
}

void CubemapRenderInstance::ComputeCoordinatesGPUSerial(
	const CubemapWorkRange& range, Eigen::MatrixXd& weights)
{
	const auto totalStart = std::chrono::steady_clock::now();
	LOG_DEBUG("ComputeCoordinatesGPUSerial: range.first={}, range.count={}",
		range.first, range.count);

	auto* computeStage =
		static_cast<GpuSerialComputeStrategy*>(_computeStage.get());

	const auto vertices = BuildDeformableVertexPositions();

	const uint32_t end = std::min<uint32_t>(
		range.first + range.count,
		static_cast<uint32_t>(vertices.size())
	);
	const uint32_t cubemapCount = end > range.first ? end - range.first : 0;
	const uint32_t slotCount = static_cast<uint32_t>(_cubemapRenderUnit.targets.size());

	if (cubemapCount == 0 || slotCount == 0)
	{
		weights = computeStage->Readback();
		return;
	}

	VkSemaphore timeline = _timelines[0];
	uint64_t& timelineValue = _slotDoneValue[0];

	for (uint32_t batchStart = 0; batchStart < cubemapCount; batchStart += slotCount)
	{
		const uint32_t batchCount = std::min(slotCount, cubemapCount - batchStart);

		if (timelineValue > 0)
		{
			VkSemaphoreWaitInfo waitInfo{};
			waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
			waitInfo.semaphoreCount = 1;
			waitInfo.pSemaphores = &timeline;
			waitInfo.pValues = &timelineValue;
			VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
		}

		std::vector<uint64_t> renderDoneValues(batchCount);
		std::vector<uint64_t> computeDoneValues(batchCount);
		std::vector<uint64_t> copyDoneValues(batchCount);

		for (uint32_t slot = 0; slot < batchCount; ++slot)
		{
			const uint32_t cubemapIdx = range.first + batchStart + slot;
			CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];

			renderDoneValues[slot] = ++timelineValue;
			RecordAndSubmitCubemapRender(
				cubemapIdx,
				slot,
				vertices[cubemapIdx],
				target,
				timeline,
				renderDoneValues[slot],
				0
			);
		}

		for (uint32_t slot = 0; slot < batchCount; ++slot)
		{
			const uint32_t cubemapIdx = range.first + batchStart + slot;
			CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];

			computeDoneValues[slot] = ++timelineValue;
			computeStage->DispatchAfterRender(
				cubemapIdx,
				slot,
				timeline,
				renderDoneValues[slot],
				computeDoneValues[slot],
				target
			);
		}

		for (uint32_t slot = 0; slot < batchCount; ++slot)
		{
			copyDoneValues[slot] = ++timelineValue;
			computeStage->SubmitReadbackCopy(
				slot,
				timeline,
				computeDoneValues[slot],
				copyDoneValues[slot]
			);
		}

		for (uint32_t slot = 0; slot < batchCount; ++slot)
		{
			const uint32_t cubemapIdx = range.first + batchStart + slot;
			computeStage->ConsumeSlot(
				cubemapIdx,
				slot,
				timeline,
				copyDoneValues[slot]
			);
		}

		timelineValue = copyDoneValues.back();
	}

	weights = computeStage->Readback();
	const auto totalEnd = std::chrono::steady_clock::now();
	_computeTotalMs = std::chrono::duration<double, std::milli>(totalEnd - totalStart).count();

	LOG_DEBUG("ComputeCoordinatesGPUSerial: done");
}

void CubemapRenderInstance::ComputeCoordinatesCpu(
	const CubemapWorkRange& range, Eigen::MatrixXd& weights)
{
	LOG_DEBUG("Start cpu");
	auto* computeStage = static_cast<CpuComputeStrategy*>(_computeStage.get());
	const auto vertices = BuildDeformableVertexPositions();

	const uint32_t end = std::min<uint32_t>(
		range.first + range.count,
		static_cast<uint32_t>(vertices.size())
	);
	const uint32_t cubemapCount = end - range.first;
	const uint32_t slotCount = static_cast<uint32_t>(_cubemapRenderUnit.targets.size());
	double renderAccumulatedMs = 0.0;
	double computeAccumulatedMs = 0.0;
	double transferAccumulatedMs = 0.0;

	if (cubemapCount == 0 || slotCount == 0)
	{
		weights = computeStage->Readback();
		return;
	}

	VkSemaphore timeline = _timelines[0];
	uint64_t& timelineValue = _slotDoneValue[0];

	for (uint32_t batchStart = 0; batchStart < cubemapCount; batchStart += slotCount)
	{
		const uint32_t batchCount = std::min(slotCount, cubemapCount - batchStart);

		if (timelineValue > 0)
		{
			VkSemaphoreWaitInfo waitInfo{};
			waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
			waitInfo.semaphoreCount = 1;
			waitInfo.pSemaphores = &timeline;
			waitInfo.pValues = &timelineValue;
			VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
		}

		const auto renderStart = std::chrono::steady_clock::now();
		uint64_t renderDone = timelineValue;
		for (uint32_t slot = 0; slot < batchCount; ++slot)
		{
			const uint32_t cubemapIdx = range.first + batchStart + slot;
			CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];

			renderDone = ++timelineValue;
			RecordAndSubmitCubemapRender(
				cubemapIdx,
				slot,
				vertices[cubemapIdx],
				target,
				timeline,
				renderDone,
				0);

			computeStage->RecordReadback(slot, target, cubemapIdx);
		}
		const auto renderEnd = std::chrono::steady_clock::now();
		renderAccumulatedMs += std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();

		const auto transferStart = std::chrono::steady_clock::now();
		const uint64_t readbackDone = ++timelineValue;
		computeStage->SubmitAllReadbacks(timeline, renderDone, timeline, readbackDone);

		VkSemaphoreWaitInfo waitInfo{};
		waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		waitInfo.semaphoreCount = 1;
		waitInfo.pSemaphores = &timeline;
		waitInfo.pValues = &readbackDone;
		VK_CHECK(vkWaitSemaphores(_device, &waitInfo, UINT64_MAX));
		const auto transferEnd = std::chrono::steady_clock::now();
		transferAccumulatedMs += std::chrono::duration<double, std::milli>(transferEnd - transferStart).count();

		timelineValue = readbackDone;
		const auto computeStart = std::chrono::steady_clock::now();
		computeStage->ConsumeAllSlots();
		const auto computeEnd = std::chrono::steady_clock::now();
		computeAccumulatedMs += std::chrono::duration<double, std::milli>(computeEnd - computeStart).count();
	}

	const auto readbackStart = std::chrono::steady_clock::now();
	weights = computeStage->Readback();
	const auto readbackEnd = std::chrono::steady_clock::now();
	transferAccumulatedMs += std::chrono::duration<double, std::milli>(readbackEnd - readbackStart).count();

	_renderMs = renderAccumulatedMs;
	_computeMs = computeAccumulatedMs;
	_transferMs = transferAccumulatedMs;
	_computeTotalMs = renderAccumulatedMs + computeAccumulatedMs + transferAccumulatedMs;
}

void CubemapRenderInstance::ComputeCoordinates(
	const CubemapWorkRange& range, Eigen::MatrixXd& weights) {
	_renderMs.reset();
	_computeMs.reset();
	_computeTotalMs.reset();
	_transferMs.reset();
	if (_computeType == PMVCComputeType::Serial) {
		ComputeCoordinatesGPUSerial(range, weights);
	}
	else if (_computeType == PMVCComputeType::Ring) {
		ComputeCoordinatesGPUAtomic(range, weights);
	}
	else if (_computeType == PMVCComputeType::All) {
		ComputeCoordinatesGPUMP(range, weights);
	}
	else if (_computeType == PMVCComputeType::ThreeHit) {
		ComputeCoordinatesGPUMPThreeHit(range, weights);
	}
	else if (_computeType == PMVCComputeType::Cpu) {
		ComputeCoordinatesCpu(range, weights);
	}
}
