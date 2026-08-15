#include <Rendering/PMVC/CubemapRenderInstance.h>
#include <Rendering/PMVC/CubemapManager.h>
#include <Rendering/PMVC/RingComputeStrategy.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Mesh/Operations/MeshWeightsParams.h>
#include <Thread/ThreadPool.h>
#include <Logging/Logging.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <exception>
#include <future>
#include <limits>
#include <mutex>
#include <numeric>
#include <thread>

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

CubemapRenderInstance::CubemapRenderInstance(
	const uint32_t cubemapSize,
	const VkFormat format,
	const bool useOffset,
	const uint32_t targetCount,
	const uint32_t hitCount,
	const float alpha,
	const float beta,
	const float theta,
	const Eigen::MatrixXf* interiorDetours,

	RenderResourceRef<Device> device,
	RenderResourceRef<DescriptorPool> descriptorPool,
	std::shared_ptr<RenderResourceManager> resourceManager,
	std::shared_ptr<RenderPipelineManager> renderPipelineManager,

	EigenMesh cageMesh,
	EigenMesh deformableMesh,

	VkRenderPass renderPass,
	PipelineHandle cubemapPipelineHandle,
	PipelineHandle cubemapPipelineHitHandle,

	RenderResourceRef<DescriptorSetLayout> matricesLayout,
	RenderResourceRef<DescriptorSetLayout> depthHistoryLayout,

	MemoryMappedBuffer indexBuffer,
	MemoryMappedBuffer vertexBuffer
): _pmvcUseOffset(useOffset)
, _interiorDetours(interiorDetours)
, _cubemapSize(cubemapSize)
, _targetCount(targetCount == 0 ? 1u : targetCount)
, _hitCount(hitCount == 0 ? 1u : hitCount)
, _alpha(alpha)
, _beta(beta)
, _theta(theta)
, _format(format)
, _device(std::move(device))
, _descriptorPool(std::move(descriptorPool))
, _resourceManager(std::move(resourceManager))
, _renderPipelineManager(std::move(renderPipelineManager))
, _cageMesh(std::move(cageMesh))
, _deformableMesh(std::move(deformableMesh))
, _renderPass(renderPass)
, _cubemapPipelineHandle(cubemapPipelineHandle)
, _cubemapPipelineHitHandle(cubemapPipelineHitHandle)
, _matricesLayout(std::move(matricesLayout))
, _depthHistoryLayout(std::move(depthHistoryLayout))
, _indexBuffer(std::move(indexBuffer))
, _vertexBuffer(std::move(vertexBuffer))
{
	UpdateProjectionPlanes();
	Initialize();
}

CubemapRenderInstance::~CubemapRenderInstance()
{
	Cleanup();
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

		for (auto depthViewArray : target.depthViewsArray)
		{
			if (depthViewArray != VK_NULL_HANDLE)
				vkDestroyImageView(_device, depthViewArray, nullptr);
		}

		if (target.cubemapImage != VK_NULL_HANDLE)
			vkDestroyImage(_device, target.cubemapImage, nullptr);
		if (target.cubemapMemory != VK_NULL_HANDLE)
			vkFreeMemory(_device, target.cubemapMemory, nullptr);

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

void CubemapRenderInstance::Initialize()
{
	InteriorDistanceSettings interiorSettings{};
	interiorSettings.detours = _interiorDetours;
	interiorSettings.nearPlane = _projectionNearPlane;
	interiorSettings.farPlane = _projectionFarPlane;

	_computeStage = std::make_unique<RingComputeStrategy>(
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
		interiorSettings
	);

	CreateCommandPool(_device->GetQueueFamilies()._graphics.value());

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

float CubemapRenderInstance::HitWeight(const uint32_t hitIndex) const
{
	// The three-hit variant is the only configuration that keeps the negative (second)
	// hit, weighted by beta, and adds a third hit weighted by theta.
	if (_hitCount == PMVCSettings::kThreeHitCount)
	{
		switch (hitIndex)
		{
		case 0: return _alpha;
		case 1: return _beta;
		default: return _theta;
		}
	}

	// Every second hit carries the negative contributions, which are always omitted
	// outside the three-hit variant. Those layers are still rendered because the next
	// layer is peeled against their depth, they simply do not contribute any weight.
	return (hitIndex % 2u == 0u) ? 1.0f : 0.0f;
}

CubemapRenderTarget CubemapRenderInstance::CreateCubemapRenderTarget() const
{
	CubemapRenderTarget target{};

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
		VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
	imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	VK_CHECK(vkCreateImage(_device, &imageInfo, nullptr, &target.cubemapImage));

	VkMemoryRequirements memReq{};
	vkGetImageMemoryRequirements(_device, target.cubemapImage, &memReq);

	VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
	allocInfo.allocationSize = memReq.size;
	allocInfo.memoryTypeIndex = FindMemoryType(memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	VK_CHECK(vkAllocateMemory(_device, &allocInfo, nullptr, &target.cubemapMemory));
	VK_CHECK(vkBindImageMemory(_device, target.cubemapImage, target.cubemapMemory, 0));

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
	// Create the array view the compute shader samples
	// ---------------------------------------------------------------------
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
	// Create depth images (ping-pong)
	// ---------------------------------------------------------------------
	const VkFormat depthFormat = _device->FindDepthFormat();

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
		// The hit rendering into slot N samples the depth written by the previous hit,
		// which lives in the other ping-pong slot.
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

	for (uint32_t pingPong = 0; pingPong < 2; ++pingPong)
	{
		for (uint32_t face = 0; face < 6; ++face)
		{
			VkImageView attachments[2] = {
				target.faceViews[face],
				target.depthViews[pingPong][face]
			};

			VkFramebufferCreateInfo fbInfo{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
			fbInfo.renderPass = _renderPass;
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
	const VkDeviceSize matricesUBOSize = sizeof(CubemapMatricesUBO);
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
	// Allocate command buffers (one per face and target)
	// ------------------------------------------------------------
	const auto targetCount = static_cast<uint32_t>(unit.targets.size());
	unit.graphicsCmdPerTarget.resize(targetCount);

	if (targetCount > 0)
	{
		std::vector<VkCommandBuffer> flatBuffers(static_cast<size_t>(targetCount) * 6);

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

	return unit;
}

void CubemapRenderInstance::CreateCommandPool(const uint32_t queueFamilyIndex)
{
	VkCommandPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	poolInfo.queueFamilyIndex = queueFamilyIndex;
	poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

	if (vkCreateCommandPool(_device, &poolInfo, nullptr, &_graphicCommandPool) != VK_SUCCESS)
	{
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
	const uint32_t targetIndex,
	const glm::vec3& camPos,
	const CubemapRenderTarget& target,
	VkSemaphore timeline,
	const uint64_t waitValue,
	const uint64_t signalValue,
	const uint32_t hitIndex)
{
	assert(targetIndex < _cubemapRenderUnit.graphicsCmdPerTarget.size());

	VkClearValue clearValues[2]{};
	clearValues[0].color = { {0.f, 0.f, 0.f, 1.f} };
	clearValues[1].depthStencil = { 1.f, 0 };

	VkCommandBufferBeginInfo beginInfo{
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
		.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
	};

	// Everything but the first hit is peeled against the depth of the previous hit.
	const PipelineHandle pipelineHandle = (hitIndex > 0) ? _cubemapPipelineHitHandle : _cubemapPipelineHandle;

	const PipelineObject& pipelineObj =
		_renderPipelineManager->GetPipelineObject(pipelineHandle);

	// ---------------------------------------------------------------------
	// Update shared UBO (outside render loop)
	// ---------------------------------------------------------------------
	const auto numTriangles =
		static_cast<uint32_t>(_cageMesh._faces.size() / 3);

	CubemapMatricesUBO ubo{};
	ubo.invNumTriangles = 1.0f / static_cast<float>(numTriangles);
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

		VkRenderPassBeginInfo rpInfo{
			.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO,
			.renderPass = _renderPass,
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

	for (uint32_t i = 0; i < 6; ++i)
	{
		cmdInfos[i] = {
			VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
			nullptr,
			_cubemapRenderUnit.graphicsCmdPerTarget[targetIndex][i]
		};
	}

	// Waiting on the previous stage of the slot keeps the hits of a vertex ordered: the
	// color image is shared by all hits and the peeling pass samples the depth the
	// previous hit wrote.
	VkSemaphoreSubmitInfo waitInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = timeline,
		.value = waitValue,
		.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT
	};

	VkSemaphoreSubmitInfo signalInfo{
		.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
		.semaphore = timeline,
		.value = signalValue,
		.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT
	};

	VkSubmitInfo2 submit{
		.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2,
		.waitSemaphoreInfoCount = 1,
		.pWaitSemaphoreInfos = &waitInfo,
		.commandBufferInfoCount = static_cast<uint32_t>(cmdInfos.size()),
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
	const auto slotCount =
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

glm::mat4 CubemapRenderInstance::ComputeCubemapViewMatrix(const uint32_t faceIndex, const glm::vec3& pos) const
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

uint32_t CubemapRenderInstance::FindMemoryType(const uint32_t typeFilter, const VkMemoryPropertyFlags properties) const
{
	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(_device->GetPhysicalDeviceHandle(), &memProperties);

	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++)
	{
		if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
			return i;
	}

	throw std::runtime_error("Failed to find suitable memory type!");
}

void CubemapRenderInstance::ComputeCoordinates(
	const CubemapWorkRange& range, Eigen::MatrixXd& weights)
{
	_renderMs.reset();
	_computeMs.reset();
	_computeTotalMs.reset();
	_transferMs.reset();

	const auto totalStart = std::chrono::steady_clock::now();
	LOG_DEBUG("PMVC Ring compute: range.first={}, range.count={}, hitCount={}",
		range.first, range.count, _hitCount);

	const auto vertices = BuildDeformableVertexPositions();

	const uint32_t end = std::min<uint32_t>(
		range.first + range.count,
		static_cast<uint32_t>(vertices.size())
	);
	const uint32_t cubemapCount = end > range.first ? end - range.first : 0;
	const auto slotCount = static_cast<uint32_t>(_cubemapRenderUnit.targets.size());

	if (cubemapCount == 0 || slotCount == 0)
	{
		weights = _computeStage->Readback();
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

	std::vector<double> renderMsPerWorker(workerCount, 0.0);
	std::vector<double> computeMsPerWorker(workerCount, 0.0);

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

				const auto waitTimeline = [&](const uint64_t value)
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

					for (const uint32_t slot : pool)
					{
						if (slot >= batchCount)
						{
							continue;
						}

						const uint32_t cubemapIdx = range.first + batchStart + slot;
						CubemapRenderTarget& target = _cubemapRenderUnit.targets[slot];

						_computeStage->BeginVertex(cubemapIdx);

						// Every hit is a depth peeling layer of the same cubemap: it is
						// rendered on top of the depth of the previous hit and consumed
						// by its own dispatch, weighted by HitWeight().
						for (uint32_t hit = 0; hit < _hitCount; ++hit)
						{
							const auto renderStart = std::chrono::steady_clock::now();

							const uint64_t previousDone = timelineValue;
							const uint64_t renderDone = ++timelineValue;
							{
								std::lock_guard lock(submitMutex);
								RecordAndSubmitCubemapRender(
									slot,
									vertices[cubemapIdx],
									target,
									timeline,
									previousDone,
									renderDone,
									hit);
							}

							const auto renderEnd = std::chrono::steady_clock::now();
							renderMsPerWorker[workerId] +=
								std::chrono::duration<double, std::milli>(renderEnd - renderStart).count();

							const float hitWeight = HitWeight(hit);
							if (hitWeight == 0.0f)
							{
								// The layer is only rendered so the next hit can be
								// peeled against its depth. Nothing is dispatched for it,
								// so wait here instead: the next hit re-records the render
								// command buffers of this slot, which must not be reset
								// while they are still executing.
								waitTimeline(renderDone);
								continue;
							}

							const auto computeStart = std::chrono::steady_clock::now();

							const uint64_t computeDone = ++timelineValue;
							{
								std::lock_guard lock(submitMutex);
								_computeStage->DispatchAfterRender(
									cubemapIdx,
									slot,
									timeline,
									renderDone,
									computeDone,
									target.cubemapView,
									target.depthViewsArray[hit % 2],
									hitWeight
								);
							}

							const uint64_t copyDone = ++timelineValue;
							{
								std::lock_guard lock(submitMutex);
								_computeStage->SubmitReadbackCopy(
									slot,
									timeline,
									computeDone,
									copyDone
								);
							}

							_computeStage->AccumulateSlot(
								cubemapIdx,
								slot,
								timeline,
								copyDone
							);

							const auto computeEnd = std::chrono::steady_clock::now();
							computeMsPerWorker[workerId] +=
								std::chrono::duration<double, std::milli>(computeEnd - computeStart).count();
						}
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

	weights = _computeStage->Readback();

	const auto totalEnd = std::chrono::steady_clock::now();
	_renderMs = std::accumulate(renderMsPerWorker.begin(), renderMsPerWorker.end(), 0.0);
	_computeMs = std::accumulate(computeMsPerWorker.begin(), computeMsPerWorker.end(), 0.0);
	_computeTotalMs = std::chrono::duration<double, std::milli>(totalEnd - totalStart).count();

	LOG_DEBUG("PMVC Ring compute: done in {} ms.", _computeTotalMs.value());
}
