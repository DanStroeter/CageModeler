#include <Rendering/Core/RenderProxyCollector.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Mesh/PolygonMesh.h>

#include <glm/gtc/matrix_inverse.hpp>

namespace
{
	constexpr std::size_t MaxNumRenderProxies = 16;
}

RenderProxyCollector::RenderProxyCollector(const RenderResourceRef<Device>& device,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
	const std::shared_ptr<RenderResourceManager>& resourceManager)
	: _device(device)
	, _renderCommandScheduler(renderCommandScheduler)
	, _resourceManager(resourceManager)
	, _renderProxies(MaxNumRenderProxies)
	, _allocatedRenderProxies(MaxNumRenderProxies)
	, _destroyedRenderProxies(MaxNumRenderProxies)
	, _dirtyRenderProxies(MaxNumRenderProxies)
{
	std::fill(_allocatedRenderProxies.begin(), _allocatedRenderProxies.end(), false);
	std::fill(_destroyedRenderProxies.begin(), _destroyedRenderProxies.end(), false);
	std::fill(_dirtyRenderProxies.begin(), _dirtyRenderProxies.end(), false);

	// Calculate required alignment based on minimum device offset alignment
	_objectsBufferDynamicAlignment = _device->GetMinimumMemoryAlignment<ModelInfo>();
}

RenderProxyCollector::~RenderProxyCollector()
{
	for (std::size_t i = 0; i < _allocatedRenderProxies.size(); ++i)
	{
		if (_allocatedRenderProxies[i])
		{
			_renderProxies[i]->DestroyRenderProxy(_device);
		}
	}
}

void RenderProxyCollector::UpdateObjectsData(AlignedDeviceVector<ModelInfo>& objectsData,
	const ViewInfo& viewInfo,
	const MemoryMappedBuffer& buffer)
{
	for (std::size_t i = 0; i < _allocatedRenderProxies.size(); ++i)
	{
		if (_allocatedRenderProxies[i])
		{
			auto& objectData = objectsData[i];
			const auto modelMatrix = _renderProxies[i]->GetModelMatrix();
			objectData._model = modelMatrix;
			objectData._normalMatrix = glm::inverseTranspose(glm::mat3(modelMatrix));
		}
	}

	// Update the objects data.
	memcpy(buffer._mappedData,
		objectsData.Data(),
		buffer._allocatedSize);

	// Flush to make changes visible to the host
	VkMappedMemoryRange mappedMemoryRange { };
	mappedMemoryRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
	mappedMemoryRange.memory = buffer._deviceMemory;
	mappedMemoryRange.size = buffer._allocatedSize;
	vkFlushMappedMemoryRanges(_device, 1, &mappedMemoryRange);
}

void RenderProxyCollector::Render(const std::size_t currentFrameIndex,
	const double deltaTime,
	const ViewInfo& viewInfo) const
{
	const auto currentCommandBuffer = _renderCommandScheduler->GetCommandBuffer(currentFrameIndex);

	for (std::size_t i = 0; i < _allocatedRenderProxies.size(); ++i)
	{
		if (_allocatedRenderProxies[i])
		{
			const auto& renderProxy = _renderProxies[i];

			if (renderProxy->IsVisible())
			{
				const auto dynamicOffset = i * static_cast<uint32_t>(_objectsBufferDynamicAlignment);
				renderProxy->Render(currentCommandBuffer, currentFrameIndex, dynamicOffset);
			}
		}
	}
}

void RenderProxyCollector::DestroyPendingRenderProxies()
{
	// Clean up all destroyed render proxies.
	for (std::size_t i = 0; i < _destroyedRenderProxies.size(); ++i)
	{
		if (_destroyedRenderProxies[i])
		{
			_renderProxies[i]->DestroyRenderProxy(_device);
			_allocatedRenderProxies[i] = false;
			_destroyedRenderProxies[i] = false;
		}
	}
}

void RenderProxyCollector::RecreateDirtyRenderProxies()
{
	for (std::size_t i = 0; i < _dirtyRenderProxies.size(); ++i)
	{
		if (_dirtyRenderProxies[i])
		{
			_renderProxies[i] = std::move(_newRenderProxies[i]);
			_dirtyRenderProxies[i] = false;
		}
	}
}

void RenderProxyCollector::FreeRenderProxy(const RenderProxyHandle proxyHandle)
{
	// Mark the index as not allocated.
	_allocatedRenderProxies[proxyHandle] = false;

	// Set the value of the pipeline object.
	_renderProxies[proxyHandle] = nullptr;
}
