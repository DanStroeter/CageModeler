#pragma once

#include <Rendering/Core/RenderProxy.h>
#include <Rendering/Core/Device.h>
#include <Rendering/Core/Buffer.h>
#include <Rendering/Core/AlignedVector.h>
#include <Rendering/Scene/SceneData.h>

class ScreenPass;
class PolygonMesh;

class RenderProxyCollector
{
public:
	RenderProxyCollector(const RenderResourceRef<Device>& device,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
		const std::shared_ptr<RenderResourceManager>& resourceManager);
	~RenderProxyCollector();

	void UpdateObjectsData(AlignedDeviceVector<ModelInfo>& objectsData,
		const ViewInfo& viewInfo,
		const MemoryMappedBuffer& buffer);

	void Render(const std::size_t currentFrameIndex,
		const double deltaTime,
		const ViewInfo& viewInfo) const;

	void DestroyPendingRenderProxies();
	void RecreateDirtyRenderProxies();

	template <typename T>
	void MarkRenderProxyAsDirty(const RenderProxyHandle renderProxyHandle, const T newRenderProxy)
	{
		_newRenderProxies[renderProxyHandle] = newRenderProxy;
		_dirtyRenderProxies[renderProxyHandle] = true;
	}

	template <typename T>
	RenderProxyHandle RegisterRenderProxy(std::unique_ptr<T> renderProxy)
	{
		// Get the next free index and mark it as set.
		const auto nextFreeIndex = std::ranges::find(_allocatedRenderProxies, false) - _allocatedRenderProxies.begin();
		_allocatedRenderProxies[nextFreeIndex] = true;

		// Set the value of the pipeline object.
		_renderProxies[nextFreeIndex] = std::move(renderProxy);

		return nextFreeIndex;
	}

	void MarkRenderProxyAsDestroyed(const RenderProxyHandle renderProxyHandle)
	{
		CheckFormat(_allocatedRenderProxies[renderProxyHandle], "No render proxy has been allocated with handle {}.", renderProxyHandle);

		_destroyedRenderProxies[renderProxyHandle] = true;
	}

	[[nodiscard]] RenderProxy* GetRenderProxy(const RenderProxyHandle renderProxyHandle) const
	{
		Check(renderProxyHandle < _renderProxies.size() - 1);

		return _renderProxies[renderProxyHandle].get();
	}

private:
	void FreeRenderProxy(const RenderProxyHandle proxyHandle);

private:
	RenderResourceRef<Device> _device;

	std::shared_ptr<RenderCommandScheduler> _renderCommandScheduler = nullptr;

	/// Pointer to the resource manager.
	std::shared_ptr<RenderResourceManager> _resourceManager = nullptr;

	/// A list of all registered render proxies.
	std::vector<std::unique_ptr<RenderProxy>> _renderProxies;

	/// All allocated render proxies to mimic a sparse array.
	std::vector<bool> _allocatedRenderProxies;

	/// Render proxies that should be destroyed instead of being recorded into next frame.
	std::vector<bool> _destroyedRenderProxies;

	/// All render proxies that were marked as dirty.
	std::vector<bool> _dirtyRenderProxies;

	/// All render proxies that were marked as dirty and have to be replaced by during the current frame.
	std::vector<std::unique_ptr<RenderProxy>> _newRenderProxies;

	/// The alignment of the objects buffer.
	std::size_t _objectsBufferDynamicAlignment = 0;
};
