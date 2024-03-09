#pragma once

#include <Rendering/Core/RenderProxy.h>

class RenderProxyCollector;
class RenderResourceManager;
class RenderCommandScheduler;

class ScreenPassRenderProxy final : public RenderProxy
{
public:
	ScreenPassRenderProxy(const RenderResourceRef<Device>& device,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
		MeshProxySolidPipeline pipeline);

	//~BEGIN RenderProxy
	void Render(const VkCommandBuffer commandBuffer,
		const uint32_t currentFrameIndex,
		const uint32_t dynamicBufferOffset) override;
	void DestroyRenderProxy(const RenderResourceRef<Device>& device) override;
	//~END RenderProxy
};

class ScreenPass
{
public:
	explicit ScreenPass(MeshProxySolidPipeline pipeline);

	/**
	 * Creates a new render proxy for the screen pass.
	 * @param renderProxyCollector A pointer to the render proxy collector that will register the new proxy instance.
	 * @param device A reference to the Vulkan device.
	 * @param renderCommandScheduler A pointer to the render command scheduler to pass to the proxy.
	*/
	void CollectRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector,
		const RenderResourceRef<Device>& device,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler);

	/**
	 * Destroy the render proxy associated with the mesh.
	 * @param renderProxyCollector A pointer to the render proxy collector that will deregister the new proxy instance.
	 */
	void DestroyRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector);

private:
	template <typename T> friend struct ProxyCollectorHelper;

	/// The pipeline used to render the mesh.
	MeshProxySolidPipeline _pipeline;

	/// TODO: No time to think of something better, but since the pointer will only change under special conditions it's ok.
	RenderProxy* _renderProxy = nullptr;

	/// A handle to the render proxy if we want to update it.
	RenderProxyHandle _renderProxyHandle = InvalidHandle;
};
