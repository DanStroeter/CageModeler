#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxy.h>

RenderProxy::RenderProxy(const RenderResourceRef<Device>& device,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
	MeshProxySolidPipeline solidPipeline,
	const bool supportsWireframeRendering)
	: _device(device)
	, _renderCommandScheduler(renderCommandScheduler)
	, _solidPipeline(std::move(solidPipeline))
{
	_renderFlags._supportsWireframeRendering = supportsWireframeRendering;
}

MeshRenderProxy::MeshRenderProxy(const RenderResourceRef<Device>& device,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
	MeshProxySolidPipeline solidPipeline,
	MeshProxyWireframePipeline wireframePipeline,
	const bool supportsWireframeRendering,
	const WireframeRenderMode wireframeRenderMode)
	: RenderProxy(device, renderCommandScheduler, std::move(solidPipeline), supportsWireframeRendering)
	, _wireframePipeline(std::move(wireframePipeline))
{
	SetWireframeRenderMode(wireframeRenderMode);
}
