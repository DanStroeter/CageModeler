#include <Mesh/ScreenPass.h>
#include <Rendering/Commands/DrawCommand.h>
#include <Rendering/Commands/RenderCommandScheduler.h>
#include <Rendering/Core/RenderProxyCollector.h>

ScreenPassRenderProxy::ScreenPassRenderProxy(const RenderResourceRef<Device>& device,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
	MeshProxySolidPipeline pipeline)
	: RenderProxy(device, renderCommandScheduler, std::move(pipeline), false)
{

}

void ScreenPassRenderProxy::Render(const VkCommandBuffer commandBuffer,
	const uint32_t currentFrameIndex,
	const uint32_t dynamicBufferOffset)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	// TODO: Group objects that require the same pipeline.
	renderCommandScheduler->ExecuteCommand<BindGraphicsPipelineCommand>(commandBuffer, _solidPipeline._handle);

	// Bind the matrices descriptor sets.
	if (!_solidPipeline._descriptorSets[currentFrameIndex].empty())
	{
		renderCommandScheduler->ExecuteCommand<BindDescriptorSetCommand>(commandBuffer,
																		 _solidPipeline._handle,
																		 _solidPipeline._descriptorSets[currentFrameIndex],
																		 std::span<uint32_t, 0> { });
	}

	// Draw the 2 triangles full-screen.
	renderCommandScheduler->ExecuteCommand<DrawCommand>(commandBuffer, 6, 0);
}

void ScreenPassRenderProxy::DestroyRenderProxy(const RenderResourceRef<Device>& device)
{

}

ScreenPass::ScreenPass(MeshProxySolidPipeline pipeline)
	: _pipeline(std::move(pipeline))
{
}

void ScreenPass::CollectRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector,
	const RenderResourceRef<Device>& device,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler)
{
	auto renderProxy = std::make_unique<ScreenPassRenderProxy>(device,
		renderCommandScheduler,
		_pipeline);

	_renderProxyHandle = renderProxyCollector->RegisterRenderProxy(std::move(renderProxy));
	_renderProxy = renderProxyCollector->GetRenderProxy(_renderProxyHandle);
}

void ScreenPass::DestroyRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector)
{
	renderProxyCollector->MarkRenderProxyAsDestroyed(_renderProxyHandle);

	_renderProxy = nullptr;
	_renderProxyHandle = InvalidHandle;
}
