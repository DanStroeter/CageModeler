#include <Mesh/PolygonMeshProxy.h>
#include <Rendering/Commands/DrawCommand.h>
#include <Rendering/Commands/BufferCommand.h>

PolygonMeshRenderProxy::PolygonMeshRenderProxy(const RenderResourceRef<Device>& device,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
	MeshProxySolidPipeline solidPipeline,
	MeshProxyWireframePipeline wireframePipeline,
	const bool supportsWireframeRendering,
	const WireframeRenderMode wireframeRenderMode,
	const SolidMeshProxyBuffers& solidMeshBuffers,
	const PointsMeshProxyBuffers& pointsWireframeBuffers,
	const EdgesListProxyBuffers& edgesWireframeBuffers)
	: MeshRenderProxy(device,
		renderCommandScheduler,
		std::move(solidPipeline),
		std::move(wireframePipeline),
		supportsWireframeRendering,
		wireframeRenderMode)
	, _drawInfluenceMap(false)
{
	Check(!solidMeshBuffers._positions.empty());
	Check(!solidMeshBuffers._vertices.empty());
	Check(!solidMeshBuffers._indices.empty());

	_solidMeshBuffers._positionBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(solidMeshBuffers._positions),
		VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
	_solidMeshBuffers._vertexBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(solidMeshBuffers._vertices),
		VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
	_solidMeshBuffers._indexBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(solidMeshBuffers._indices),
		VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT);

	_numVertices = solidMeshBuffers._vertices.size();
	_numIndices = solidMeshBuffers._indices.size();

	if (pointsWireframeBuffers.IsValid())
	{
		_pointsWireframeBuffers._positionBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(pointsWireframeBuffers._positions),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
		_pointsWireframeBuffers._vertexBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(pointsWireframeBuffers._vertices),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
		_pointsWireframeBuffers._indexBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(pointsWireframeBuffers._indices),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT);

		_numWireframePoints = pointsWireframeBuffers.GetNumElements();
	}

	if (edgesWireframeBuffers.IsValid())
	{
		_edgesWireframeBuffers._positionBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(edgesWireframeBuffers._positions),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
		_edgesWireframeBuffers._vertexBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(edgesWireframeBuffers._vertices),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
		_edgesWireframeBuffers._indexBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(edgesWireframeBuffers._indices),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT);

		_numWireframeEdges = edgesWireframeBuffers.GetNumElements();
	}
}

void PolygonMeshRenderProxy::Render(const VkCommandBuffer commandBuffer,
	const uint32_t currentFrameIndex,
	const uint32_t dynamicBufferOffset)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	std::array dynamicBufferOffsets { dynamicBufferOffset };

	const auto solidPipelineHandle = (_drawInfluenceMap ? _solidPipeline._vertexColorHandle : _solidPipeline._handle);

	// TODO: Group objects that require the same pipeline.
	renderCommandScheduler->ExecuteCommand<BindGraphicsPipelineCommand>(commandBuffer, solidPipelineHandle);

	// Bind the vertex and index buffers and also the descriptor sets.
	std::array vertexBuffers { _solidMeshBuffers._positionBuffer._deviceBuffer, _solidMeshBuffers._vertexBuffer._deviceBuffer };
	renderCommandScheduler->ExecuteCommand<BindVertexBuffersCommand>(commandBuffer, vertexBuffers, _solidMeshBuffers._indexBuffer._deviceBuffer);

	// Bind the matrices descriptor sets.
	if (!_solidPipeline._descriptorSets[currentFrameIndex].empty())
	{
		renderCommandScheduler->ExecuteCommand<BindDescriptorSetCommand>(commandBuffer,
			solidPipelineHandle,
			_solidPipeline._descriptorSets[currentFrameIndex],
			dynamicBufferOffsets);
	}

	renderCommandScheduler->ExecuteCommand<DrawIndexedCommand>(commandBuffer, _numIndices, 0);

	if (_renderFlags._renderPoints)
	{
		// TODO: Group objects that require the same pipeline.
		renderCommandScheduler->ExecuteCommand<BindGraphicsPipelineCommand>(commandBuffer, _wireframePipeline._pointsHandle);

		// Bind the vertex and index buffers and also the descriptor sets.
		std::array wireframeVertexBuffers { _pointsWireframeBuffers._positionBuffer._deviceBuffer, _pointsWireframeBuffers._vertexBuffer._deviceBuffer };
		renderCommandScheduler->ExecuteCommand<BindVertexBuffersCommand>(commandBuffer, wireframeVertexBuffers, _pointsWireframeBuffers._indexBuffer._deviceBuffer);

		// Bind the matrices descriptor sets.
		if (!_wireframePipeline._descriptorSets[currentFrameIndex].empty())
		{
			renderCommandScheduler->ExecuteCommand<BindDescriptorSetCommand>(commandBuffer,
				_wireframePipeline._pointsHandle,
				_wireframePipeline._descriptorSets[currentFrameIndex],
				dynamicBufferOffsets);
		}

		renderCommandScheduler->ExecuteCommand<DrawCommand>(commandBuffer, _numWireframePoints, 0);
	}

	if (_renderFlags._renderEdges)
	{
		// TODO: Group objects that require the same pipeline.
		renderCommandScheduler->ExecuteCommand<BindGraphicsPipelineCommand>(commandBuffer, _wireframePipeline._edgesHandle);

		// Bind the vertex and index buffers and also the descriptor sets.
		std::array wireframeVertexBuffers { _edgesWireframeBuffers._positionBuffer._deviceBuffer, _edgesWireframeBuffers._vertexBuffer._deviceBuffer };
		renderCommandScheduler->ExecuteCommand<BindVertexBuffersCommand>(commandBuffer, wireframeVertexBuffers, _edgesWireframeBuffers._indexBuffer._deviceBuffer);

		// Bind the matrices descriptor sets.
		if (!_wireframePipeline._descriptorSets[currentFrameIndex].empty())
		{
			renderCommandScheduler->ExecuteCommand<BindDescriptorSetCommand>(commandBuffer,
				_wireframePipeline._edgesHandle,
				_wireframePipeline._descriptorSets[currentFrameIndex],
				dynamicBufferOffsets);
		}

		renderCommandScheduler->ExecuteCommand<DrawIndexedCommand>(commandBuffer, _numWireframeEdges, 0);
	}
}

void PolygonMeshRenderProxy::DestroyRenderProxy(const RenderResourceRef<Device>& device)
{
	device->WaitIdle();

	_solidMeshBuffers.ReleaseResource(device);

	if (_numWireframePoints)
	{
		_pointsWireframeBuffers.ReleaseResource(device);
	}

	if (_numWireframeEdges)
	{
		_edgesWireframeBuffers.ReleaseResource(device);
	}
}

std::vector<VkVertexInputBindingDescription> PolygonMeshRenderProxy::GetBindingDescription()
{
	std::vector<VkVertexInputBindingDescription> bindingDescriptions;
	bindingDescriptions.reserve(2);

	VkVertexInputBindingDescription positionsBindingDescription { };
	positionsBindingDescription.binding = 0;
	positionsBindingDescription.stride = sizeof(glm::vec3);
	positionsBindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
	bindingDescriptions.push_back(positionsBindingDescription);

	VkVertexInputBindingDescription vertexBindingDescription { };
	vertexBindingDescription.binding = 1;
	vertexBindingDescription.stride = sizeof(Vertex);
	vertexBindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
	bindingDescriptions.push_back(vertexBindingDescription);

	return bindingDescriptions;
}

std::vector<VkVertexInputAttributeDescription> PolygonMeshRenderProxy::GetAttributeDescriptions()
{
	std::vector<VkVertexInputAttributeDescription> attributeDescriptions;
	attributeDescriptions.reserve(3);

	VkVertexInputAttributeDescription positionsAttributeDescription { };
	positionsAttributeDescription.binding = 0;
	positionsAttributeDescription.location = 0;
	positionsAttributeDescription.format = VK_FORMAT_R32G32B32_SFLOAT;
	positionsAttributeDescription.offset = 0;
	attributeDescriptions.push_back(positionsAttributeDescription);

	VkVertexInputAttributeDescription normalAttributeDescription { };
	normalAttributeDescription.binding = 1;
	normalAttributeDescription.location = 1;
	normalAttributeDescription.format = VK_FORMAT_R32G32B32_SFLOAT;
	normalAttributeDescription.offset = offsetof(Vertex, _normal);
	attributeDescriptions.push_back(normalAttributeDescription);

	VkVertexInputAttributeDescription vertexColorAttributeDescription { };
	vertexColorAttributeDescription.binding = 1;
	vertexColorAttributeDescription.location = 2;
	vertexColorAttributeDescription.format = VK_FORMAT_R32G32B32_SFLOAT;
	vertexColorAttributeDescription.offset = offsetof(Vertex, _vertexColor);
	attributeDescriptions.push_back(vertexColorAttributeDescription);

	return attributeDescriptions;
}

void PolygonMeshRenderProxy::SetDrawInfluenceMap(const bool drawInfluenceMap)
{
	_drawInfluenceMap = drawInfluenceMap;
}

void PolygonMeshRenderProxy::SetVertices(const std::span<Vertex> newVertices)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newVertices, _solidMeshBuffers._vertexBuffer);
}

void PolygonMeshRenderProxy::SetPositions(const std::span<glm::vec3> newPositions)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newPositions, _solidMeshBuffers._positionBuffer);
}

void PolygonMeshRenderProxy::SetPointsWireframeVertices(const std::span<Vertex> newWireframeVertices)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframeVertices, _pointsWireframeBuffers._vertexBuffer);
}

void PolygonMeshRenderProxy::SetPointsWireframePositions(const std::span<glm::vec3> newWireframePositions)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframePositions, _pointsWireframeBuffers._positionBuffer);
}

void PolygonMeshRenderProxy::SetEdgesWireframeVertices(const std::span<Vertex> newWireframeVertices)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframeVertices, _edgesWireframeBuffers._vertexBuffer);
}

void PolygonMeshRenderProxy::SetEdgesWireframePositions(const std::span<glm::vec3> newWireframePositions)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframePositions, _edgesWireframeBuffers._positionBuffer);
}
