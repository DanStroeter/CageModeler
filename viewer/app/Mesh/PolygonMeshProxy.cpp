#include <Mesh/PolygonMeshProxy.h>
#include <Rendering/Commands/DrawCommand.h>
#include <Rendering/Commands/BufferCommand.h>

PolygonMeshRenderProxy::PolygonMeshRenderProxy(const RenderResourceRef<Device>& device,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
	MeshProxySolidPipeline solidPipeline,
	MeshProxyWireframePipeline wireframePipeline,
	const bool supportsWireframeRendering,
	const WireframeRenderMode wireframeRenderMode,
	const TrianglesListProxyBuffers& solidMeshBuffers,
	const PointsListProxyBuffers& pointsWireframeBuffers,
	const EdgesListProxyBuffers& edgesWireframeBuffers,
	const TrianglesListProxyBuffers& polysListBuffers)
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

	if (polysListBuffers.IsValid())
	{
		_polygonsWireframeBuffers._positionBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(polysListBuffers._positions),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
		_polygonsWireframeBuffers._vertexBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(polysListBuffers._vertices),
			VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);

		if (!polysListBuffers._indices.empty())
		{
			_polygonsWireframeBuffers._indexBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(polysListBuffers._indices),
				VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
		}

		_numWireframeVertices = polysListBuffers._vertices.size();
		_numWireframeIndices = polysListBuffers._indices.size();
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

	{
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
	}

	renderCommandScheduler->ExecuteCommand<DrawIndexedCommand>(commandBuffer, _numIndices, 0);

	if (_renderFlags._renderPoints)
	{
		// TODO: Group objects that require the same pipeline.
		renderCommandScheduler->ExecuteCommand<BindGraphicsPipelineCommand>(commandBuffer, _wireframePipeline._pointsHandle);

		// Bind the vertex and index buffers and also the descriptor sets.
		std::array wireframeVertexBuffers { _solidMeshBuffers._positionBuffer._deviceBuffer, _pointsWireframeBuffers._vertexBuffer._deviceBuffer };
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

	if (_renderFlags._renderPolygons && _polygonsWireframeBuffers._indexBuffer._allocatedSize > 0)
	{
		// TODO: Group objects that require the same pipeline.
		renderCommandScheduler->ExecuteCommand<BindGraphicsPipelineCommand>(commandBuffer, _wireframePipeline._polyHandle);

		// Bind the vertex and index buffers and also the descriptor sets.
		std::array vertexBuffers { _polygonsWireframeBuffers._positionBuffer._deviceBuffer, _polygonsWireframeBuffers._vertexBuffer._deviceBuffer };
		renderCommandScheduler->ExecuteCommand<BindVertexBuffersCommand>(commandBuffer, vertexBuffers, _polygonsWireframeBuffers._indexBuffer._deviceBuffer);

		// Bind the matrices descriptor sets.
		if (!_solidPipeline._descriptorSets[currentFrameIndex].empty())
		{
			renderCommandScheduler->ExecuteCommand<BindDescriptorSetCommand>(commandBuffer,
				_wireframePipeline._polyHandle,
				_wireframePipeline._descriptorSets[currentFrameIndex],
				dynamicBufferOffsets);
		}

		renderCommandScheduler->ExecuteCommand<DrawIndexedCommand>(commandBuffer, _numWireframeIndices, 0);
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

void PolygonMeshRenderProxy::SetWireframePositions(const WireframeRenderMode renderMode, const std::span<glm::vec3> newWireframePositions)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	if (renderMode == WireframeRenderMode::Points)
	{
		CheckNoEntry("Wireframe positions are inherited from the mesh itself.");
	}
	else if (renderMode == WireframeRenderMode::Edges)
	{
		renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframePositions, _edgesWireframeBuffers._positionBuffer);
	}
	else if (renderMode == WireframeRenderMode::Polygons)
	{
		if (!newWireframePositions.empty())
		{
			if (_numWireframeVertices == newWireframePositions.size())
			{
				renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframePositions, _polygonsWireframeBuffers._positionBuffer);
			}
			else
			{
				// First release the old buffer.
				if (_polygonsWireframeBuffers._positionBuffer._allocatedSize > 0)
				{
					_polygonsWireframeBuffers._positionBuffer.ReleaseResource(_device);
				}

				// Create the buffer again.
				// TODO: That is the worst way of doing it, but the easiest to implement.
				_polygonsWireframeBuffers._positionBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(newWireframePositions,
					VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
			}
		}

		_numWireframeVertices = newWireframePositions.size();
	}
}

void PolygonMeshRenderProxy::SetWireframeVertices(const WireframeRenderMode renderMode, const std::span<Vertex> newWireframeVertices)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	if (renderMode == WireframeRenderMode::Points)
	{
		renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframeVertices, _pointsWireframeBuffers._vertexBuffer);
	}
	else if (renderMode == WireframeRenderMode::Edges)
	{
		renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframeVertices, _edgesWireframeBuffers._vertexBuffer);
	}
	else if (renderMode == WireframeRenderMode::Polygons)
	{
		if (!newWireframeVertices.empty())
		{
			if (_numWireframeIndices == newWireframeVertices.size())
			{
				renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframeVertices, _polygonsWireframeBuffers._vertexBuffer);
			}
			else
			{
				// First release the old buffer.
				if (_polygonsWireframeBuffers._vertexBuffer._allocatedSize > 0)
				{
					_polygonsWireframeBuffers._vertexBuffer.ReleaseResource(_device);
				}

				// Create the buffer again.
				// TODO: That is the worst way of doing it, but the easiest to implement.
				_polygonsWireframeBuffers._vertexBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(newWireframeVertices,
					VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
			}
		}

		_numWireframeVertices = newWireframeVertices.size();
	}
}

void PolygonMeshRenderProxy::SetWireframeIndices(const WireframeRenderMode renderMode, const std::span<uint32_t> newWireframeIndices)
{
	const auto renderCommandScheduler = _renderCommandScheduler.lock();
	if (renderCommandScheduler == nullptr)
	{
		return;
	}

	if (renderMode == WireframeRenderMode::Points)
	{
		renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframeIndices, _pointsWireframeBuffers._indexBuffer);
	}
	else if (renderMode == WireframeRenderMode::Edges)
	{
		renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframeIndices, _edgesWireframeBuffers._indexBuffer);
	}
	else if (renderMode == WireframeRenderMode::Polygons)
	{
		if (!newWireframeIndices.empty())
		{
			if (_numWireframeIndices == newWireframeIndices.size())
			{
				renderCommandScheduler->ExecuteCommand<UpdateBufferCommand>(newWireframeIndices, _polygonsWireframeBuffers._indexBuffer);
			}
			else
			{
				// First release the old buffer.
				if (_polygonsWireframeBuffers._indexBuffer._allocatedSize > 0)
				{
					_polygonsWireframeBuffers._indexBuffer.ReleaseResource(_device);
				}

				// Create the buffer again.
				// TODO: That is the worst way of doing it, but the easiest to implement.
				_polygonsWireframeBuffers._indexBuffer = renderCommandScheduler->ExecuteCommand<CreateBufferCommand>(std::span(newWireframeIndices),
					VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
			}
		}

		_numWireframeIndices = newWireframeIndices.size();
	}
}
