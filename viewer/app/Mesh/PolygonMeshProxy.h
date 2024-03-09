#pragma once

#include <Rendering/Core/RenderProxy.h>
#include <Rendering/Core/Buffer.h>

#include <vector>

struct SolidMeshProxyBuffers
{
	std::vector<glm::vec3> _positions;
	std::vector<Vertex> _vertices;
	std::vector<uint32_t> _indices;
};

struct PointsMeshProxyBuffers
{
	[[nodiscard]] bool IsValid() const
	{
		return !_positions.empty() && !_vertices.empty() && !_indices.empty();
	}

	[[nodiscard]] std::size_t GetNumElements() const
	{
		return _positions.size();
	}

	std::vector<glm::vec3> _positions;
	std::vector<Vertex> _vertices;
	std::vector<uint32_t> _indices;
};

struct EdgesListProxyBuffers
{
	[[nodiscard]] bool IsValid() const
	{
		return !_positions.empty() && !_vertices.empty() && !_indices.empty();
	}

	[[nodiscard]] std::size_t GetNumElements() const
	{
		return _indices.size();
	}

	std::vector<glm::vec3> _positions;
	std::vector<Vertex> _vertices;
	std::vector<uint32_t> _indices;
};

struct ProxyGPUBuffers
{
	/**
	 * Releases the memory and the buffer from the GPU for all buffers.
	 * @todo Move that into the resource manager or somewhere else where we can track individual allocations.
	 * @param device A reference to the Vulkan device.
	 */
	void ReleaseResource(const RenderResourceRef<Device>& device) const
	{
		_positionBuffer.ReleaseResource(device);
		_vertexBuffer.ReleaseResource(device);
		_indexBuffer.ReleaseResource(device);
	}

	Buffer _positionBuffer;
	Buffer _vertexBuffer;
	Buffer _indexBuffer;
};

class PolygonMeshRenderProxy final : public MeshRenderProxy
{
public:
	PolygonMeshRenderProxy(const RenderResourceRef<Device>& device,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler,
		MeshProxySolidPipeline solidPipeline,
		MeshProxyWireframePipeline wireframePipeline,
		const bool supportsWireframeRendering,
		const WireframeRenderMode wireframeRenderMode,
		const SolidMeshProxyBuffers& solidMeshBuffers,
		const PointsMeshProxyBuffers& pointsWireframeBuffers,
		const EdgesListProxyBuffers& edgesWireframeBuffers);

	//~BEGIN RenderProxy
	void Render(const VkCommandBuffer commandBuffer,
		const uint32_t currentFrameIndex,
		const uint32_t dynamicBufferOffset) override;
	void DestroyRenderProxy(const RenderResourceRef<Device>& device) override;
	//~END RenderProxy

	/**
	 * Returns the input binding descriptions for a shader pipeline to render a mesh.
	 * @return The input bindings of a static mesh object.
	 */
	[[nodiscard]] static std::vector<VkVertexInputBindingDescription> GetBindingDescription();

	/**
	 * Returns the input attribute descriptions for a shader pipeline to render a mesh.
	 * @return The input attributes of a static mesh object.
	 */
	[[nodiscard]] static std::vector<VkVertexInputAttributeDescription> GetAttributeDescriptions();

	/**
	 * Whether the mesh should be drawn with its vertex colors as influence map.
	 * @param drawInfluenceMap Whether the mesh should be drawn with its vertex colors as influence map.
	 */
	void SetDrawInfluenceMap(const bool drawInfluenceMap);

	/**
	 * Updates the vertex buffer of the proxy.
	 * @param newVertices The new vertex buffer of the proxy.
	 */
	void SetVertices(const std::span<Vertex> newVertices);

	/**
	 * Updates the positions buffer of the proxy.
	 * @param newPositions The new positions buffer of the proxy.
	 */
	void SetPositions(const std::span<glm::vec3> newPositions);

	/**
	 * Set the vertices data in wireframe mode.
	 * @param newWireframeVertices The new wireframe vertices.
	 */
	void SetPointsWireframeVertices(const std::span<Vertex> newWireframeVertices);

	/**
	 * Updates the positions buffer of the proxy in wireframe mode.
	 * @param newWireframePositions The new positions buffer of the proxy in wireframe mode.
	 */
	void SetPointsWireframePositions(const std::span<glm::vec3> newWireframePositions);

	/**
	 * Set the vertices data in wireframe mode.
	 * @param newWireframeVertices The new wireframe vertices.
	 */
	void SetEdgesWireframeVertices(const std::span<Vertex> newWireframeVertices);

	/**
	 * Updates the positions buffer of the proxy in wireframe mode.
	 * @param newWireframePositions The new positions buffer of the proxy in wireframe mode.
	 */
	void SetEdgesWireframePositions(const std::span<glm::vec3> newWireframePositions);

private:
	ProxyGPUBuffers _solidMeshBuffers;
	ProxyGPUBuffers _pointsWireframeBuffers;
	ProxyGPUBuffers _edgesWireframeBuffers;

	std::size_t _numVertices;
	std::size_t _numIndices;

	std::size_t _numWireframePoints;
	std::size_t _numWireframeEdges;

	uint32_t _drawInfluenceMap : 1;
};
