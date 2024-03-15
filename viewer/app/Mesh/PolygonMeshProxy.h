#pragma once

#include <Rendering/Core/RenderProxy.h>
#include <Rendering/Core/Buffer.h>

#include <vector>

struct PointsListProxyBuffers
{
	[[nodiscard]] bool IsValid() const
	{
		return !_vertices.empty() && !_indices.empty();
	}

	[[nodiscard]] std::size_t GetNumElements() const
	{
		return _vertices.size();
	}

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
		return _positions.size();
	}

	std::vector<glm::vec3> _positions;
	std::vector<Vertex> _vertices;
	std::vector<uint32_t> _indices;
};

struct TrianglesListProxyBuffers
{
	[[nodiscard]] bool IsValid() const
	{
		return !_positions.empty() || !_vertices.empty() || !_indices.empty();
	}

	[[nodiscard]] std::size_t GetNumElements() const
	{
		return _indices.size();
	}

	std::vector<glm::vec3> _positions;
	std::vector<Vertex> _vertices;
	std::vector<uint32_t> _indices;
};

struct ProxyGPUBuffersNoPosition
{
	/**
	 * Releases the memory and the buffer from the GPU for all buffers.
	 * @todo Move that into the resource manager or somewhere else where we can track individual allocations.
	 * @param device A reference to the Vulkan device.
	 */
	void ReleaseResource(const RenderResourceRef<Device>& device) const
	{
		_vertexBuffer.ReleaseResource(device);
		_indexBuffer.ReleaseResource(device);
	}

	Buffer _vertexBuffer;
	Buffer _indexBuffer;
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
		const TrianglesListProxyBuffers& solidMeshBuffers,
		const PointsListProxyBuffers& pointsWireframeBuffers,
		const EdgesListProxyBuffers& edgesWireframeBuffers,
		const TrianglesListProxyBuffers& polysListBuffers);

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
	void SetWireframeVertices(const WireframeRenderMode renderMode, const std::span<Vertex> newWireframeVertices);

	/**
	 * Updates the positions buffer of the proxy in wireframe mode.
	 * @param newWireframePositions The new positions buffer of the proxy in wireframe mode.
	 */
	void SetWireframePositions(const WireframeRenderMode renderMode, const std::span<glm::vec3> newWireframePositions);

	/**
	 * Updates the positions buffer of the proxy in wireframe mode.
	 * @param newWireframePositions The new positions buffer of the proxy in wireframe mode.
	 */
	void SetWireframeIndices(const WireframeRenderMode renderMode, const std::span<uint32_t> newWireframeIndices);

private:
	ProxyGPUBuffers _solidMeshBuffers;
	ProxyGPUBuffersNoPosition _pointsWireframeBuffers;
	ProxyGPUBuffers _edgesWireframeBuffers;
	ProxyGPUBuffers _polygonsWireframeBuffers;

	std::size_t _numVertices;
	std::size_t _numIndices;

	/// Used to schedule the command for points wireframe mode.
	std::size_t _numWireframePoints;

	/// Used to schedule the command for edges wireframe mode.
	std::size_t _numWireframeEdges;

	/// Used to schedule the command for polygons wireframe mode.
	std::size_t _numWireframeVertices;
	std::size_t _numWireframeIndices;

	uint32_t _drawInfluenceMap : 1;
};
