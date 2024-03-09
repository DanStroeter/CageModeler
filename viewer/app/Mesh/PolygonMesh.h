#pragma once

#include <Rendering/Core/Device.h>
#include <Rendering/Core/RenderResource.h>
#include <Rendering/Core/Pipeline.h>
#include <Mesh/PolygonMeshKernel.h>
#include <Mesh/GeometryUtils.h>
#include <Mesh/PolygonMeshProxy.h>
#include <Mesh/Selection.h>

#include <filesystem>
#include <unordered_set>

struct ViewInfo;
class Camera;
class RenderProxyCollector;
class BVH;
class RenderResourceManager;
class RenderCommandScheduler;

struct ClosestVertexResult
{
	VertexHandle _vertexHandle;
	glm::vec3 _worldPosition;
	glm::vec3 _vertexNormal;
};

struct ClosestEdgeResult
{
	EdgeHandle _edgeHandle;
};

struct ClosestPolygonResult
{
	FaceHandle _polyHandle;
	float _distance = std::numeric_limits<float>::max();
};

class PolygonMesh
{
public:
	PolygonMesh()
		: _isVisible(true)
		, _supportsWireframeRendering(false)
		, _isCachedGeometryDirty(false)
	{ }

	PolygonMesh(const Eigen::MatrixXd& vertices,
		const Eigen::MatrixXi& indices,
		MeshProxySolidPipeline solidPipeline,
		MeshProxyWireframePipeline wireframePipeline,
		const bool supportsWireframeRendering,
		const WireframeRenderMode wireframeRenderMode);

	PolygonMesh(const MeshGeometry& geom,
		MeshProxySolidPipeline solidPipeline,
		MeshProxyWireframePipeline wireframePipeline,
		const bool supportsWireframeRendering,
		const WireframeRenderMode wireframeRenderMode);

	/**
	 * Creates a new render proxy for the mesh.
	 * @param renderProxyCollector A pointer to the render proxy collector that will register the new proxy instance.
	 * @param device A reference to the Vulkan device.
	 * @param renderCommandScheduler A pointer to the render command scheduler to pass to the proxy.
	 * @return A new instance of a render proxy.
	*/
	void CollectRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector,
		const RenderResourceRef<Device>& device,
		const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler);

	/**
	 * Destroy the render proxy associated with the mesh.
	 * @param renderProxyCollector A pointer to the render proxy collector that will deregister the new proxy instance.
	 */
	void DestroyRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector);

	/**
	 * Converts a static mesh into one that is used by the cage deformation algorithms.
	 * @return A new mesh used by the cage deformation algorithms.
	 */
	[[nodiscard]] EigenMesh CopyAsEigen() const;

	/**
	 * Gets a copy of the geometry data from the underlying mesh.
	 */
	[[nodiscard]] MeshGeometry GetGeometry() const;

	/**
	 * Update the model matrix of the object.
	 * @param modelMatrix The model matrix of the object.
	 */
	void SetModelMatrix(const glm::mat4& modelMatrix);

	/**
	 * Returns the model matrix of the object.
	 * @return The model matrix of the mesh.
	 */
	[[nodiscard]] const glm::mat4& GetModelMatrix() const;

	/**
	 * Sets the visibility of the object.
	 * @param isVisible The visibility of the mesh.
	 */
	void SetVisible(const bool isVisible);

	/**
	 * Returns the visibility of the mesh.
	 * @return The visibility of the mesh.
	 */
	[[nodiscard]] bool IsVisible() const;

	/**
	 * Updates the color of the mesh which will also mark the render proxy as dirty.
	 * @param color The new color of the mesh.
	 */
	void SetColor(const glm::vec3& color);

	[[nodiscard]] std::size_t GetNumVertices() const
	{
		return _mesh->n_vertices();
	}

	[[nodiscard]] std::size_t GetNumEdges() const
	{
		return _mesh->n_edges();
	}

	[[nodiscard]] std::size_t GetNumFaces() const
	{
		return _mesh->n_faces();
	}

	[[nodiscard]] glm::vec3 GetPosition(const VertexHandle handle) const
	{
		return _mesh->point(handle);
	}

	[[nodiscard]] glm::vec3 GetAverageVertexPosition(const VertexHandle handle) const
	{
		return _mesh->point(handle);
	}

	[[nodiscard]] glm::vec3 GetAverageVertexPosition(const FaceHandle handle) const
	{
		glm::vec3 position(0.0f);
		std::size_t numVertices = 0;

		for (auto it = _mesh->fv_begin(handle); it != _mesh->fv_end(handle); ++it)
		{
			position += _mesh->point(static_cast<VertexHandle>(*it));
			++numVertices;
		}

		return position / static_cast<float>((std::max)(1_sz, numVertices));
	}

	[[nodiscard]] std::unordered_set<VertexHandle> GetUniqueVerticesFromEdges(const std::span<EdgeHandle> handles) const
	{
		std::unordered_set<VertexHandle> uniqueVertexHandles;

		for (const auto handle : handles)
		{
			const auto heHandle = _mesh->halfedge_handle(handle, 0);
			const auto v0 = _mesh->from_vertex_handle(heHandle);
			const auto v1 = _mesh->to_vertex_handle(heHandle);

			uniqueVertexHandles.insert(v0);
			uniqueVertexHandles.insert(v1);
		}

		return uniqueVertexHandles;
	}

	template <typename HandleType>
	[[nodiscard]] glm::vec3 GetNormal(const HandleType handle) const
	{
		return _mesh->normal(handle);
	}

	/**
	 * Get the selection of a specific type.
	 */
	template <SelectionType Type>
	auto GetSelection() -> decltype(auto);

	/**
	 * Update the positions of the vertices of the mesh.
	 * @param positions The new positions of the vertices.
	 */
	void SetPositions(const std::span<glm::vec3> positions);

	/**
	 * Update the colors of the vertices of the mesh.
	 * @param colors The new colors of the vertices.
	 * @param drawInfluenceMap Whether the influence map should be drawn.
	 */
	void SetColors(const std::span<glm::vec3> colors, const bool drawInfluenceMap);

	/**
	 * Find the closest ray hit with a triangle of the mesh.
	 * @param ray A ray that should be tested against each triangle.
	 * @param transform Pre-transform the points using a transformation matrix.
	 * @return A hit parameter T along the ray.
	 */
	[[nodiscard]] std::optional<float> QueryRayHit(const Ray& ray, const glm::mat4& transform = glm::mat4(1.0f)) const;

	/**
	 * Queries the closest mesh vertex in screen space closest to the mouse cursor.
	 * @param viewInfo The view info of the camera.
	 * @param screenSpacePosition The position of the mouse in screen space.
	 * @param minScreenSpaceDistance The minimum distance to check against.
	 * @return The mesh vertex that is the closest to the mouse cursor.
	 */
	[[nodiscard]] std::optional<ClosestVertexResult> QueryClosestPointScreenSpace(const ViewInfo& viewInfo,
		const glm::vec2 screenSpacePosition,
		const float minScreenSpaceDistance) const;

	/**
	 * Queries the closest mesh edge in screen space closest to the mouse cursor.
	 * @param viewInfo The view info of the camera.
	 * @param screenSpacePosition The position of the mouse in screen space.
	 * @param minScreenSpaceDistance The minimum distance to check against.
	 * @return The mesh edge that is the closest to the mouse cursor.
	 */
	[[nodiscard]] std::optional<ClosestEdgeResult> QueryClosestEdgeScreenSpace(const ViewInfo& viewInfo,
		const glm::vec2 screenSpacePosition,
		const float minScreenSpaceDistance) const;

	/**
	 * Queries the closest mesh face in screen space closest to the mouse cursor.
	 * @param viewInfo The view info of the camera.
	 * @param screenSpacePosition The position of the mouse in screen space.
	 * @param minScreenSpaceDistance The minimum distance to check against.
	 * @return The mesh face that is the closest to the mouse cursor.
	 */
	std::optional<ClosestPolygonResult> QueryClosestPolygonScreenSpace(const ViewInfo& viewInfo,
		const glm::vec2 screenSpacePosition,
		const float minScreenSpaceDistance) const;

	/**
	 * Deproject all points from world into screen space.
	 * @param viewInfo The view info of the camera.
	 * @return Deproject all vertex positions from world to screen space.
	 */
	void CacheProjectedPointsWorldToScreen(const ViewInfo& viewInfo);

	/**
	 * Returns all points of the mesh projected into screen space.
	 * @return The cache of projected points into screen space.
	 */
	[[nodiscard]] const std::vector<glm::vec3>& GetProjectedPointsCache() const
	{
		return _cachedProjectedPoints;
	}

	/**
	 * Sets a flag whether the mesh should be rendered as wireframe.
	 * @param wireframeRenderMode A flag how the mesh should be rendered in wireframe mode.
	 */
	void SetWireframeRenderMode(const WireframeRenderMode wireframeRenderMode)
	{
		CheckFormat(_renderProxy != nullptr, "Proxy cannot be a nullptr.");

		_wireframeRenderMode = wireframeRenderMode;
		_renderProxy->SetWireframeRenderMode(wireframeRenderMode);
	}

	/**
	 * Marks specific flags of the proxy as dirty. This will retrigger a proxy update at the end of the frame.
	 * @param dirtyFlags Dirty flags.
	 */
	void AddProxyDirtyFlag(const MeshProxyDirtyFlags dirtyFlags)
	{
		_dirtyFlags |= dirtyFlags;
	}

	/**
	 * Marks specific flags of the proxy as dirty. This will retrigger a proxy update at the end of the frame.
	 * @param dirtyFlags Dirty flags.
	 */
	void AddWireframeProxyDirtyFlag(const MeshProxyDirtyFlags dirtyFlags)
	{
		_wireframeDirtyFlags |= dirtyFlags;
	}

	/**
	 * Checks whether the proxy is marked dirty.
	 * @return Whether or not the proxy is marked dirty.
	 */
	[[nodiscard]] bool IsDirty() const
	{
		return _wireframeDirtyFlags != MeshProxyDirtyFlags::None || _dirtyFlags != MeshProxyDirtyFlags::None;
	}

	/**
	 * Marks the cached geometry for de-projecting points from world to screen space as dirty, so we can lazily
	 * recompute them in the next query.
	 */
	void MarkCachedGeometryDirty()
	{
		// Mark the cached points as dirty so we can do the de-projection again.
		_isCachedGeometryDirty = true;
	}

	/**
	 * Copies over the mesh vertex positions into the render proxy wireframe data.
	 */
	void UpdateRenderProxy();

private:
	/**
	 * Creates a vector of vertices based on the underlying mesh to render the wireframe of it.
	 * @return A vector of vertices for the wireframe mesh.
	 */
	[[nodiscard]] std::vector<Vertex> ComputePointsWireframeVertices() const;

	/**
	 * Creates a vector of vertices based on the underlying mesh to render the wireframe of it.
	 * @return A vector of vertices for the wireframe mesh.
	 */
	[[nodiscard]] std::vector<Vertex> ComputeEdgesWireframeVertices() const;

private:
	template <typename T> friend struct ProxyCollectorHelper;

	std::unique_ptr<PolyMeshKernel> _mesh = nullptr;

	/// The pipeline used to render the mesh with a solid color.
	MeshProxySolidPipeline _solidPipeline;

	/// The pipeline used to render the wireframe mesh.
	/// TODO: Make this a separate object with its own proxy.
	MeshProxyWireframePipeline _wireframePipeline;

	/// TODO: No time to think of something better, but since the pointer will only change under special conditions it's ok.
	PolygonMeshRenderProxy* _renderProxy = nullptr;

	/// Stores the render proxy handle to destroy it from the proxy collector.
	RenderProxyHandle _renderProxyHandle = InvalidHandle;

	/// All dirty flags of the mesh so we can update the proxy when required.
	MeshProxyDirtyFlags _dirtyFlags = MeshProxyDirtyFlags::None;

	/// All dirty flags of the wireframe mesh so we can update the proxy when required.
	MeshProxyDirtyFlags _wireframeDirtyFlags = MeshProxyDirtyFlags::None;

	/// The model matrix of the object.
	glm::mat4 _modelMatrix { 1.0f };

	/// A vector of all points projected into screen space.
	std::vector<glm::vec3> _cachedProjectedPoints;

	/// Whether the mesh should render points.
	WireframeRenderMode _wireframeRenderMode = WireframeRenderMode::Edges;

	/// Whether the mesh should be rendered.
	uint32_t _isVisible : 1;

	/// Whether the mesh should ever support wireframe rendering.
	uint32_t _supportsWireframeRendering : 1;

	/// Dirty flag for the cached projected points.
	uint32_t _isCachedGeometryDirty : 1;
};

template <SelectionType Type>
auto PolygonMesh::GetSelection() -> decltype(auto)
{
	if constexpr (Type == SelectionType::Vertex)
	{
		return VertexSelection(*_mesh, _cachedProjectedPoints, _wireframeDirtyFlags);
	}
	else if constexpr (Type == SelectionType::Edge)
	{
		return EdgeSelection(*_mesh, _cachedProjectedPoints, _wireframeDirtyFlags);
	}
	else
	{
		return PolygonSelection(*_mesh, _cachedProjectedPoints, _wireframeDirtyFlags);
	}
}
