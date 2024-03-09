#include <Mesh/PolygonMesh.h>
#include <Mesh/BVH.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Core/Types.h>

#include <OpenMesh/Core/IO/reader/OBJReader.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>

#include <algorithm>
#include <execution>

namespace
{
	constexpr auto HighlightedVertexColor = glm::vec3(0.95f);
	constexpr auto SelectedVertexColor = glm::vec3(0.6f);
	constexpr auto DeselectedVertexColor = glm::vec3(0.2f);
}

PolygonMesh::PolygonMesh(const Eigen::MatrixXd& vertices,
						 const Eigen::MatrixXi& indices,
						 MeshProxySolidPipeline solidPipeline,
						 MeshProxyWireframePipeline wireframePipeline,
						 const bool supportsWireframeRendering,
						 const WireframeRenderMode wireframeRenderMode)
	: _solidPipeline(std::move(solidPipeline))
	, _wireframePipeline(std::move(wireframePipeline))
	, _wireframeRenderMode(wireframeRenderMode)
	, _isVisible(true)
	, _supportsWireframeRendering(supportsWireframeRendering)
	, _isCachedGeometryDirty(true)
{
	_mesh = std::make_unique<PolyMeshKernel>();
	_mesh->reserve(vertices.size(), vertices.size() + indices.size(), indices.size());

	_mesh->request_vertex_normals();
	_mesh->request_vertex_colors();
	_mesh->request_vertex_status();
	_mesh->request_face_normals();

	std::vector<OpenMesh::SmartVertexHandle> vertexHandlesMap(vertices.size());

	for (auto i = 0; i < vertices.rows(); ++i)
	{
		vertexHandlesMap[i] = _mesh->add_vertex(glm::vec3(vertices(i, 0), vertices(i, 1), vertices(i, 2)));
	}

	CheckFormat(indices.cols() == 3 || indices.cols() == 4, "Only triangular or quad meshes are supported.");

	for (auto i = 0; i < indices.rows(); ++i)
	{
		if (indices.cols() == 3)
		{
			const auto vertexHandleA = static_cast<VertexHandle>(vertexHandlesMap[indices(i, 0)]);
			const auto vertexHandleB = static_cast<VertexHandle>(vertexHandlesMap[indices(i, 1)]);
			const auto vertexHandleC = static_cast<VertexHandle>(vertexHandlesMap[indices(i, 2)]);

			_mesh->add_face(vertexHandleA, vertexHandleB, vertexHandleC);
		}
		else
		{
			const auto vertexHandleA = static_cast<VertexHandle>(vertexHandlesMap[indices(i, 0)]);
			const auto vertexHandleB = static_cast<VertexHandle>(vertexHandlesMap[indices(i, 1)]);
			const auto vertexHandleC = static_cast<VertexHandle>(vertexHandlesMap[indices(i, 2)]);
			const auto vertexHandleD = static_cast<VertexHandle>(vertexHandlesMap[indices(i, 3)]);

			_mesh->add_face(vertexHandleA, vertexHandleB, vertexHandleC, vertexHandleD);
		}
	}

	_mesh->update_face_normals();
	_mesh->update_vertex_normals();
}

PolygonMesh::PolygonMesh(const MeshGeometry& geom,
	MeshProxySolidPipeline solidPipeline,
	MeshProxyWireframePipeline wireframePipeline,
	const bool supportsWireframeRendering,
	const WireframeRenderMode wireframeRenderMode)
	: _solidPipeline(std::move(solidPipeline))
	, _wireframePipeline(std::move(wireframePipeline))
	, _wireframeRenderMode(wireframeRenderMode)
	, _isVisible(true)
	, _supportsWireframeRendering(supportsWireframeRendering)
	, _isCachedGeometryDirty(true)
{
	_mesh = std::make_unique<PolyMeshKernel>();
	_mesh->reserve(geom._positions.size(), geom._positions.size() + geom._indices.size(), geom._indices.size());

	_mesh->request_vertex_normals();
	_mesh->request_vertex_colors();
	_mesh->request_vertex_status();
	_mesh->request_face_normals();

	std::vector<OpenMesh::SmartVertexHandle> vertexHandlesMap(geom._positions.size());

	for (std::size_t i = 0; i < geom._positions.size(); ++i)
	{
		vertexHandlesMap[i] = _mesh->add_vertex(geom._positions[i]);
	}

	for (std::size_t i = 0; i < geom._indices.size(); i += 3)
	{
		const auto vertexHandleA = static_cast<VertexHandle>(vertexHandlesMap[geom._indices[i]]);
		const auto vertexHandleB = static_cast<VertexHandle>(vertexHandlesMap[geom._indices[i + 1]]);
		const auto vertexHandleC = static_cast<VertexHandle>(vertexHandlesMap[geom._indices[i + 2]]);

		_mesh->add_face(vertexHandleA, vertexHandleB, vertexHandleC);
	}

	if (!geom._colors.empty())
	{
		_mesh->request_vertex_colors();
		for (std::size_t i = 0; i < geom._colors.size(); ++i)
		{
			_mesh->set_color(static_cast<VertexHandle>(vertexHandlesMap[i]), geom._colors[i]);
		}
	}

	_mesh->update_face_normals();
	_mesh->update_vertex_normals();
}

void PolygonMesh::CollectRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector,
	const RenderResourceRef<Device>& device,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler)
{
	// Triangulate the mesh before we build the render proxy.
	SolidMeshProxyBuffers solidMeshBuffers;
	{
		auto triMesh = *_mesh;
		triMesh.triangulate();

		const auto numVertices = triMesh.n_vertices();

		solidMeshBuffers._positions.resize(numVertices);
		memcpy(solidMeshBuffers._positions.data(), triMesh.points(), numVertices * sizeof(glm::vec3));

		{
			solidMeshBuffers._vertices.reserve(numVertices);

			const PolyMeshKernel::VertexIter vertexEndIt(triMesh.vertices_end());
			for (auto vertexIt = triMesh.vertices_begin(); vertexIt != vertexEndIt; ++vertexIt)
			{
				solidMeshBuffers._vertices.emplace_back(triMesh.normal(static_cast<VertexHandle>(*vertexIt)), triMesh.color(static_cast<VertexHandle>(*vertexIt)));
			}
		}

		// Extract the indices from each face.
		{
			solidMeshBuffers._indices.reserve(4 * triMesh.n_faces());

			for (auto faceIt = triMesh.faces_begin(); faceIt != triMesh.faces_end(); ++faceIt)
			{
				auto vertexIt = triMesh.cfv_begin(static_cast<FaceHandle>(*faceIt));

				for (; vertexIt != triMesh.cfv_end(static_cast<FaceHandle>(*faceIt)); ++vertexIt)
				{
					solidMeshBuffers._indices.emplace_back(vertexIt->idx());
				}
			}
		}
	}

	PointsMeshProxyBuffers pointsBuffers;
	EdgesListProxyBuffers edgesListBuffers;

	if (_supportsWireframeRendering)
	{
		pointsBuffers._positions.resize(_mesh->n_vertices());
		memcpy(pointsBuffers._positions.data(), _mesh->points(), _mesh->n_vertices() * sizeof(glm::vec3));

		pointsBuffers._vertices = ComputePointsWireframeVertices();

		pointsBuffers._indices.resize(pointsBuffers._positions.size());

		for (std::size_t i = 0; i < pointsBuffers._positions.size(); ++i)
		{
			pointsBuffers._indices[i] = static_cast<uint32_t>(i);
		}

		edgesListBuffers._positions.reserve(2 * _mesh->n_edges());
		edgesListBuffers._vertices.reserve(2 * _mesh->n_edges());
		edgesListBuffers._indices.reserve(_mesh->n_edges());

		// Extract the edge lines from the original mesh. For each of them we have to insert vertices double, otherwise
		// we cannot color them correctly in the shader.
		for (auto it = _mesh->edges_begin(); it != _mesh->edges_end(); ++it)
		{
			// Insert the points positions from the mesh.
			edgesListBuffers._positions.push_back(_mesh->point(static_cast<VertexHandle>(it->v0())));
			edgesListBuffers._positions.push_back(_mesh->point(static_cast<VertexHandle>(it->v1())));

			// If the edge is highlighted or selected then we set its vertices color to be set.
			const auto edgeStatus = _mesh->status(static_cast<EdgeHandle>(*it));
			glm::vec3 vertexColor = DeselectedVertexColor;

			if (edgeStatus.tagged())
			{
				vertexColor = HighlightedVertexColor;
			}
			else if (edgeStatus.selected())
			{
				vertexColor = SelectedVertexColor;
			}

			edgesListBuffers._vertices.emplace_back(_mesh->normal(static_cast<VertexHandle>(it->v0())), vertexColor);
			edgesListBuffers._vertices.emplace_back(_mesh->normal(static_cast<VertexHandle>(it->v1())), vertexColor);

			// Insert the indices into the buffer.
			edgesListBuffers._indices.push_back(edgesListBuffers._positions.size() - 2);
			edgesListBuffers._indices.push_back(edgesListBuffers._positions.size() - 1);
		}
	}

	auto renderProxy = std::make_unique<PolygonMeshRenderProxy>(device,
		renderCommandScheduler,
		_solidPipeline,
		_wireframePipeline,
		static_cast<bool>(_supportsWireframeRendering),
		_wireframeRenderMode,
		solidMeshBuffers,
		pointsBuffers,
		edgesListBuffers);
	renderProxy->SetModelMatrix(_modelMatrix);
	renderProxy->SetWireframeRenderMode(_wireframeRenderMode);

	_renderProxyHandle = renderProxyCollector->RegisterRenderProxy(std::move(renderProxy));
	_renderProxy = static_cast<PolygonMeshRenderProxy*>(renderProxyCollector->GetRenderProxy(_renderProxyHandle));
}

void PolygonMesh::DestroyRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector)
{
	renderProxyCollector->MarkRenderProxyAsDestroyed(_renderProxyHandle);

	_renderProxy = nullptr;
	_renderProxyHandle = InvalidHandle;
}

EigenMesh PolygonMesh::CopyAsEigen() const
{
	EigenMesh result;
	result._vertices.resize(3, static_cast<Eigen::Index>(GetNumVertices()));

	const auto& positions = _mesh->points();

	for (auto i = 0; i < result._vertices.cols(); ++i)
	{
		for (auto j = 0; j < result._vertices.rows(); ++j)
		{
			result._vertices(j, i) = positions[i][j];
		}
	}

	// Check the number of elements in the faces and if we have at least 1 quad face we treat the mesh as a quad.
	Eigen::Index maxNumFaceVertices = 0;
	for (auto faceIt = _mesh->faces_begin(); faceIt != _mesh->faces_end(); ++faceIt)
	{
		Eigen::Index numVerticesPerFace = 0;

		auto vertexIt = _mesh->cfv_ccwbegin(static_cast<FaceHandle>(*faceIt));
		for (; vertexIt != _mesh->cfv_ccwend(static_cast<FaceHandle>(*faceIt)); ++vertexIt)
		{
			++numVerticesPerFace;
		}

		maxNumFaceVertices = (std::max)(maxNumFaceVertices, numVerticesPerFace);

		CheckFormat(numVerticesPerFace <= 4, "Cannot have more than 4 vertices per face.");
	}

	CheckFormat(maxNumFaceVertices == 3 || maxNumFaceVertices == 4, "Faces should only be composed of triangles or quads.");
	result._faces.resize(maxNumFaceVertices, static_cast<Eigen::Index>(GetNumFaces()));

	std::size_t columnIndex = 0;
	for (auto faceIt = _mesh->faces_begin(); faceIt != _mesh->faces_end(); ++faceIt, ++columnIndex)
	{
		std::size_t rowIndex = 0;

		auto vertexIt = _mesh->cfv_ccwbegin(static_cast<FaceHandle>(*faceIt));
		const auto endVertexIt = _mesh->cfv_ccwend(static_cast<FaceHandle>(*faceIt));
		for (; vertexIt != endVertexIt; ++vertexIt, ++rowIndex)
		{
			result._faces(static_cast<Eigen::Index>(rowIndex), static_cast<Eigen::Index>(columnIndex)) = vertexIt->idx();
		}
	}

	result._vertices.transposeInPlace();
	result._faces.transposeInPlace();

	return result;
}

MeshGeometry PolygonMesh::GetGeometry() const
{
	// Copy and triangulate the mesh before we build the geometry buffers.
	auto triMesh = *_mesh;
	triMesh.triangulate();

	MeshGeometry geom;
	geom._positions = std::vector(triMesh.points(), triMesh.points() + triMesh.n_vertices());
	geom._indices.reserve(4 * triMesh.n_faces());

	// Extract the indices from each face.
	{
		const PolyMeshKernel::ConstFaceIter faceEndIt(triMesh.faces_end());
		for (auto faceIt = triMesh.faces_begin(); faceIt != faceEndIt; ++faceIt)
		{
			auto vertexIt = triMesh.cfv_begin(static_cast<FaceHandle>(*faceIt));

			for (; vertexIt != triMesh.cfv_end(static_cast<FaceHandle>(*faceIt)); ++vertexIt)
			{
				geom._indices.push_back(vertexIt->idx());
			}
		}
	}

	return geom;
}

void PolygonMesh::SetModelMatrix(const glm::mat4& modelMatrix)
{
	CheckFormat(_renderProxy != nullptr, "Proxy cannot be a nullptr.");

	_modelMatrix = modelMatrix;

	_renderProxy->SetModelMatrix(modelMatrix);
}

const glm::mat4& PolygonMesh::GetModelMatrix() const
{
	return _modelMatrix;
}

void PolygonMesh::SetVisible(const bool isVisible)
{
	CheckFormat(_renderProxy != nullptr, "Proxy cannot be a nullptr.");

	_isVisible = isVisible;

	_renderProxy->SetVisible(_isVisible);
}

bool PolygonMesh::IsVisible() const
{
	return _isVisible;
}

void PolygonMesh::SetColor(const glm::vec3& color)
{
	// Update the vertex colors of the original mesh, so if we re-create the proxy we get the correct ones.
	for (auto it = _mesh->vertices_begin(); it != _mesh->vertices_end(); ++it)
	{
		_mesh->set_color(static_cast<VertexHandle>(*it), color);
	}

	_dirtyFlags |= MeshProxyDirtyFlags::Color;

	UpdateRenderProxy();
}

void PolygonMesh::SetPositions(const std::span<glm::vec3> positions)
{
	CheckFormat(positions.size() == _mesh->n_vertices(), "The new positions count should be the same as the original.");

	memcpy(const_cast<glm::vec3*>(_mesh->points()), positions.data(), positions.size_bytes());

	_mesh->update_normals();

	_dirtyFlags |= MeshProxyDirtyFlags::Position;
	_dirtyFlags |= MeshProxyDirtyFlags::Normal;

	UpdateRenderProxy();

	// Mark the cached points as dirty so we can do the de-projection again.
	_isCachedGeometryDirty = true;
}

void PolygonMesh::SetColors(const std::span<glm::vec3> colors, const bool drawInfluenceMap)
{
	CheckFormat(colors.size() == _mesh->n_vertices(), "The new positions count should be the same as the original.");

	memcpy(const_cast<glm::vec3*>(_mesh->vertex_colors()), colors.data(), colors.size_bytes());

	_renderProxy->SetDrawInfluenceMap(drawInfluenceMap);

	_dirtyFlags |= MeshProxyDirtyFlags::Color;

	UpdateRenderProxy();
}

std::optional<float> PolygonMesh::QueryRayHit(const Ray& ray, const glm::mat4& transform) const
{
	float closestT = std::numeric_limits<float>::max();
	int32_t closestTriIndex = -1;

	const auto numVertices = _mesh->n_vertices();
	const auto& points = std::vector(_mesh->points(), _mesh->points() + numVertices);

	std::vector<glm::vec3> transformedPoints(_mesh->n_vertices());
	std::transform(points.begin(),
		points.end(),
		transformedPoints.begin(),
		[&transform](const auto& point)
		{
			const auto transformedPoint = transform * glm::vec4(point, 1.0f);

			return glm::vec3(transformedPoint);
		});

	const PolyMeshKernel::ConstFaceIter faceEndIt(_mesh->faces_end());
	for (auto faceIt = _mesh->faces_begin(); faceIt != faceEndIt; ++faceIt)
	{
		auto vertexIt = _mesh->cfv_begin(static_cast<FaceHandle>(*faceIt));
		const auto vA = vertexIt->idx();
		const auto vB = (++vertexIt)->idx();
		const auto vC = (++vertexIt)->idx();
		const auto hitT = GeometryUtils::IntersectRayTriangle(ray, transformedPoints[vA], transformedPoints[vB], transformedPoints[vC]);

		if (hitT.has_value() && hitT.value() < closestT)
		{
			closestT = hitT.value();
			closestTriIndex = faceIt->idx();
		}
	}

	if (closestTriIndex == -1)
	{
		return { };
	}

	return closestT;
}

std::optional<ClosestVertexResult> PolygonMesh::QueryClosestPointScreenSpace(const ViewInfo& viewInfo,
	const glm::vec2 screenSpacePosition,
	const float minScreenSpaceDistance) const
{
	std::size_t closestVertexIndex = std::numeric_limits<std::size_t>::max();
	float minDistanceSq = minScreenSpaceDistance * minScreenSpaceDistance;

	for (std::size_t pointIndex = 0; pointIndex < _cachedProjectedPoints.size(); ++pointIndex)
	{
		const auto& point = _cachedProjectedPoints[pointIndex];
		const auto scaledPoint = glm::vec2(point / point.z);
		const auto distSq = glm::length2(screenSpacePosition - scaledPoint);

		if (distSq < minDistanceSq)
		{
			minDistanceSq = distSq;
			closestVertexIndex = pointIndex;
		}
	}

	if (closestVertexIndex != std::numeric_limits<std::size_t>::max())
	{
		const VertexHandle vertexHandle(static_cast<int>(closestVertexIndex));

		return ClosestVertexResult { vertexHandle,
			_mesh->point(vertexHandle),
			_mesh->normal(vertexHandle) };
	}

	return { };
}

std::optional<ClosestPolygonResult> PolygonMesh::QueryClosestPolygonScreenSpace(const ViewInfo& viewInfo,
	const glm::vec2 screenSpacePosition,
	const float minScreenSpaceDistance) const
{
	FaceHandle closestFaceHandle { };
	float minDistance = minScreenSpaceDistance;

	for (auto it = _mesh->faces_begin(); it != _mesh->faces_end(); ++it)
	{
		auto vertexIt = _mesh->cfv_begin(static_cast<FaceHandle>(*it));
		const auto vA = vertexIt->idx();
		const auto vB = (++vertexIt)->idx();
		const auto vC = (++vertexIt)->idx();

		const auto& pointA = _cachedProjectedPoints[vA];
		const auto scaledPointA = glm::vec2(pointA / pointA.z);

		const auto& pointB = _cachedProjectedPoints[vB];
		const auto scaledPointB = glm::vec2(pointB / pointB.z);

		const auto& pointC = _cachedProjectedPoints[vC];
		const auto scaledPointC = glm::vec2(pointC / pointC.z);

		const auto dist = GeometryUtils::CalculateDistanceFromPointToTriangle2D(screenSpacePosition,
			scaledPointA,
			scaledPointB,
			scaledPointC);

		if (dist < minDistance)
		{
			minDistance = dist;
			closestFaceHandle = static_cast<FaceHandle>(*it);
		}
	}

	if (closestFaceHandle.is_valid())
	{
		return ClosestPolygonResult { closestFaceHandle, minDistance };
	}

	return { };
}

std::optional<ClosestEdgeResult> PolygonMesh::QueryClosestEdgeScreenSpace(const ViewInfo& viewInfo,
	const glm::vec2 screenSpacePosition,
	const float minScreenSpaceDistance) const
{
	EdgeHandle closestEdgeHandle { };
	float minDistance = minScreenSpaceDistance;

	for (auto it = _mesh->edges_begin(); it != _mesh->edges_end(); ++it)
	{
		const auto& pointA = _cachedProjectedPoints[it->v0().idx()];
		const auto scaledPointA = glm::vec2(pointA / pointA.z);

		const auto& pointB = _cachedProjectedPoints[it->v1().idx()];
		const auto scaledPointB = glm::vec2(pointB / pointB.z);

		const auto dist = GeometryUtils::PointLineDistance(screenSpacePosition, scaledPointA, scaledPointB);

		if (dist < minDistance)
		{
			minDistance = dist;
			closestEdgeHandle = static_cast<EdgeHandle>(*it);
		}
	}

	if (closestEdgeHandle.is_valid())
	{
		return ClosestEdgeResult { closestEdgeHandle };
	}

	return { };
}

void PolygonMesh::CacheProjectedPointsWorldToScreen(const ViewInfo& viewInfo)
{
	if (!_isCachedGeometryDirty)
	{
		return;
	}

	const auto numVertices = _mesh->n_vertices();

	if (numVertices != _cachedProjectedPoints.size())
	{
		_cachedProjectedPoints.clear();
		_cachedProjectedPoints.resize(numVertices);
	}

	for (auto it = _mesh->vertices_begin(); it != _mesh->vertices_end(); ++it)
	{
		_cachedProjectedPoints[it->idx()] = viewInfo.ProjectWorldToScreen(glm::vec3(_modelMatrix * glm::vec4(_mesh->point(static_cast<VertexHandle>(*it)), 1.0f)));
	}

	_isCachedGeometryDirty = false;
}

void PolygonMesh::UpdateRenderProxy()
{
	CheckFormat(_renderProxy != nullptr, "Proxy cannot be a nullptr.");

	if (_dirtyFlags == MeshProxyDirtyFlags::None && _wireframeDirtyFlags == MeshProxyDirtyFlags::None)
	{
		return;
	}

	// Triangulate the mesh before we build the render proxy.
	auto triMesh = *_mesh;
	triMesh.triangulate();

	const auto numVertices = triMesh.n_vertices();

	if (IsSet(_dirtyFlags, MeshProxyDirtyFlags::Position))
	{
		std::vector<glm::vec3> positions(numVertices);
		memcpy(positions.data(), triMesh.points(), numVertices * sizeof(glm::vec3));

		// Update the proxy positions and wireframe positions.
		_renderProxy->SetPositions(positions);

		if (_supportsWireframeRendering)
		{
			// We use the original mesh here, because we don't want the triangulated edges and vertices.
			std::vector<glm::vec3> wireframePositions(_mesh->n_vertices());
			memcpy(wireframePositions.data(), _mesh->points(), _mesh->n_vertices() * sizeof(glm::vec3));

			_renderProxy->SetPointsWireframePositions(wireframePositions);

			std::vector<glm::vec3> edgesPositions;
			edgesPositions.reserve(2 * _mesh->n_edges());

			// Extract the edge lines from the original mesh. For each of them we have to insert vertices double, otherwise
			// we cannot color them correctly in the shader.
			for (auto it = _mesh->edges_begin(); it != _mesh->edges_end(); ++it)
			{
				// Insert the points positions from the mesh.
				edgesPositions.push_back(_mesh->point(static_cast<VertexHandle>(it->v0())));
				edgesPositions.push_back(_mesh->point(static_cast<VertexHandle>(it->v1())));
			}

			_renderProxy->SetEdgesWireframePositions(edgesPositions);
		}
	}

	if (IsSet(_dirtyFlags, MeshProxyDirtyFlags::Normal) || IsSet(_dirtyFlags, MeshProxyDirtyFlags::Color))
	{
		std::vector<Vertex> vertices;
		vertices.reserve(numVertices);

		const PolyMeshKernel::VertexIter vertexEndIt(triMesh.vertices_end());
		for (auto vertexIt = triMesh.vertices_begin(); vertexIt != vertexEndIt; ++vertexIt)
		{
			vertices.emplace_back(triMesh.normal(static_cast<VertexHandle>(*vertexIt)), triMesh.color(static_cast<VertexHandle>(*vertexIt)));
		}

		_renderProxy->SetVertices(vertices);

		if (_supportsWireframeRendering)
		{
			auto wireframeVertices = ComputePointsWireframeVertices();
			_renderProxy->SetPointsWireframeVertices(wireframeVertices);
		}
	}

	if (IsSet(_wireframeDirtyFlags, MeshProxyDirtyFlags::Position))
	{
		CheckFormat(_supportsWireframeRendering, "The polygon object doesn't support wireframe rendering, but attempts to re-create the vertex buffer.");

		// We use the original mesh here, because we don't want the triangulated edges and vertices.
		std::vector<glm::vec3> wireframePositions(_mesh->n_vertices());
		memcpy(wireframePositions.data(), _mesh->points(), _mesh->n_vertices() * sizeof(glm::vec3));

		_renderProxy->SetPointsWireframePositions(wireframePositions);
	}

	if (IsSet(_wireframeDirtyFlags, MeshProxyDirtyFlags::Color) || IsSet(_wireframeDirtyFlags, MeshProxyDirtyFlags::Normal))
	{
		CheckFormat(_supportsWireframeRendering, "The polygon object doesn't support wireframe rendering, but attempts to re-create the vertex buffer.");

		auto pointsWireframeVertices = ComputePointsWireframeVertices();
		_renderProxy->SetPointsWireframeVertices(pointsWireframeVertices);

		auto edgesWireframeVertices = ComputeEdgesWireframeVertices();
		_renderProxy->SetEdgesWireframeVertices(edgesWireframeVertices);
	}

	// TODO: An optimization would be to batch those dirty flags resets and sets.
	_dirtyFlags = MeshProxyDirtyFlags::None;
	_wireframeDirtyFlags = MeshProxyDirtyFlags::None;
}

std::vector<Vertex> PolygonMesh::ComputePointsWireframeVertices() const
{
	std::vector<Vertex> vertices(_mesh->n_vertices());
	for (auto it = _mesh->vertices_begin(); it != _mesh->vertices_end(); ++it)
	{
		vertices[it->idx()]._normal = _mesh->normal(static_cast<VertexHandle>(*it));

		// Highlighted vertex is considered tagged.
		if (_mesh->status(static_cast<VertexHandle>(*it)).tagged())
		{
			vertices[it->idx()]._vertexColor = HighlightedVertexColor;
		}
		else if (_mesh->status(static_cast<VertexHandle>(*it)).selected())
		{
			vertices[it->idx()]._vertexColor = SelectedVertexColor;
		}
		else
		{
			vertices[it->idx()]._vertexColor = DeselectedVertexColor;
		}
	}

	return vertices;
}

std::vector<Vertex> PolygonMesh::ComputeEdgesWireframeVertices() const
{
	std::vector<Vertex> vertices;
	vertices.reserve(2 * _mesh->n_edges());

	for (auto it = _mesh->edges_begin(); it != _mesh->edges_end(); ++it)
	{
		// If the edge is highlighted or selected then we set its vertices color to be set.
		const auto edgeStatus = _mesh->status(static_cast<EdgeHandle>(*it));
		glm::vec3 vertexColor = DeselectedVertexColor;

		if (edgeStatus.tagged())
		{
			vertexColor = HighlightedVertexColor;
		}
		else if (edgeStatus.selected())
		{
			vertexColor = SelectedVertexColor;
		}

		vertices.emplace_back(_mesh->normal(static_cast<VertexHandle>(it->v0())), vertexColor);
		vertices.emplace_back(_mesh->normal(static_cast<VertexHandle>(it->v1())), vertexColor);
	}

	return vertices;
}
