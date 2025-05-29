#include <Mesh/PolygonMesh.h>
#include <Mesh/BVH.h>
#include <Rendering/Core/RenderResourceManager.h>
#include <Rendering/Core/RenderProxyCollector.h>
#include <Core/Types.h>

#include <OpenMesh/Core/IO/reader/OBJReader.hh>
#include <glm/gtx/intersect.hpp>

#include <algorithm>

namespace
{
	constexpr auto HighlightedVertexColor = glm::vec3(0.95f);
	constexpr auto SelectedVertexColor = glm::vec3(0.96f, 0.66f, 0.25f);
	constexpr auto DeselectedVertexColor = glm::vec3(0.35f);
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
		if (indices.cols() == 3 || (indices.cols() == 4 && indices(i, 3) == -1))
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
	TrianglesListProxyBuffers solidMeshBuffers;

	bool containsQuads = false;
	bool nonTriQuad = false;
	for (auto faceIt = _mesh->faces_begin(); faceIt != _mesh->faces_end(); ++faceIt)
	{
		auto const val = faceIt->valence();
		if (val == 4)
		{
			containsQuads = true;
		}
		if (val > 4 || val < 3)
		{
			nonTriQuad = true;
		}
	}

	PolyMeshKernel triMesh;

	if (!containsQuads || nonTriQuad)
	{
		triMesh = *_mesh;
	}
	else
	{
		for (PolyMeshKernel::VertexIter v_it = _mesh->vertices_begin(); v_it != _mesh->vertices_end(); ++v_it) {
			triMesh.add_vertex(_mesh->point(*v_it));
		}

		for (auto faceIt = _mesh->faces_begin(); faceIt != _mesh->faces_end(); ++faceIt)
		{
			std::vector<PolyMeshKernel::VertexHandle> face_vertices;

			for (auto fv_it = _mesh->fv_iter(*faceIt); fv_it.is_valid(); ++fv_it)
			{
				face_vertices.push_back(*fv_it);
			}

			assert(faceIt->valence() == face_vertices.size());

			if (faceIt->valence() == 3)
			{
				std::vector<PolyMeshKernel::VertexHandle> triangle;
				triangle.push_back(PolyMeshKernel::VertexHandle(face_vertices[0].idx()));
				triangle.push_back(PolyMeshKernel::VertexHandle(face_vertices[1].idx()));
				triangle.push_back(PolyMeshKernel::VertexHandle(face_vertices[2].idx()));
				triMesh.add_face(triangle);
			}
			else
			{
				std::vector<PolyMeshKernel::VertexHandle> triangle1;
				triangle1.push_back(PolyMeshKernel::VertexHandle(face_vertices[0].idx()));
				triangle1.push_back(PolyMeshKernel::VertexHandle(face_vertices[1].idx()));
				triangle1.push_back(PolyMeshKernel::VertexHandle(face_vertices[2].idx()));
				triMesh.add_face(triangle1);

				std::vector<PolyMeshKernel::VertexHandle> triangle2;
				triangle2.push_back(PolyMeshKernel::VertexHandle(face_vertices[0].idx()));
				triangle2.push_back(PolyMeshKernel::VertexHandle(face_vertices[2].idx()));
				triangle2.push_back(PolyMeshKernel::VertexHandle(face_vertices[3].idx()));
				triMesh.add_face(triangle2);
			}
		}
	}

	if (nonTriQuad)
	{
		triMesh.triangulate();
	}

	triMesh.request_vertex_normals();

	const auto numVertices = triMesh.n_vertices();

	// Triangulate the mesh before we build the render proxy.
	{
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

	PointsListProxyBuffers pointsListBuffers;
	EdgesListProxyBuffers edgesListBuffers;
	TrianglesListProxyBuffers polysListBuffers;

	if (_supportsWireframeRendering)
	{
		// Populate the points buffers.
		{
			pointsListBuffers._vertices = ComputePointsWireframeVertices();

			pointsListBuffers._indices.resize(numVertices);

			for (std::size_t i = 0; i < numVertices; ++i)
			{
				pointsListBuffers._indices[i] = static_cast<uint32_t>(i);
			}
		}

		// Populate the edges buffers.
		{
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

		// Populate the polygons buffers.
		{
			polysListBuffers._positions.reserve(numVertices);
			polysListBuffers._vertices.reserve(numVertices);
			polysListBuffers._indices.reserve(3 * numVertices);

			for (auto faceIt = triMesh.faces_begin(); faceIt != triMesh.faces_end(); ++faceIt)
			{
				const auto polyStatus = triMesh.status(*faceIt);
				const auto isSelected = (polyStatus.selected() || polyStatus.tagged());

				auto vertexColor = DeselectedVertexColor;

				if (polyStatus.tagged())
				{
					vertexColor = HighlightedVertexColor;
				}
				else if (polyStatus.selected())
				{
					vertexColor = SelectedVertexColor;
				}

				for (auto vertexIt = triMesh.cfv_begin(static_cast<FaceHandle>(*faceIt)); vertexIt != triMesh.cfv_end(static_cast<FaceHandle>(*faceIt)); ++vertexIt)
				{
					polysListBuffers._positions.emplace_back(triMesh.point(static_cast<VertexHandle>(*vertexIt)));
					polysListBuffers._vertices.emplace_back(triMesh.normal(static_cast<VertexHandle>(*vertexIt)), vertexColor);
				}

				// Whenever we have a selected face we add the indices of the triangle to be rendered.
				if (isSelected)
				{
					for (auto vertexIt = triMesh.cfv_begin(static_cast<FaceHandle>(*faceIt)); vertexIt != triMesh.cfv_end(static_cast<FaceHandle>(*faceIt)); ++vertexIt)
					{
						polysListBuffers._indices.push_back(vertexIt->idx());
					}
				}
			}
		}
	}

	auto renderProxy = std::make_unique<PolygonMeshRenderProxy>(device,
		renderCommandScheduler,
		_solidPipeline,
		_wireframePipeline,
		static_cast<bool>(_supportsWireframeRendering),
		_wireframeRenderMode,
		solidMeshBuffers,
		pointsListBuffers,
		edgesListBuffers,
		polysListBuffers);
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

ClosestPolygonResult PolygonMesh::QueryRayHit(const Ray& ray) const
{
	float closestT = (std::numeric_limits<float>::max)();
	FaceHandle closestFaceHandle{ };

	for (auto it = _mesh->faces_begin(); it != _mesh->faces_end(); ++it)
	{
		// Store the first point of the face as the origin of each triangle.
		const auto endVertexIt = _mesh->cfv_end(static_cast<FaceHandle>(*it));;
		auto originVertexIt = _mesh->cfv_begin(static_cast<FaceHandle>(*it));
		const auto pointA = GetPosition(static_cast<VertexHandle>(*originVertexIt));

		// Get the next and the points after that of the face that we will increment and use as second and third points.
		auto prevVertexIt = ++originVertexIt;

		auto nextVertexIt = prevVertexIt;
		++nextVertexIt;

		do
		{
			const auto pointB = GetPosition(static_cast<VertexHandle>(*prevVertexIt));
			const auto pointC = GetPosition(static_cast<VertexHandle>(*nextVertexIt));

			glm::vec2 barycenter(0.0f);
			float dist = (std::numeric_limits<float>::max)();

			if (glm::intersectRayTriangle(ray._origin, ray._direction, pointA, pointB, pointC, barycenter, dist) && dist < closestT)
			{
				closestFaceHandle = static_cast<FaceHandle>(*it);
				closestT = dist;
			}

			++prevVertexIt;
			++nextVertexIt;
		} while (nextVertexIt != endVertexIt);
	}

	if (closestFaceHandle.is_valid())
	{
		return ClosestPolygonResult{ closestFaceHandle, closestT };
	}

	return ClosestPolygonResult{ FaceHandle(), (std::numeric_limits<float>::max)() };
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

		return ClosestVertexResult{ vertexHandle,
			GetPosition(vertexHandle),
			GetNormal(vertexHandle) };
	}

	return { };
}

std::optional<ClosestPolygonResult> PolygonMesh::QueryClosestPolygonScreenSpace(const ViewInfo& viewInfo,
	const glm::vec2 screenSpacePosition,
	const float minScreenSpaceDistance) const
{
	FaceHandle closestFaceHandle{ };
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
		return ClosestPolygonResult{ closestFaceHandle, minDistance };
	}

	return { };
}

std::optional<ClosestEdgeResult> PolygonMesh::QueryClosestEdgeScreenSpace(const ViewInfo& viewInfo,
	const glm::vec2 screenSpacePosition,
	const float minScreenSpaceDistance) const
{
	EdgeHandle closestEdgeHandle{ };
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
		return ClosestEdgeResult{ closestEdgeHandle };
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
		_cachedProjectedPoints[it->idx()] = viewInfo.ProjectWorldToScreen(GetPosition(static_cast<VertexHandle>(*it)));
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

			_renderProxy->SetWireframePositions(WireframeRenderMode::Points, wireframePositions);

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

			_renderProxy->SetWireframePositions(WireframeRenderMode::Edges, edgesPositions);

			std::vector<glm::vec3> polygonsPositions;
			polygonsPositions.reserve(numVertices);

			for (auto faceIt = triMesh.faces_begin(); faceIt != triMesh.faces_end(); ++faceIt)
			{
				for (auto vertexIt = triMesh.cfv_begin(static_cast<FaceHandle>(*faceIt)); vertexIt != triMesh.cfv_end(static_cast<FaceHandle>(*faceIt)); ++vertexIt)
				{
					polygonsPositions.emplace_back(triMesh.point(static_cast<VertexHandle>(*vertexIt)));
				}
			}

			_renderProxy->SetWireframePositions(WireframeRenderMode::Polygons, polygonsPositions);
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
			_renderProxy->SetWireframeVertices(WireframeRenderMode::Points, wireframeVertices);
		}
	}

	if (IsSet(_wireframeDirtyFlags, MeshProxyDirtyFlags::Color) || IsSet(_wireframeDirtyFlags, MeshProxyDirtyFlags::Normal))
	{
		CheckFormat(_supportsWireframeRendering, "The polygon object doesn't support wireframe rendering, but attempts to re-create the vertex buffer.");

		auto pointsWireframeVertices = ComputePointsWireframeVertices();
		_renderProxy->SetWireframeVertices(WireframeRenderMode::Points, pointsWireframeVertices);

		auto edgesWireframeVertices = ComputeEdgesWireframeVertices();
		_renderProxy->SetWireframeVertices(WireframeRenderMode::Edges, edgesWireframeVertices);

		std::size_t numSelected = 0;

		for (auto faceIt = triMesh.faces_begin(); faceIt != triMesh.faces_end(); ++faceIt)
		{
			if (triMesh.status(*faceIt).selected() || triMesh.status(*faceIt).tagged())
			{
				++numSelected;
			}
		}

		std::vector<Vertex> polygonsWireframeVertices;
		polygonsWireframeVertices.reserve(numVertices);

		std::vector<uint32_t> polygonsWireframeIndices;
		polygonsWireframeIndices.reserve(3 * numSelected);

		for (auto faceIt = triMesh.faces_begin(); faceIt != triMesh.faces_end(); ++faceIt)
		{
			const auto polyStatus = triMesh.status(*faceIt);
			const auto isSelected = (polyStatus.selected() || polyStatus.tagged());

			auto vertexColor = DeselectedVertexColor;

			if (polyStatus.tagged())
			{
				vertexColor = HighlightedVertexColor;
			}
			else if (polyStatus.selected())
			{
				vertexColor = SelectedVertexColor;
			}

			for (auto vertexIt = triMesh.cfv_begin(static_cast<FaceHandle>(*faceIt)); vertexIt != triMesh.cfv_end(static_cast<FaceHandle>(*faceIt)); ++vertexIt)
			{
				polygonsWireframeVertices.emplace_back(triMesh.normal(static_cast<VertexHandle>(*vertexIt)), vertexColor);
			}

			// Whenever we have a selected face we add the indices of the triangle to be rendered.
			if (isSelected)
			{
				polygonsWireframeIndices.push_back(polygonsWireframeVertices.size() - 3);
				polygonsWireframeIndices.push_back(polygonsWireframeVertices.size() - 2);
				polygonsWireframeIndices.push_back(polygonsWireframeVertices.size() - 1);
			}
		}

		_renderProxy->SetWireframeVertices(WireframeRenderMode::Polygons, polygonsWireframeVertices);
		_renderProxy->SetWireframeIndices(WireframeRenderMode::Polygons, polygonsWireframeIndices);
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
