#pragma once

#include <Mesh/PolygonMeshKernel.h>
#include <Rendering/Scene/SceneData.h>

#include <span>
#include <unordered_set>

enum class SelectionType : uint8_t
{
	Vertex,
	Edge,
	Polygon
};

enum class MeshProxyDirtyFlags : uint8_t
{
	None = 0,
	Position = 1 << 0,
	Normal = 1 << 1,
	Color = 1 << 2,
	All = 1 << 3
};

template <typename HandleType>
struct SelectionHelpers
{
	template <typename F>
	static void Iterate(const PolyMeshKernel& mesh, F&& f)
	{
		CheckNoEntry("No specialization.");
	}

	static std::size_t GetNumElements(const PolyMeshKernel& mesh)
	{
		CheckNoEntry("No specialization.");

		return 0;
	}
};

template <>
struct SelectionHelpers<VertexHandle>
{
	template <typename F>
	static void Iterate(const PolyMeshKernel& mesh, F&& f)
	{
		for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it)
		{
			f(static_cast<VertexHandle>(*it));
		}
	}

	static std::size_t GetNumElements(const PolyMeshKernel& mesh)
	{
		return mesh.n_vertices();
	}
};

template <>
struct SelectionHelpers<EdgeHandle>
{
	template <typename F>
	static void Iterate(PolyMeshKernel& mesh, F&& f)
	{
		for (auto it = mesh.edges_begin(); it != mesh.edges_end(); ++it)
		{
			f(static_cast<EdgeHandle>(*it));
		}
	}

	static std::size_t GetNumElements(const PolyMeshKernel& mesh)
	{
		return mesh.n_edges();
	}
};

template <>
struct SelectionHelpers<FaceHandle>
{
	template <typename F>
	static void Iterate(PolyMeshKernel& mesh, F&& f)
	{
		for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it)
		{
			f(static_cast<FaceHandle>(*it));
		}
	}

	static std::size_t GetNumElements(const PolyMeshKernel& mesh)
	{
		return mesh.n_faces();
	}
};

template <typename HandleType>
class Selection
{
public:
	Selection() = delete;

	/**
	 * Make vertex selection.
	 * @param handles Vertex handles to mark as selected.
	 */
	void Select(const std::span<HandleType> handles)
	{
		for (const auto vertexHandle : handles)
		{
			_mesh.get().status(vertexHandle).set_selected(true);
		}

		_dirtyFlags.get() |= MeshProxyDirtyFlags::Color;
	}

	/**
	 * Deselect vertices.
	 * @param handles Vertex handles to mark as deselected.
	 */
	void Deselect(const std::span<HandleType> handles)
	{
		for (const auto vertexHandle : handles)
		{
			_mesh.get().status(vertexHandle).set_selected(false);
		}

		_dirtyFlags.get() |= MeshProxyDirtyFlags::Color;
	}

	/**
	 * Deselect all vertices.
	 */
	void DeselectAll()
	{
		SelectionHelpers<HandleType>::Iterate(_mesh.get(),
			[this](const HandleType handle) { _mesh.get().status(handle).set_selected(false); });

		_dirtyFlags.get() |= MeshProxyDirtyFlags::Color;
	}

	/**
	 * Make vertex highlighted.
	 * @param handle Vertex handle to mark as highlighted.
	 */
	void Highlight(const HandleType handle)
	{
		_mesh.get().status(handle).set_tagged(true);
		_dirtyFlags.get() |= MeshProxyDirtyFlags::Color;
	}

	/**
	 * Make vertex unhighlighted.
	 * @param handle Vertex handle to mark as unhighlighted.
	 */
	void Unhighlight(const HandleType handle)
	{
		_mesh.get().status(handle).set_tagged(false);
		_dirtyFlags.get() |= MeshProxyDirtyFlags::Color;
	}

	/**
	 * Unhighlights all elements of the selection.
	 */
	void UnhighlightAll()
	{
		SelectionHelpers<HandleType>::Iterate(_mesh.get(),
			[this](const HandleType handle) { _mesh.get().status(handle).set_tagged(false); });
		_dirtyFlags.get() |= MeshProxyDirtyFlags::Color;
	}

	/**
	 * Sets the positions of the currently selected vertices.
	 * @param positions The new positions.
	 */
	void SetPositions(const std::unordered_map<VertexHandle, glm::vec3>& positions)
	{
		// Update the points by the translation vector.
		for (const auto& pair : positions)
		{
			_mesh.get().point(pair.first) = _invModelMatrix * glm::vec4(positions.find(pair.first)->second, 1.0f);
		}

		UpdateNormals();
	}

	/**
	 * Returns the current selection of the handle type.
	 * @return All selected elements.
	 */
	[[nodiscard]] std::vector<HandleType> GetSelection() const
	{
		std::vector<HandleType> selection;
		selection.reserve(std::round(0.5f * static_cast<float>(SelectionHelpers<HandleType>::GetNumElements(_mesh.get()))));

		SelectionHelpers<HandleType>::Iterate(_mesh.get(),
			[this, &selection](const HandleType handle)
			{
				if (_mesh.get().status(handle).selected())
				{
					selection.push_back(handle);
				}
			});

		return selection;
	}

	/**
	 * Returns whether any element in the mesh is selected.
	 * @return Whether we have any selected elements.
	 */
	[[nodiscard]] bool HasSelection() const
	{
		bool hasSelection = false;

		SelectionHelpers<HandleType>::Iterate(_mesh.get(),
			[this, &hasSelection](const HandleType handle)
			{
				hasSelection |= _mesh.get().status(handle).selected();
			});

		return hasSelection;
	}

protected:
	void UpdateNormals()
	{
		_mesh.get().update_normals();

		_dirtyFlags.get() |= MeshProxyDirtyFlags::Position;
		_dirtyFlags.get() |= MeshProxyDirtyFlags::Normal;
	}

protected:
	friend class PolygonMesh;

	Selection(const glm::mat4& modelMatrix,
		PolyMeshKernel& mesh,
		const std::vector<glm::vec3>& cachedProjectedPoints,
		MeshProxyDirtyFlags& dirtyFlags)
		: _modelMatrix(modelMatrix)
		, _invModelMatrix(glm::inverse(modelMatrix))
		, _mesh(mesh)
		, _cachedProjectedPoints(cachedProjectedPoints)
		, _dirtyFlags(dirtyFlags)
	{ }

protected:
	glm::mat4 _modelMatrix;
	glm::mat4 _invModelMatrix;
	std::reference_wrapper<PolyMeshKernel> _mesh;
	std::vector<glm::vec3> _cachedProjectedPoints;
	std::reference_wrapper<MeshProxyDirtyFlags> _dirtyFlags;

};

class VertexSelection : public Selection<VertexHandle>
{
public:
	VertexSelection() = delete;

	/**
	 * Selects any points that are in the rectangle.
	 * @param viewInfo The current camera view info.
	 * @param rectMin Rectangle minimum bounds.
	 * @param rectMax Rectangle maximum bounds.
	 */
	void SelectInRectangle(const ViewInfo& viewInfo,
		const glm::vec2& rectMin,
		const glm::vec2& rectMax)
	{
		auto dirtyFlags = MeshProxyDirtyFlags::None;
		const auto actualRectMin = glm::min(rectMin, rectMax);
		const auto actualRectMax = glm::max(rectMin, rectMax);

		if (rectMin != rectMax)
		{
			DeselectAll();
		}

		for (std::size_t pointIndex = 0; pointIndex < _cachedProjectedPoints.size(); ++pointIndex)
		{
			const auto& point = _cachedProjectedPoints[pointIndex];
			const auto scaledPoint = glm::vec2(point / point.z);

			if (scaledPoint.x > actualRectMin.x && scaledPoint.y > actualRectMin.y &&
			    scaledPoint.x < actualRectMax.x && scaledPoint.y < actualRectMax.y)
			{
				_mesh.get().status(VertexHandle(static_cast<int>(pointIndex))).set_selected(true);

				dirtyFlags |= MeshProxyDirtyFlags::Color;
			}
		}

		_dirtyFlags.get() |= dirtyFlags;
	}

	/**
	 * Translates the currently selected vertices.
	 * @param translation The translation vector.
	 */
	void Translate(const glm::vec3& translation)
	{
		// Update the points by the translation vector.
		SelectionHelpers<VertexHandle>::Iterate(_mesh.get(),
			[this, &translation](const VertexHandle handle)
			{
				if (_mesh.get().status(handle).selected())
				{
					auto point = glm::vec3(_modelMatrix * glm::vec4(_mesh.get().point(handle), 1.0f));
					point += translation;

					_mesh.get().point(handle) = _invModelMatrix * glm::vec4(point, 1.0f);
				}
			});

		UpdateNormals();
	}

	/**
	 * Rotates the currently selected vertices.
	 * @param translation The translation vector.
	 */
	void Rotate(const glm::vec3& rotationOrigin, const glm::vec3& rotationAxis, const float rotationAngle)
	{
		// Create the rotation matrix.
		const auto rotMat = glm::rotate(glm::mat4(1.0f), glm::radians(rotationAngle), rotationAxis);

		// Update the points by the translation vector.
		SelectionHelpers<VertexHandle>::Iterate(_mesh.get(),
			[this, &rotMat, &rotationOrigin](const VertexHandle handle)
			{
				if (_mesh.get().status(handle).selected())
				{
					const auto point = rotMat * (_modelMatrix * glm::vec4(_mesh.get().point(handle), 1.0f) - glm::vec4(rotationOrigin, 1.0f));

					_mesh.get().point(handle) = _invModelMatrix * (point + glm::vec4(rotationOrigin, 1.0f));
				}
			});

		UpdateNormals();
	}

private:
	friend class PolygonMesh;

	using Selection::Selection;
};

class EdgeSelection : public Selection<EdgeHandle>
{
public:
	EdgeSelection() = delete;

	/**
	 * Selects any edges that are in the rectangle.
	 * @param viewInfo The current camera view info.
	 * @param rectMin Rectangle minimum bounds.
	 * @param rectMax Rectangle maximum bounds.
	 */
	void SelectInRectangle(const ViewInfo& viewInfo,
		const glm::vec2& rectMin,
		const glm::vec2& rectMax)
	{
		auto dirtyFlags = MeshProxyDirtyFlags::None;
		const auto actualRectMin = glm::min(rectMin, rectMax);
		const auto actualRectMax = glm::max(rectMin, rectMax);

		if (rectMin != rectMax)
		{
			DeselectAll();
		}

		for (auto it = _mesh.get().edges_begin(); it != _mesh.get().edges_end(); ++it)
		{
			const auto& pointA = _cachedProjectedPoints[it->v0().idx()];
			const auto scaledPointA = glm::vec2(pointA / pointA.z);

			const auto& pointB = _cachedProjectedPoints[it->v1().idx()];
			const auto scaledPointB = glm::vec2(pointB / pointB.z);

			if (GeometryUtils::LineSegmentAABBIntersection(rectMin, rectMax, scaledPointA, scaledPointB))
			{
				_mesh.get().status(*it).set_selected(true);

				dirtyFlags |= MeshProxyDirtyFlags::Color;
			}
		}

		_dirtyFlags.get() |= dirtyFlags;
	}

	/**
	 * Translates the currently selected edges.
	 * @param translation The translation vector.
	 */
	void Translate(const glm::vec3& translation)
	{
		const auto uniqueVertices = GetVertexSelection();

		for (const auto vertexHandle : uniqueVertices)
		{
			auto point = glm::vec3(_modelMatrix * glm::vec4(_mesh.get().point(vertexHandle), 1.0f));
			point += translation;

			_mesh.get().point(vertexHandle) = _invModelMatrix * glm::vec4(point, 1.0f);
		}

		UpdateNormals();
	}

	/**
	 * Returns the currently selected vertices.
	 * @return All selected vertices.
	 */
	[[nodiscard]] std::vector<VertexHandle> GetVertexSelection() const
	{
		const auto maxSize = static_cast<std::size_t>(glm::floor(0.5f * static_cast<float>(_mesh.get().n_vertices())));

		std::unordered_set<VertexHandle> uniqueVertices;
		uniqueVertices.reserve(maxSize);

		// Update the points by the translation vector.
		SelectionHelpers<EdgeHandle>::Iterate(_mesh.get(),
			[this, &uniqueVertices](const EdgeHandle handle)
			{
				if (_mesh.get().status(handle).selected())
				{
					const auto heh = _mesh.get().halfedge_handle(handle, 0);
					const auto v0 = _mesh.get().from_vertex_handle(heh);
					const auto v1 = _mesh.get().to_vertex_handle(heh);

					uniqueVertices.insert(v0);
					uniqueVertices.insert(v1);
				}
			});

		std::vector<VertexHandle> result(uniqueVertices.size());
		std::copy(uniqueVertices.begin(), uniqueVertices.end(), result.begin());

		return result;
	}

private:
	friend class PolygonMesh;

	using Selection::Selection;
};

class PolygonSelection : public Selection<FaceHandle>
{
public:
	PolygonSelection() = delete;

	/**
	 * Selects any polygons that are in the rectangle.
	 * @param viewInfo The current camera view info.
	 * @param rectMin Rectangle minimum bounds.
	 * @param rectMax Rectangle maximum bounds.
	 */
	void SelectInRectangle(const ViewInfo& viewInfo,
		const glm::vec2& rectMin,
		const glm::vec2& rectMax)
	{
		auto dirtyFlags = MeshProxyDirtyFlags::None;
		const auto actualRectMin = glm::min(rectMin, rectMax);
		const auto actualRectMax = glm::max(rectMin, rectMax);

		if (rectMin != rectMax)
		{
			DeselectAll();
		}

		const auto checkEdge = [this, &rectMin, &rectMax, &dirtyFlags](const VertexHandle vA, const VertexHandle vB)
		{
			const auto& pointA = _cachedProjectedPoints[vA.idx()];
			const auto scaledPointA = glm::vec2(pointA / pointA.z);

			const auto& pointB = _cachedProjectedPoints[vB.idx()];
			const auto scaledPointB = glm::vec2(pointB / pointB.z);

			return GeometryUtils::LineSegmentAABBIntersection(rectMin, rectMax, scaledPointA, scaledPointB);
		};

		for (auto it = _mesh.get().faces_begin(); it != _mesh.get().faces_end(); ++it)
		{
			// Store the first point of the face as the origin of each triangle.
			auto startVertexIt = _mesh.get().cfv_begin(static_cast<FaceHandle>(*it));
			auto endVertexIt = _mesh.get().cfv_end(static_cast<FaceHandle>(*it));
			auto nextVertexIt = startVertexIt;
			++nextVertexIt;

			if (checkEdge(*startVertexIt, *nextVertexIt))
			{
				_mesh.get().status(*it).set_selected(true);

				dirtyFlags |= MeshProxyDirtyFlags::Color;
			}
			else
			{
				while (nextVertexIt != endVertexIt)
				{
					const auto prevVertexIt = nextVertexIt;
					++nextVertexIt;

					if (checkEdge(*prevVertexIt, *nextVertexIt))
					{
						_mesh.get().status(*it).set_selected(true);

						dirtyFlags |= MeshProxyDirtyFlags::Color;

						break;
					}
				}
			}
		}

		_dirtyFlags.get() |= dirtyFlags;
	}

	/**
	 * Translates the currently selected faces.
	 * @param translation The translation vector.
	 */
	void Translate(const glm::vec3& translation)
	{
		const auto uniqueVertices = GetVertexSelection();

		for (const auto vertexHandle : uniqueVertices)
		{
			auto point = glm::vec3(_modelMatrix * glm::vec4(_mesh.get().point(vertexHandle), 1.0f));
			point += translation;

			_mesh.get().point(vertexHandle) = _invModelMatrix * glm::vec4(point, 1.0f);
		}

		UpdateNormals();
	}

	/**
	 * Returns the currently selected vertices.
	 * @return All selected vertices.
	 */
	[[nodiscard]] std::vector<VertexHandle> GetVertexSelection() const
	{
		const auto maxSize = static_cast<std::size_t>(glm::floor(0.5f * static_cast<float>(_mesh.get().n_vertices())));

		std::unordered_set<VertexHandle> uniqueVertices;
		uniqueVertices.reserve(maxSize);

		// Update the points by the translation vector.
		SelectionHelpers<FaceHandle>::Iterate(_mesh.get(),
			[this, &uniqueVertices](const FaceHandle handle)
			{
				if (_mesh.get().status(handle).selected())
				{
					auto vertexIt = _mesh.get().cfv_begin(handle);

					for (; vertexIt != _mesh.get().cfv_end(handle); ++vertexIt)
					{
						uniqueVertices.insert(*vertexIt);
					}
				}
			});

		std::vector<VertexHandle> result(uniqueVertices.size());
		std::copy(uniqueVertices.begin(), uniqueVertices.end(), result.begin());

		return result;
	}

private:
	friend class PolygonMesh;

	using Selection::Selection;
};
