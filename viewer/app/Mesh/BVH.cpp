#include <Editor/Scene.h>
#include <Mesh/BVH.h>
#include <Mesh/PolygonMesh.h>

namespace
{
	glm::vec3 MemberwiseMin(const glm::vec3& vertexA, const glm::vec3& vertexB)
	{
		return glm::vec3(std::min(vertexA.x, vertexB.x), std::min(vertexA.y, vertexB.y), std::min(vertexA.z, vertexB.z));
	}

	glm::vec3 MemberwiseMax(const glm::vec3& vertexA, const glm::vec3& vertexB)
	{
		return glm::vec3(std::max(vertexA.x, vertexB.x), std::max(vertexA.y, vertexB.y), std::max(vertexA.z, vertexB.z));
	}
}

BVHGeometryBuilder::~BVHGeometryBuilder()
{
	_bvh.get().Reserve(_numTriangles);

	for (auto& meshTriangles : _meshes)
	{
		_bvh.get().AddGeometry(meshTriangles._vertices, meshTriangles._indices);
	}

	_bvh.get().Build();
}

void BVHGeometryBuilder::ReserveNumMeshes(const std::size_t numMeshes)
{
	_meshes.reserve(numMeshes);
}

void BVHGeometryBuilder::AddGeometry(const MeshHandle handle)
{
	const auto& mesh = _scene.get()._polyMeshes[handle];
	auto meshGeometry = _scene.get()._polyMeshes[handle]->GetGeometry();

	_meshes.emplace_back(std::move(meshGeometry._positions),
		std::move(meshGeometry._indices));

	_numTriangles += mesh->GetNumVertices();
}

BVHGeometryBuilder::BVHGeometryBuilder(const Scene& scene, BVH& bvh)
	: _scene(scene)
	, _bvh(bvh)
{
	bvh.Clear();
}

void BVH::Reserve(const std::size_t numTriangles)
{
	_triangles.reserve(numTriangles);
}

void BVH::AddGeometry(const std::span<glm::vec3> vertices, const std::span<uint32_t> indices)
{
	for (std::size_t i = 0; i < indices.size(); i += 3)
	{
		_triangles.emplace_back(vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]]);
	}
}

std::optional<float> BVH::IntersectBVH(const Ray& ray) const
{
	float minT = std::numeric_limits<float>::max();
	IntersectBVH(ray, _rootNodeIndex, minT);

	return (minT < std::numeric_limits<float>::max()) ? minT : std::optional<float>();
}

std::optional<float> BVH::QueryClosestPoint(const Ray& ray) const
{
	float closestT = std::numeric_limits<float>::max();

	for (const auto& triangle : _triangles)
	{
		const auto tA = GeometryUtils::ClosestPointToRay(ray, triangle._vA);
		const auto tB = GeometryUtils::ClosestPointToRay(ray, triangle._vB);
		const auto tC = GeometryUtils::ClosestPointToRay(ray, triangle._vC);

		closestT = std::min(closestT, std::min(std::min(tA, tB), tC));
	}

	return closestT;
}

void BVH::Build()
{
	_triangleIndices.resize(_triangles.size());
	_nodes.resize(_triangles.size() * 2);

	// Populate the triangle indices array first.
	for (std::size_t i = 0; i < _triangles.size(); ++i)
	{
		_triangleIndices[i] = i;
	}

	for (auto& _triangle : _triangles)
	{
		_triangle._centroid = (_triangle._vA + _triangle._vB + _triangle._vC) * 0.3333f;
	}

	BVHNode& root = _nodes[_rootNodeIndex];
	root._leftFirst = 0;
	root._triangleCount = _triangles.size();

	UpdateNodeBounds(_rootNodeIndex);
	Subdivide(_rootNodeIndex);
}

void BVH::Clear()
{
	_nodes.clear();
	_triangles.clear();
	_triangleIndices.clear();
	_trianglesMapping.clear();
	_nodesUsed = 1;
	_rootNodeIndex = 0;
}

void BVH::UpdateNodeBounds(const std::size_t nodeIdx)
{
	BVHNode& node = _nodes[nodeIdx];
	node._aabb._min = glm::vec3(std::numeric_limits<float>::max());
	node._aabb._max = glm::vec3(std::numeric_limits<float>::min());

	for (std::size_t first = node._leftFirst, i = 0; i < node._triangleCount; ++i)
	{
		const auto leafTriIdx = _triangleIndices[first + i];
		auto& leafTri = _triangles[leafTriIdx];
		node._aabb._min = MemberwiseMin(node._aabb._min, leafTri._vA);
		node._aabb._min = MemberwiseMin(node._aabb._min, leafTri._vB);
		node._aabb._min = MemberwiseMin(node._aabb._min, leafTri._vC);
		node._aabb._max = MemberwiseMax(node._aabb._max, leafTri._vA);
		node._aabb._max = MemberwiseMax(node._aabb._max, leafTri._vB);
		node._aabb._max = MemberwiseMax(node._aabb._max, leafTri._vC);
	}
}

void BVH::Subdivide(const std::size_t nodeIdx)
{
	BVHNode& node = _nodes[nodeIdx];
	if (node._triangleCount <= 2)
	{
		return;
	}

	// Determine split axis and position.
	const auto extent = node._aabb._max - node._aabb._min;

	int axis = 0;
	if (extent.y > extent.x)
	{
		axis = 1;
	}

	if (extent.z > extent[axis])
	{
		axis = 2;
	}

	const auto splitPos = node._aabb._min[axis] + extent[axis] * 0.5f;

	// Partitioning.
	auto i = node._leftFirst;
	auto j = i + node._triangleCount - 1;
	while (i <= j)
	{
		if (_triangles[_triangleIndices[i]]._centroid[axis] < splitPos)
		{
			i++;
		}
		else
		{
			std::swap(_triangleIndices[i], _triangleIndices[j--]);
		}
	}

	// Abort split if one of the sides is empty.
	const auto leftCount = i - node._leftFirst;
	if (leftCount == 0 || leftCount == node._triangleCount)
	{
		return;
	}

	// Create child nodes.
	const auto leftChildIdx = _nodesUsed++;
	const auto rightChildIdx = _nodesUsed++;
	_nodes[leftChildIdx]._leftFirst = node._leftFirst;
	_nodes[leftChildIdx]._triangleCount = leftCount;
	_nodes[rightChildIdx]._leftFirst = i;
	_nodes[rightChildIdx]._triangleCount = node._triangleCount - leftCount;
	node._leftFirst = leftChildIdx;
	node._triangleCount = 0;

	// Update the bounds of the children.
	UpdateNodeBounds(leftChildIdx);
	UpdateNodeBounds(rightChildIdx);

	// Subdivide recursively.
	Subdivide(leftChildIdx);
	Subdivide(rightChildIdx);
}

void BVH::IntersectBVH(const Ray& ray, const std::size_t nodeIdx, float& minT) const
{
	auto& node = _nodes[nodeIdx];
	if (!IntersectAABB(ray, node._aabb._min, node._aabb._max, minT))
	{
		return;
	}

	if (node.isLeaf())
	{
		for (std::size_t i = 0; i < node._triangleCount; ++i)
		{
			const auto& triangle = _triangles[_triangleIndices[node._leftFirst + i]];
			const auto resultT = GeometryUtils::IntersectRayTriangle(ray, triangle._vA, triangle._vB, triangle._vC);

			if (resultT.has_value())
			{
				minT = resultT.value();
			}
		}
	}
	else
	{
		IntersectBVH(ray, node._leftFirst, minT);
		IntersectBVH(ray, node._leftFirst + 1, minT);
	}
}

bool BVH::IntersectAABB(const Ray& ray, const glm::vec3& minBoundingBox, const glm::vec3& maxBoundingBox, const float t) const
{
	const auto t1 = (minBoundingBox - ray._origin) / ray._direction;
	const auto t2 = (maxBoundingBox - ray._origin) / ray._direction;

	auto minT = std::min(t1.x, t2.x);
	auto maxT = std::max(t1.x, t2.x);
	minT = std::min(minT, std::min(t1.y, t2.y));
	maxT = std::max(maxT, std::max(t1.y, t2.y));
	minT = std::min(minT, std::min(t1.z, t2.z));
	maxT = std::max(maxT, std::max(t1.z, t2.z));

	return maxT >= minT && minT < t && maxT > 0;
}
