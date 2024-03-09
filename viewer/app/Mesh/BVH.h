#pragma once

#include <Mesh/GeometryUtils.h>

#include <span>
#include <glm/vec3.hpp>

class PolygonMesh;
class BVH;
class Scene;

class BVHGeometryBuilder
{
public:
	~BVHGeometryBuilder();

	/**
	 * Reserves some number of meshes to be stored in the builder.
	 * @param numMeshes A number of meshes.
	 */
	void ReserveNumMeshes(const std::size_t numMeshes);

	/**
	 * Adds a polygon object geometry to the BVH.
	 * @param handle A handle to the mesh.
	 */
	void AddGeometry(const MeshHandle handle);

private:
	friend Scene;

	BVHGeometryBuilder(const Scene& scene, BVH& bvh);

private:
	struct MeshTriangles
	{
		MeshTriangles(std::vector<glm::vec3> vertices,
			std::vector<uint32_t> indices)
			: _vertices(std::move(vertices))
			, _indices(std::move(indices))
		{ }

		std::vector<glm::vec3> _vertices;
		std::vector<uint32_t> _indices;
	};

	std::reference_wrapper<const Scene> _scene;
	std::reference_wrapper<BVH> _bvh;
	std::vector<MeshTriangles> _meshes;
	std::size_t _numTriangles = 0;
};

struct ALIGN_SIZE(32) BVHNode
{
	[[nodiscard]] bool isLeaf() const
	{
		return _triangleCount > 0;
	}

	AABB _aabb;
	int32_t _leftFirst = 0;
	int32_t _triangleCount = 0;
};

class BVH
{
public:
	/**
	 * Reserve enough space for the number of triangles to reduce build-up time.
	 * @param numTriangles Number of triangles.
	 */
	void Reserve(const std::size_t numTriangles);

	/**
	 * Adds geometry data to the BVH before we build it.
	 * @param vertices Vertices of the geometry.
	 * @param indices Indices of the geometry.
	 */
	void AddGeometry(const std::span<glm::vec3> vertices, const std::span<uint32_t> indices);

	/**
	 * Intersect the BVH with a ray.
	 * @param ray A ray.
	 * @return The minimum T of the ray.
	 */
	[[nodiscard]] std::optional<float> IntersectBVH(const Ray& ray) const;

	/**
	 * Finds the closest point to the ray.
	 * @param ray A ray.
	 * @return The closest point to the ray.
	 */
	[[nodiscard]] std::optional<float> QueryClosestPoint(const Ray& ray) const;

	/**
	 * Builds the BVH structure from the geometry that has been added to it.
	 */
	void Build();

	/**
	 * Removes all primitives from the BVH and prepares it for rebuild.
	 */
	void Clear();

private:
	/**
	 * Updates the bounds of a single node.
	 * @param nodeIdx The index of the node to be updated.
	 */
	void UpdateNodeBounds(const std::size_t nodeIdx);

	/**
	 * Subdivides the volume that is being spanned by the given node.
	 * \param nodeIdx The index of the node to be subdivided.
	 */
	void Subdivide(const std::size_t nodeIdx);

	/**
	 * Shoots a ray that intersects a given node.
	 * \param ray A ray.
	 * \param nodeIdx The node index to be intersected.
	 * \param minT An output minimum value T of the ray that has intersected the node.
	 */
	void IntersectBVH(const Ray& ray, const std::size_t nodeIdx, float& minT) const;

	/**
	 * Check if a bounding box has been intersected by a ray.
	 * \param ray A ray.
	 * \param minBoundingBox Minimum bounding box.
	 * \param maxBoundingBox Maximum bounding box.
	 * \param t The parameter T of the ray to be used.
	 * \return Whether or not we intersect the bounding box.
	 */
	[[nodiscard]] bool IntersectAABB(const Ray& ray, const glm::vec3& minBoundingBox, const glm::vec3& maxBoundingBox, const float t) const;

private:
	struct Triangle
	{
		Triangle(const glm::vec3& vA,
			const glm::vec3& vB,
			const glm::vec3& vC)
			: _vA(vA)
			, _vB(vB)
			, _vC(vC)
		{ }

		glm::vec3 _vA;
		glm::vec3 _vB;
		glm::vec3 _vC;
		glm::vec3 _centroid = glm::vec3(0.0f);
	};

	struct MeshInfo
	{
		/// The original mesh handle.
		MeshHandle _meshHandle;

		/// The original mesh polygon index.
		std::size_t _polyIndex;
	};

private:
	/// ALl allocated BVH nodes.
	std::vector<BVHNode> _nodes;

	/// The index of the root node in the hierarchy.
	int32_t _rootNodeIndex = 0;

	/// The number of used nodes.
	int32_t _nodesUsed = 1;

	/// All triangle primitives in the BVH.
	std::vector<Triangle> _triangles;

	/// Triangle indices
	std::vector<int32_t> _triangleIndices;

	/// A mapping from the original triangles into the index of the face of the original mesh.
	std::vector<MeshInfo> _trianglesMapping;
};