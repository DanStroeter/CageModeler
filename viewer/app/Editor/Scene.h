#pragma once

#include <Editor/Gizmo.h>
#include <Editor/Light.h>
#include <Rendering/Core/RenderProxy.h>
#include <Rendering/Scene/SceneRenderer.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/BVH.h>
#include <Core/Subsystem.h>

#include <cagedeformations/Parametrization.h>

class CameraSubsystem;
class InputSubsystem;
struct Ray;
class PolygonMesh;

struct MeshHitResult
{
	std::size_t _vertexIndex;
	std::size_t _edgeIndex;
	std::size_t _polyIndex;
	MeshHandle _meshHandle;
	glm::vec3 _worldPosition;
	glm::vec3 _polyNormal;
};

class Scene;

/**
 * A scene representation in the editor, contains all data about the meshes, computed weights, lights, etc.
 */
class Scene
{
public:
	Scene() = default;
	explicit Scene(const std::shared_ptr<SceneRenderer>& renderer);

	[[nodiscard]] MeshHandle AddCage(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& indices)
	{
		return AddMesh(_renderer->AddCage(vertices, indices));
	}

	[[nodiscard]] MeshHandle AddMesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& indices)
	{
		return AddMesh(_renderer->AddMesh(vertices, indices));
	}

	[[nodiscard]] MeshHandle AddGizmo(const MeshGeometry& geom)
	{
		return AddMesh(_renderer->AddGizmo(geom));
	}

	[[nodiscard]] std::shared_ptr<PolygonMesh> GetMesh(const MeshHandle handle) const
	{
		return _polyMeshes[handle];
	}

	void RemoveMesh(const MeshHandle handle);

	void UpdateGizmoModelMatrix(const MeshHandle handle, const glm::mat4& modelMatrix);

	void AddLightSource(const PointLight& pointLight);

	/**
	 * Returns information about all scene lights.
	 * @return All scene lights.
	 */
	[[nodiscard]] const std::vector<PointLight>& GetLights() const
	{
		return _lights;
	}

	/**
	 * Go through the BVH and check for the closest mesh.
	 * @param ray A ray.
	 * @return An optional mesh hit result containing mesh data of the closest hit.
	 */
	[[nodiscard]] std::optional<MeshHitResult> QueryClosestMesh(const Ray& ray) const;

	/**
	 * Begins a new scope to build a BVH scene.
	 * @return A new builder scope.
	 */
	[[nodiscard]] BVHGeometryBuilder BeginGeometryBVH() const
	{
		return BVHGeometryBuilder(*this, *_bvh);
	}

	/**
	 * Rebuilds the entire BVH structure from scratch. Updating might be quicker, but in the current scale it should
	 * not be that slow to rebuild it when our meshes change.
	 */
	void RebuildBVH() const;

	/**
	 * Requests an update of all dirty meshes proxies.
	 */
	void UpdateDirtyRenderProxies() const;

private:
	/**
	 * Adds a new mesh to the scene and returns the handle to it. The function will assert if we have ran out of free slots.
	 * @param mesh The mesh to add to the scene.
	 * @return The handle to the new mesh.
	 */
	MeshHandle AddMesh(const std::shared_ptr<PolygonMesh>& mesh);

private:
	friend BVHGeometryBuilder;

	/// The renderer of the scene.
	std::shared_ptr<SceneRenderer> _renderer = nullptr;

	/// All structured meshes in the scene, that is the ones that are represented by a half-edge structure.
	std::vector<std::shared_ptr<PolygonMesh>> _polyMeshes;

	/// All allocated render proxies to mimic a sparse array.
	std::vector<bool> _allocatedPolyMeshes;

	/// A pointer to the camera system to get view information.
	SubsystemPtr<CameraSubsystem> _cameraSubsystem = nullptr;

	/// All lights in the scene.
	std::vector<PointLight> _lights;

	/// Pointer to the current BVH used for ray-casting in the scene.
	std::unique_ptr<BVH> _bvh;
};
