#include <Editor/Scene.h>
#include <Rendering/Scene/SceneRenderer.h>
#include <Mesh/GeometryUtils.h>
#include <Mesh/PolygonMesh.h>
#include <Navigation/CameraSubsystem.h>

namespace
{
	constexpr auto MaxNumLights = 4;
	constexpr auto MaxNumSceneMeshes = 32;
}

Scene::Scene(const std::shared_ptr<SceneRenderer>& renderer)
	: _renderer(renderer)
	, _polyMeshes(MaxNumSceneMeshes)
	, _allocatedPolyMeshes(MaxNumSceneMeshes)
{
	_lights.reserve(MaxNumLights);

	std::fill(_allocatedPolyMeshes.begin(), _allocatedPolyMeshes.end(), false);

	for (std::size_t i = 0; i < _allocatedPolyMeshes.size(); ++i)
	{
		if (_allocatedPolyMeshes[i])
		{

		}
	}

	_bvh = std::make_unique<BVH>();
}

void Scene::RemoveMesh(const MeshHandle handle)
{
	CheckFormat(_polyMeshes[handle] != nullptr, "The mesh with handle {} has not been allocated.", handle);

	_renderer->RemoveMesh(_polyMeshes[handle]);
	_polyMeshes[handle] = nullptr;
	_allocatedPolyMeshes[handle] = false;
}

void Scene::UpdateGizmoModelMatrix(const MeshHandle handle, const glm::mat4& modelMatrix)
{
	_polyMeshes[handle]->SetModelMatrix(modelMatrix);
}

void Scene::AddLightSource(const PointLight& pointLight)
{
	_lights.push_back(pointLight);
	_renderer->AddLightSource(PointLightGPU(pointLight._position, pointLight._intensity));
}

std::optional<MeshHitResult> Scene::QueryClosestMesh(const Ray& ray) const
{
	const auto t = _bvh->IntersectBVH(ray);

	if (t.has_value())
	{
		const auto worldPosition = ray._origin + t.value() * ray._direction;

		return MeshHitResult { 0, 0, 0, 0, worldPosition, glm::vec3(0.0f) };
	}

	return { };
}

void Scene::RebuildBVH() const
{
	_bvh->Build();
}

void Scene::UpdateDirtyRenderProxies() const
{
	for (std::size_t i = 0; i < _allocatedPolyMeshes.size(); ++i)
	{
		if (_allocatedPolyMeshes[i] && _polyMeshes[i]->IsDirty())
		{
			_polyMeshes[i]->UpdateRenderProxy();
		}
	}
}

MeshHandle Scene::AddMesh(const std::shared_ptr<PolygonMesh>& mesh)
{
	// Find the next free poly mesh index and mark it as set.
	const auto nextFreeIndex = std::ranges::find(_allocatedPolyMeshes, false) - _allocatedPolyMeshes.begin();
	_allocatedPolyMeshes[nextFreeIndex] = true;
	_polyMeshes[nextFreeIndex] = mesh;

	return nextFreeIndex;
}
