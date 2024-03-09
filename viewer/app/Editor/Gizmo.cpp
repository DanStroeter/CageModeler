#include <Editor/Gizmo.h>
#include <Editor/Scene.h>
#include <Navigation/Camera.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/MeshLibrary.h>
#include <Input/InputSubsystem.h>

#include <glm/ext/matrix_transform.hpp>

namespace
{
	constexpr auto ScreenSpaceGizmoSize = glm::vec3(10.0f, 10.0f, 0.0f);
	constexpr auto GizmoSelectionMinDistanceThreshold = 15.0f;

	constexpr GizmoType GetGizmoType(const std::size_t index)
	{
		return static_cast<GizmoType>(index);
	}

	constexpr std::size_t GetGizmoIndex(const GizmoType gizmoType)
	{
		return static_cast<std::size_t>(gizmoType);
	}

	constexpr glm::vec3 GetGizmoColorForAxis(const GizmoAxis axis)
	{
		if (axis == GizmoAxis::X)
		{
			return glm::vec3(1.0f, 0.0f, 0.0f);
		}
		else if (axis == GizmoAxis::Y)
		{
			return glm::vec3(0.0f, 1.0f, 0.0f);
		}
		else if (axis == GizmoAxis::Z)
		{
			return glm::vec3(0.0f, 0.0f, 1.0f);
		}

		return glm::vec3(1.0f);
	}
}

Gizmo::Gizmo(Scene& scene)
{
	_meshes[GetGizmoIndex(GizmoType::Translate)][0] = scene.GetMesh(scene.AddGizmo(MeshLibrary::MakeTranslate(GizmoInteractionAxis::X)));
	_meshes[GetGizmoIndex(GizmoType::Translate)][1] = scene.GetMesh(scene.AddGizmo(MeshLibrary::MakeTranslate(GizmoInteractionAxis::Y)));
	_meshes[GetGizmoIndex(GizmoType::Translate)][2] = scene.GetMesh(scene.AddGizmo(MeshLibrary::MakeTranslate(GizmoInteractionAxis::Z)));

	_meshes[GetGizmoIndex(GizmoType::Rotate)][0] = scene.GetMesh(scene.AddGizmo(MeshLibrary::MakeRotation(GizmoInteractionAxis::X)));
	_meshes[GetGizmoIndex(GizmoType::Rotate)][1] = scene.GetMesh(scene.AddGizmo(MeshLibrary::MakeRotation(GizmoInteractionAxis::Y)));
	_meshes[GetGizmoIndex(GizmoType::Rotate)][2] = scene.GetMesh(scene.AddGizmo(MeshLibrary::MakeRotation(GizmoInteractionAxis::Z)));

	_meshes[GetGizmoIndex(GizmoType::Scale)][0] = scene.GetMesh(scene.AddGizmo(MeshLibrary::MakeScale(GizmoInteractionAxis::X)));
	_meshes[GetGizmoIndex(GizmoType::Scale)][1] = scene.GetMesh(scene.AddGizmo(MeshLibrary::MakeScale(GizmoInteractionAxis::Y)));
	_meshes[GetGizmoIndex(GizmoType::Scale)][2] = scene.GetMesh(scene.AddGizmo(MeshLibrary::MakeScale(GizmoInteractionAxis::Z)));

	for (auto& mesh : _meshes)
	{
		mesh[0]->SetVisible(false);
		mesh[1]->SetVisible(false);
		mesh[2]->SetVisible(false);
	}
}

void Gizmo::CollectRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector,
	const RenderResourceRef<Device>& device,
	const std::shared_ptr<RenderCommandScheduler>& renderCommandScheduler)
{
	for (auto& mesh : _meshes)
	{
		for (std::size_t i = 0; i < 3; ++i)
		{
			mesh[i]->CollectRenderProxy(renderProxyCollector, device, renderCommandScheduler);
		}
	}
}

void Gizmo::DestroyRenderProxy(const std::shared_ptr<RenderProxyCollector>& renderProxyCollector)
{
	for (auto& mesh : _meshes)
	{
		for (std::size_t i = 0; i < 3; ++i)
		{
			mesh[i]->DestroyRenderProxy(renderProxyCollector);
		}
	}
}

void Gizmo::SetPosition(const ViewInfo& viewInfo, const glm::vec3& position)
{
	_matrix[3].x = position.x;
	_matrix[3].y = position.y;
	_matrix[3].z = position.z;

	UpdateModelMatrix(viewInfo);
}

void Gizmo::UpdateModelMatrix(const ViewInfo& viewInfo, const glm::mat4& matrix)
{
	_matrix = matrix;

	UpdateModelMatrix(viewInfo);
}

void Gizmo::UpdateModelMatrix(const ViewInfo& viewInfo)
{
	_matrix = CalculateModelMatrixInternal(viewInfo);

	for (const auto& mesh : _meshes)
	{
		for (std::size_t i = 0; i < 3; ++i)
		{
			mesh[i]->SetModelMatrix(_matrix);
		}
	}
}

void Gizmo::SetVisible(const GizmoType gizmoType, const bool isVisible) const
{
	const auto& mesh = _meshes[GetGizmoIndex(gizmoType)];

	mesh[0]->SetVisible(isVisible);
	mesh[1]->SetVisible(isVisible);
	mesh[2]->SetVisible(isVisible);
}

void Gizmo::SetVisible(const bool isVisible) const
{
	for (std::size_t i = 0; i < static_cast<std::size_t>(GizmoType::MaxNum); ++i)
	{
		SetVisible(static_cast<GizmoType>(i), isVisible);
	}
}

void Gizmo::SetHighlighted(const GizmoType gizmoType, const GizmoAxis gizmoAxis, const bool isHighlighted) const
{
	const auto& mesh = _meshes[GetGizmoIndex(gizmoType)][static_cast<std::size_t>(gizmoAxis)];
	const auto color = isHighlighted ? glm::vec3(0.9f) : GetGizmoColorForAxis(gizmoAxis);

	mesh->SetColor(color);
}

std::optional<GizmoHitResult> Gizmo::QueryRayHit(const ViewInfo& viewInfo,
	const GizmoType gizmoType,
	const glm::vec2& screenPosition) const
{
	auto closestDist = std::numeric_limits<float>::max();
	auto closestAxis = GizmoAxis::X;

	const auto& mesh = _meshes[GetGizmoIndex(gizmoType)];

	for (std::size_t i = 0; i < 3; ++i)
	{
		// If we are doing vertex selection first cache the projected vertices into screen space.
		mesh[i]->MarkCachedGeometryDirty();
		mesh[i]->CacheProjectedPointsWorldToScreen(viewInfo);

		const auto result = mesh[i]->QueryClosestPolygonScreenSpace(viewInfo,
			screenPosition,
			GizmoSelectionMinDistanceThreshold);

		if (result.has_value())
		{
			closestDist = std::min(result->_distance, closestDist);
			closestAxis = static_cast<GizmoAxis>(i);
		}
	}

	if (closestDist != std::numeric_limits<float>::max())
	{
		return GizmoHitResult { closestAxis };
	}

	return { };
}

glm::mat4 Gizmo::CalculateModelMatrixInternal(const ViewInfo& viewInfo) const
{
	const auto position = GetPosition();
	auto pp = viewInfo.ProjectWorldToCamera(position);
	pp = viewInfo.ProjectCameraToScreen(pp);
	pp += ScreenSpaceGizmoSize;
	pp = viewInfo.DeprojectScreenToWorld(pp);

	const auto axisSize = 0.05f * glm::length(pp - position);

	return glm::scale(glm::translate(glm::mat4(1.0f), position), glm::vec3(axisSize));
}
