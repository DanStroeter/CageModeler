#pragma once

#include <Mesh/PolygonMesh.h>
#include <Mesh/GeometryUtils.h>

enum class GizmoInteractionAxis : uint8_t
{
	X,
	Y,
	Z
};

struct MeshLibrary
{
	/**
	 * Creates a box mesh.
	 * @param minb Minimum bounding box.
	 * @param maxb Maximum bounding box.
	 */
	static MeshGeometry MakeBox(const glm::vec3& minb, const glm::vec3& maxb);

	/**
	 * Creates a cylinder
	 */
	static MeshGeometry MakeCylinder(const glm::vec3& axis,
		const glm::vec3& arm1,
		const glm::vec3& arm2,
		const uint32_t numSegments);

	/**
	 * Creates a lathed geometry.
	 */
	static MeshGeometry MakeLathe(const glm::vec3& axis,
								  const glm::vec3& arm1,
								  const glm::vec3& arm2,
								  const uint32_t numSegments,
								  const std::span<glm::vec2> points,
								  const glm::vec3& color,
								  const float eps = Epsilon);

	/**
	 * Creates a translation gizmo.
	 * @param axis Axis of the translation.
	 * @return Mesh geometry of the translation gizmo.
	 */
	static MeshGeometry MakeTranslate(const GizmoInteractionAxis axis);

	/**
	 * Creates a rotation gizmo.
	 * @param axis Axis of the rotation.
	 * @return Mesh geometry of the rotation gizmo.
	 */
	static MeshGeometry MakeRotation(const GizmoInteractionAxis axis);

	/**
	 * Creates a scale gizmo.
	 * @param axis Axis of the scale.
	 * @return Mesh geometry of the scale gizmo.
	 */
	static MeshGeometry MakeScale(const GizmoInteractionAxis axis);

	/**
	 * Loads the mesh at the given filepath, this function will return the mesh data using the Eigen data structures.
	 * @param filepath The filepath of the mesh.
	 * @param scale The scale of the mesh.
	 * @return An Eigen mesh representation.
	 */
	static std::optional<EigenMesh> LoadMesh(const std::filesystem::path& filepath, const float scale);

	static std::optional<EigenMesh> LoadCage(const std::filesystem::path& filepath);
};
