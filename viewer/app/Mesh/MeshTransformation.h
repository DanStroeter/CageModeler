#pragma once

#include <Mesh/Selection.h>
#include <Mesh/PolygonMesh.h>

enum class TransformationType : uint8_t
{
	Translate,
	Rotate,
	Scale
};

enum class TransformationAxis : uint8_t
{
	X,
	Y,
	Z
};

class MeshTransformation
{
public:
	MeshTransformation() = default;
	MeshTransformation(const ViewInfo& viewInfo,
		const std::shared_ptr<PolygonMesh>& cage,
		const SelectionType selectionType,
		const TransformationType transformationType,
		const TransformationAxis transformationAxis,
		const glm::mat4& matrix,
		const glm::vec2 currentMousePos,
		const glm::vec2 previousMousePos);

	/**
	 * Runs an update loop of the transformation which will deproject the mouse and transform the cage object.
	 * @param viewInfo The view info of the camera.
	 * @param currentMousePos The current mouse position in screen space.
	 * @param previousMousePos The previous mouse position in screen space.
	 */
	void Transform(const ViewInfo& viewInfo,
		const glm::vec2 currentMousePos,
		const glm::vec2 previousMousePos);

	/**
	 * Returns the transformation of the current selection.
	 * @return The transformation of the current selection.
	 */
	[[nodiscard]] glm::mat4 GetTransformation() const
	{
		return _newMatrix;
	}

private:
	[[nodiscard]] glm::vec3 ProjectPointOntoRay(const ViewInfo& viewInfo,
		const glm::vec2 mousePos);

private:
	std::shared_ptr<PolygonMesh> _cage;
	SelectionType _selectionType = SelectionType::Vertex;
	TransformationType _transformationType = TransformationType::Translate;
	TransformationAxis _transformationAxis = TransformationAxis::X;
	glm::mat4 _matrix { 1.0f };
	glm::mat4 _newMatrix { 1.0 };
	std::vector<glm::vec3> _initPoints;
	std::vector<glm::vec3> _initProjectedPoints;
	glm::vec3 _initMouseRayProjPoint { 0.0f };

	float _initMouseRayDistSq = 0.0f;
	float _viewDotRotationAxis = 0.0f;
	float _initMouseDist = 0.0f;

	bool _isRotationParallel = false;
};
