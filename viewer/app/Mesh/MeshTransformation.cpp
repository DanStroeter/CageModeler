#include <Mesh/MeshTransformation.h>
#include <Rendering/Scene/SceneData.h>

MeshTransformation::MeshTransformation(const ViewInfo& viewInfo,
	const std::shared_ptr<PolygonMesh>& cage,
	const SelectionType selectionType,
	const TransformationType transformationType,
	const TransformationAxis transformationAxis,
	const glm::mat4& gizmoMatrix,
	const glm::vec2 currentMousePos,
	const glm::vec2 previousMousePos)
	: _cage(cage)
	, _selectionType(selectionType)
	, _transformationType(transformationType)
	, _transformationAxis(transformationAxis)
	, _matrix(gizmoMatrix)
	, _newMatrix(gizmoMatrix)
{
	const auto axis = glm::normalize(glm::vec3(_newMatrix[static_cast<glm::mat4::length_type>(_transformationAxis)]));

	const auto projectPoints = [this, &axis](const std::vector<VertexHandle>& selectedVertices)
		{
			// Project all points onto the normal plane.
			_initPoints.reserve(selectedVertices.size());
			_initProjectedPoints.reserve(selectedVertices.size());

			for (std::size_t i = 0; i < selectedVertices.size(); ++i)
			{
				_initPoints[i] = _cage->GetPosition(selectedVertices[i]);
				_initProjectedPoints[i] = GeometryUtils::ProjectPointOnPlane(_initPoints[i], glm::vec3(_matrix[3]), axis);
			}
		};

	if (selectionType == SelectionType::Vertex)
	{
		const auto selection = _cage->GetSelection<SelectionType::Vertex>();
		const auto selectedVertices = selection.GetSelection();

		projectPoints(selectedVertices);
	}
	else if (selectionType == SelectionType::Edge)
	{
		const auto selection = _cage->GetSelection<SelectionType::Edge>();
		const auto selectedVertices = selection.GetVertexSelection();

		projectPoints(selectedVertices);
	}

	const auto projMousePos = ProjectPointOntoRay(viewInfo, currentMousePos);
	_initMouseRayProjPoint = GeometryUtils::ProjectPointOnPlane(projMousePos, _matrix[3], axis);
	_initMouseRayDistSq = glm::length2(_initMouseRayProjPoint - projMousePos);
}

void MeshTransformation::Transform(const ViewInfo& viewInfo,
	const glm::vec2 currentMousePos,
	const glm::vec2 previousMousePos)
{
	const auto currentMouseRay = viewInfo.DeprojectScreenToWorldRay(currentMousePos);
	const auto previousMouseRay = viewInfo.DeprojectScreenToWorldRay(previousMousePos);
	const auto origin = glm::vec3(_matrix[3]);
	const auto axis = glm::normalize(glm::vec3(_newMatrix[static_cast<glm::mat4::length_type>(_transformationAxis)]));
	const auto projPosition = ProjectPointOntoRay(viewInfo, currentMousePos);

	if (_transformationType == TransformationType::Translate)
	{
		const auto projStartPosition = ProjectPointOntoRay(viewInfo, previousMousePos);
		const auto delta = projPosition - projStartPosition;
		const auto deltaProj = glm::dot(delta, axis) * axis;

		glm::vec3 translation(0.0f);
		if (_transformationAxis == TransformationAxis::X)
		{
			translation = glm::vec3(deltaProj.x, 0.0f, 0.0f);
		}
		else if (_transformationAxis == TransformationAxis::Y)
		{
			translation = glm::vec3(0.0f, deltaProj.y, 0.0f);
		}
		else if (_transformationAxis == TransformationAxis::Z)
		{
			translation = glm::vec3(0.0f, 0.0f, deltaProj.z);
		}

		if (_selectionType == SelectionType::Vertex)
		{
			auto selection = _cage->GetSelection<SelectionType::Vertex>();
			selection.Translate(translation);
		}
		else if (_selectionType == SelectionType::Edge)
		{
			auto selection = _cage->GetSelection<SelectionType::Edge>();
			selection.Translate(translation);
		}
		else if (_selectionType == SelectionType::Polygon)
		{
			auto selection = _cage->GetSelection<SelectionType::Polygon>();
			selection.Translate(translation);
		}

		_newMatrix = glm::translate(_newMatrix, translation);

		_cage->AddProxyDirtyFlag(MeshProxyDirtyFlags::Position);
	}
	else if (_transformationType == TransformationType::Rotate)
	{
		if (_selectionType == SelectionType::Vertex)
		{

		}

		// const auto axis = glm::normalize(glm::vec3(_matrix[static_cast<glm::mat4::length_type>(_transformationAxis)]));
		//
		// // Get the rotation axis from the gizmo matrix.
		// glm::vec3 rotationAxis;
		// if (_transformationAxis == TransformationAxis::X)
		// {
		// 	rotationAxis = _matrix[0];
		// }
		// else if (_transformationAxis == TransformationAxis::Y)
		// {
		// 	rotationAxis = _matrix[1];
		// }
		// else if (_transformationAxis == TransformationAxis::Z)
		// {
		// 	rotationAxis = _matrix[2];
		// }
		//
		// // Compute the intersection of the mouse cursor with the plane that is determined by the main axis.
		// const auto intersectionPt = GeometryUtils::RayPlaneIntersection(currentMouseRay, _matrix[3], rotationAxis);
		//
		// if (!intersectionPt.has_value())
		// {
		// 	return;
		// }

		// Get the angle between the start and current points on the plane and rotate the gizmo by that much.

	}
	else if (_transformationType == TransformationType::Scale)
	{
		// Calculate how much in scale the mouse has moved closer or farther from the plane.
		const auto projDelta = projPosition - _initMouseRayProjPoint;
		const auto projDeltaDistSq = glm::length2(projDelta);

		auto scale = sqrtf(projDeltaDistSq / _initMouseRayDistSq);

		if (glm::dot(projDelta, axis) < 0.0f)
		{
			scale *= -1.0f;
		}

		if (_selectionType == SelectionType::Vertex)
		{
			auto selection = _cage->GetSelection<SelectionType::Vertex>();
			const auto selectedVertices = selection.GetSelection();

			// Project all points onto the normal plane.
			std::unordered_map<VertexHandle, glm::vec3> newPositions;
			newPositions.reserve(_initProjectedPoints.size());

			// Compute the offset for each point from the current position to the projection.
			std::size_t index = 0;
			for (const auto vertexHandle : selectedVertices)
			{
				const auto delta = _initPoints[index] - _initProjectedPoints[index];
				const auto newPosition = _initProjectedPoints[index] + scale * delta;

				newPositions.insert(std::make_pair(vertexHandle, newPosition));

				++index;
			}

			selection.SetPositions(newPositions);

			_cage->AddProxyDirtyFlag(MeshProxyDirtyFlags::Position);
		}
		else if (_selectionType == SelectionType::Edge)
		{
			auto edgeSelection = _cage->GetSelection<SelectionType::Edge>();
			const auto selectedVertices = edgeSelection.GetVertexSelection();

			// Project all points onto the normal plane.
			std::unordered_map<VertexHandle, glm::vec3> newPositions;
			newPositions.reserve(_initProjectedPoints.size());

			// Compute the offset for each point from the current position to the projection.
			std::size_t index = 0;
			for (const auto vertexHandle : selectedVertices)
			{
				const auto delta = _initPoints[index] - _initProjectedPoints[index];
				const auto newPosition = _initProjectedPoints[index] + scale * delta;

				newPositions.insert(std::make_pair(vertexHandle, newPosition));

				++index;
			}

			auto vertexSelection = _cage->GetSelection<SelectionType::Vertex>();
			vertexSelection.SetPositions(newPositions);

			_cage->AddProxyDirtyFlag(MeshProxyDirtyFlags::Position);
		}
		else if (_selectionType == SelectionType::Polygon)
		{
			auto polySelection = _cage->GetSelection<SelectionType::Polygon>();
			const auto selectedVertices = polySelection.GetVertexSelection();

			// Project all points onto the normal plane.
			std::unordered_map<VertexHandle, glm::vec3> newPositions;
			newPositions.reserve(_initProjectedPoints.size());

			// Compute the offset for each point from the current position to the projection.
			std::size_t index = 0;
			for (const auto vertexHandle : selectedVertices)
			{
				const auto delta = _initPoints[index] - _initProjectedPoints[index];
				const auto newPosition = _initProjectedPoints[index] + scale * delta;

				newPositions.insert(std::make_pair(vertexHandle, newPosition));

				++index;
			}

			auto vertexSelection = _cage->GetSelection<SelectionType::Vertex>();
			vertexSelection.SetPositions(newPositions);

			_cage->AddProxyDirtyFlag(MeshProxyDirtyFlags::Position);
		}
	}
}

glm::vec3 MeshTransformation::ProjectPointOntoRay(const ViewInfo& viewInfo,
	const glm::vec2 mousePos)
{
	const auto mouseRay = viewInfo.DeprojectScreenToWorldRay(mousePos);
	const auto axis = glm::normalize(glm::vec3(_newMatrix[static_cast<glm::mat4::length_type>(_transformationAxis)]));
	const auto origin = glm::vec3(_newMatrix[3]);
	const Ray rayX { origin, axis };

	return GeometryUtils::ClosestPointRayToRay(rayX, mouseRay);
}
