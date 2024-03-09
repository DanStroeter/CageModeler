#pragma once

#include <Core/Subsystem.h>
#include <Mesh/GeometryUtils.h>
#include <Rendering/Scene/SceneData.h>

#include <glm/vec3.hpp>

struct InputActionParams;
class InputSubsystem;

class Camera final
{
public:
	Camera(const SubsystemPtr<InputSubsystem>& eventSubsystem,
		const glm::vec2 renderSize,
		const glm::vec2 windowSize,
		const float fieldOfView);

	void Update(const double deltaTime);

	/// Updates the screen extents and the projection camera.
	void SetScreenSize(const glm::vec2 newRenderSize, const glm::vec2 newWindowSize);

	/// Updates the point of interest and then all of the relevant data to the view matrix.
	void SetPointOfInterest(const glm::vec3& pointOfInterest);

	[[nodiscard]] const ViewInfo& GetViewInfo() const
	{
		return _viewInfo;
	}

private:
	void OnCameraZoom(const InputActionParams& actionParams);
	void OnCameraPan(const InputActionParams& actionParams);
	void OnCameraRotate(const InputActionParams& actionParams);

private:
	SubsystemPtr<InputSubsystem> _inputSubsystem = nullptr;

	/// View matrix of the camera.
	ViewInfo _viewInfo { };

	/// Field of view of the camera.
	float _fieldOfView;

	/// The camera rotation is active.
	uint32_t _cameraRotationActive : 1;

	/// The camera zoom is active which will move it along the camera direction.
	uint32_t _cameraZoomActive : 1;

	/// The camera panning is active which will move it along the XY axes in camera space.
	uint32_t _cameraPanningActive : 1;

	/// Point of interest if it lies on an object, otherwise we use the "virtual" plane or if we are below it
	/// we will use a plane that is created at the horizontal middle of the screen.
	glm::vec3 _pointOfInterest { 0.0f };

	/// The length of the camera arm from the point of interest.
	float _cameraArmLength = 0.0f;
};
