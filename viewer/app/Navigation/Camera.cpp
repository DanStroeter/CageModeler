#include <Input/InputSubsystem.h>
#include <Core/Types.h>
#include <Navigation/Camera.h>

#include <glm/geometric.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/polar_coordinates.hpp>

namespace
{
	constexpr auto InitialDistanceFromPointOfInterest = 15.0f;
	constexpr auto DistanceFromEyeNoIntersection = 10.0f;
	constexpr auto InitialCameraPosition = glm::vec3(1.0f, 1.0f, 1.0f);
	constexpr auto NearPlaneDistance = 0.1f;
	constexpr auto FarPlaneDistance = 100.0f;
}

Camera::Camera(const SubsystemPtr<InputSubsystem>& eventSubsystem,
	const glm::vec2 renderSize,
	const glm::vec2 windowSize,
	const float fieldOfView)
	: _inputSubsystem(eventSubsystem)
	, _fieldOfView(fieldOfView)
	, _cameraRotationActive(false)
	, _cameraZoomActive(false)
	, _cameraPanningActive(false)
{
	_viewInfo._renderSize = renderSize;
	_viewInfo._windowSize = windowSize;

	const glm::vec2 sphericalCoordinates = glm::polar(InitialCameraPosition);
	const auto newEyePosition = InitialDistanceFromPointOfInterest * glm::euclidean(sphericalCoordinates);
	_cameraArmLength = InitialDistanceFromPointOfInterest;
	_viewInfo._view = glm::lookAt(newEyePosition, _pointOfInterest, glm::vec3(0.0f, 1.0f, 0.0f));
	_viewInfo._inverseView = glm::inverse(_viewInfo._view);

	const auto screenRatio = renderSize.x / renderSize.y;
	_viewInfo._projection = glm::perspective(fieldOfView, screenRatio, NearPlaneDistance, FarPlaneDistance);
	_viewInfo._projection[1][1] *= -1.0f;
	_viewInfo._position = newEyePosition;

	_viewInfo._inverseViewProjection = glm::inverse(_viewInfo._projection * _viewInfo._view);

	// Add camera zoom.
#if PLATFORM_MAC
	_inputSubsystem->RegisterInputActionMapping(InputActionMapping { "ZoomCamera", SDL_KMOD_LALT, SDL_BUTTON_LEFT, { SDLK_LALT }});
#else
	_inputSubsystem->RegisterInputActionMapping(InputActionMapping { "ZoomCamera", SDL_KMOD_NONE, SDL_BUTTON_RIGHT, { }});
#endif
	_inputSubsystem->RegisterInputActionEntry(InputActionEntry { "ZoomCamera", [this]<typename T>(T&& PH1) { OnCameraZoom(std::forward<T>(PH1)); } });

	// Add camera panning.
	_inputSubsystem->RegisterInputActionMapping(InputActionMapping { "PanCamera", SDL_KMOD_LSHIFT, SDL_BUTTON_LEFT, { SDLK_LSHIFT }});
	_inputSubsystem->RegisterInputActionEntry(InputActionEntry { "PanCamera", [this]<typename T>(T && PH1) { OnCameraPan(std::forward<T>(PH1)); } });

	// Add camera rotation.
#if PLATFORM_MAC
	_inputSubsystem->RegisterInputActionMapping(InputActionMapping { "RotateCamera", SDL_KMOD_LGUI, SDL_BUTTON_LEFT, { SDLK_LGUI }});
	_inputSubsystem->RegisterInputActionEntry(InputActionEntry { "RotateCamera", [this]<typename T>(T && PH1) { OnCameraRotate(std::forward<T>(PH1)); } });
#else
	_inputSubsystem->RegisterInputActionMapping(InputActionMapping { "RotateCamera", SDL_KMOD_NONE, SDL_BUTTON_LEFT, { SDLK_LALT }});
	_inputSubsystem->RegisterInputActionEntry(InputActionEntry { "RotateCamera", std::bind(&Camera::OnCameraRotate, this, std::placeholders::_1) });
#endif
}

void Camera::Update(const double deltaTime)
{
	if (_cameraZoomActive)
	{
		const auto mousePosition = _inputSubsystem->GetMousePosition();
		const auto prevMousePosition = _inputSubsystem->GetPreviousMousePosition();

		// Move the eye along the camera direction.
		const auto mouseDelta = mousePosition - prevMousePosition;
		const auto normalizedMouseDelta = mouseDelta / _viewInfo._renderSize;
		const auto sign = Sign((1.0f - mouseDelta.x) + mouseDelta.y);

		const auto invView = glm::inverse(_viewInfo._view);
		const auto eyePosition = glm::vec3(invView[3]);
		const auto eyeDir = glm::vec3(invView[2]);
		const auto newEyePosition = eyePosition + 5000.0f * sign * glm::length(normalizedMouseDelta) * static_cast<float>(deltaTime) * eyeDir;

		_cameraArmLength = glm::length(_pointOfInterest - newEyePosition);
		_viewInfo._view = glm::lookAt(newEyePosition, _pointOfInterest, glm::vec3(0.0f, 1.0f, 0.0f));
		_viewInfo._inverseView = glm::inverse(_viewInfo._view);
		_viewInfo._position = newEyePosition;
		_viewInfo._inverseViewProjection = glm::inverse(_viewInfo._projection * _viewInfo._view);
	}

	if (_cameraPanningActive)
	{
		const auto mousePosition = _inputSubsystem->GetMousePosition();
		const auto prevMousePosition = _inputSubsystem->GetPreviousMousePosition();

		// Deproject the points into world space.
		const auto prevMouseWorldPosition = _viewInfo.DeprojectScreenToWorld(prevMousePosition);
		const auto mouseWorldPosition = _viewInfo.DeprojectScreenToWorld(mousePosition);
		auto deltaWorldPosition = mouseWorldPosition - prevMouseWorldPosition;

		// Get the camera position from the view matrix.
		const auto invView = glm::inverse(_viewInfo._view);
		const auto eyePosition = glm::vec3(invView[3]);
		const auto poiToEye = glm::length(_pointOfInterest - eyePosition);
		const auto posToEye = glm::length(prevMouseWorldPosition - eyePosition);
		const auto ratio = poiToEye / posToEye;
		const auto cameraDeltaWorldPosition = 200.0f * static_cast<float>(deltaTime) * deltaWorldPosition * ratio;

		const auto newEyePosition = eyePosition - cameraDeltaWorldPosition;
		const auto newPointOfInterest = _pointOfInterest - cameraDeltaWorldPosition;

		// We need to recompute the point of interest if it has changed.
		// const auto intersectionPoint = RayPlaneIntersection(eyePosition, cameraDir, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

		// // If we are behind the plane, just use a position that is slightly in front of the camera.
		// if (!intersectionPoint.has_value())
		// {
		// 	_pointOfInterest = eyePosition + DistanceFromEyeNoIntersection * cameraDir;
		// }
		// else
		// {
		// 	_pointOfInterest = intersectionPoint.value();
		// }

		_pointOfInterest = newPointOfInterest;
		_viewInfo._view = glm::lookAt(newEyePosition, newPointOfInterest, glm::vec3(0.0f, 1.0f, 0.0f));
		_viewInfo._inverseView = glm::inverse(_viewInfo._view);
		_viewInfo._position = newEyePosition;
		_viewInfo._inverseViewProjection = glm::inverse(_viewInfo._projection * _viewInfo._view);
	}

	if (_cameraRotationActive)
	{
		const auto mousePosition = _inputSubsystem->GetMousePosition();
		const auto prevMousePosition = _inputSubsystem->GetPreviousMousePosition();

		const glm::vec2 deltaPosition = (mousePosition - prevMousePosition);

		// Ignore almost 0 movements.
		if (abs(deltaPosition.x) < Epsilon && abs(deltaPosition.y) < Epsilon)
		{
			return;
		}

		const auto invView = glm::inverse(_viewInfo._view);
		const auto eyePosition = glm::vec3(invView[3]);
		const auto armLength = glm::length(_pointOfInterest - eyePosition);
		glm::tvec2<float> sphericalCoordinates = glm::polar(eyePosition - _pointOfInterest);
		sphericalCoordinates.y -= deltaPosition.x * static_cast<float>(deltaTime);
		sphericalCoordinates.x = glm::clamp(sphericalCoordinates.x + deltaPosition.y * static_cast<float>(deltaTime), -1.5f, 1.5f);
		const auto newEyePosition = _pointOfInterest + armLength * glm::euclidean(sphericalCoordinates);

		_cameraArmLength = armLength;
		_viewInfo._view = glm::lookAt(newEyePosition, _pointOfInterest, glm::vec3(0.0f, 1.0f, 0.0f));
		_viewInfo._inverseView = glm::inverse(_viewInfo._view);
		_viewInfo._position = newEyePosition;
		_viewInfo._inverseViewProjection = glm::inverse(_viewInfo._projection * _viewInfo._view);
	}
}

void Camera::SetScreenSize(const glm::vec2 newRenderSize, const glm::vec2 newWindowSize)
{
	const auto screenRatio = newRenderSize.x / newRenderSize.y;
	_viewInfo._projection = glm::perspective(_fieldOfView, screenRatio, NearPlaneDistance, FarPlaneDistance);
	_viewInfo._projection[1][1] *= -1.0f;
	_viewInfo._inverseViewProjection = glm::inverse(_viewInfo._projection * _viewInfo._view);
	_viewInfo._renderSize = newRenderSize;
	_viewInfo._windowSize = newWindowSize;
}

void Camera::SetPointOfInterest(const glm::vec3& pointOfInterest)
{
	// Get the camera position from the view matrix.
	const auto invView = glm::inverse(_viewInfo._view);
	const auto eyePosition = glm::vec3(invView[3]);

	_cameraArmLength = glm::length(pointOfInterest - eyePosition);
	_pointOfInterest = pointOfInterest;
	_viewInfo._view = glm::lookAt(eyePosition, pointOfInterest, glm::vec3(0.0f, 1.0f, 0.0f));
	_viewInfo._inverseView = glm::inverse(_viewInfo._view);
	_viewInfo._inverseViewProjection = glm::inverse(_viewInfo._projection * _viewInfo._view);
}

void Camera::OnCameraZoom(const InputActionParams& actionParams)
{
	if (actionParams._keyState == KeyState::KeyDown)
	{
		_cameraZoomActive = true;
	}
	else if (actionParams._keyState == KeyState::KeyUp)
	{
		_cameraZoomActive = false;
	}
}

void Camera::OnCameraPan(const InputActionParams& actionParams)
{
	if (actionParams._keyState == KeyState::KeyDown)
	{
		_cameraPanningActive = true;
	}
	else if (actionParams._keyState == KeyState::KeyUp)
	{
		_cameraPanningActive = false;
	}
}

void Camera::OnCameraRotate(const InputActionParams& actionParams)
{
	if (actionParams._keyState == KeyState::KeyDown)
	{
		_cameraRotationActive = true;
	}
	else if (actionParams._keyState == KeyState::KeyUp)
	{
		_cameraRotationActive = false;
	}
}
