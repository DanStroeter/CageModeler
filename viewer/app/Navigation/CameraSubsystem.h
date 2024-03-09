#pragma once

#include <Core/Subsystem.h>
#include <Navigation/Camera.h>

#include <glm/vec2.hpp>

class InputSubsystem;
class WindowSubsystem;
union SDL_Event;

class CameraSubsystem final : public Subsystem
{
	DECLARE_SUBSYSTEM(CameraSubsystem)

public:
	//~BEGIN Subsystem
	void Initialize(const SubsystemsCollection& collection) override;
	void Update(const double deltaTime) override;
	void Deinitialize() override;
	//~END Subsystem

	[[nodiscard]] Camera& GetCamera()
	{
		return *_camera;
	}

private:
	void OnMouseClick(const SDL_Event& event);

private:
	SubsystemPtr<WindowSubsystem> _windowSubsystem;
	SubsystemPtr<InputSubsystem> _inputSubsystem;

	std::unique_ptr<Camera> _camera;
	glm::vec2 _renderSize;
	glm::vec2 _windowSize;
};
