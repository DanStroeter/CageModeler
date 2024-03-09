#include <Input/InputSubsystem.h>
#include <Navigation/CameraSubsystem.h>
#include <UI/WindowSubsystem.h>

void CameraSubsystem::Initialize(const SubsystemsCollection& collection)
{
	_inputSubsystem = GetDependencySubsystem<InputSubsystem>(collection);
	_windowSubsystem = GetDependencySubsystem<WindowSubsystem>(collection);

	const auto drawableSize = _windowSubsystem->GetDrawableSize();
	const glm::vec2 currentRenderSize(static_cast<float>(drawableSize.width), static_cast<float>(drawableSize.height));

	const auto windowSize = _windowSubsystem->GetWindowSize();
	const glm::vec2 currentWindowSize(static_cast<float>(windowSize.width), static_cast<float>(windowSize.height));

	_camera = std::make_unique<Camera>(_inputSubsystem, currentRenderSize, currentWindowSize, glm::radians(50.0f));
}

void CameraSubsystem::Update(const double deltaTime)
{
	const auto drawableSize = _windowSubsystem->GetDrawableSize();
	const glm::vec2 currentRenderSize(static_cast<float>(drawableSize.width), static_cast<float>(drawableSize.height));

	const auto windowSize = _windowSubsystem->GetWindowSize();
	const glm::vec2 currentWindowSize(static_cast<float>(windowSize.width), static_cast<float>(windowSize.height));

	if (_renderSize != currentRenderSize || _windowSize != currentWindowSize)
	{
		_camera->SetScreenSize(currentRenderSize, currentWindowSize);
		_renderSize = currentRenderSize;
		_windowSize = currentWindowSize;
	}

	_camera->Update(deltaTime);
}

void CameraSubsystem::Deinitialize()
{

}

void CameraSubsystem::OnMouseClick(const SDL_Event& event)
{

}
