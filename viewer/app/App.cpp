#include <App.h>
#include <Input/InputSubsystem.h>
#include <Navigation/CameraSubsystem.h>
#include <Rendering/RenderSubsystem.h>
#include <UI/WindowSubsystem.h>

void App::Initialize()
{
	REGISTER_SUBSYSTEM(_collection, InputSubsystem)
	REGISTER_SUBSYSTEM(_collection, WindowSubsystem)
	REGISTER_SUBSYSTEM(_collection, CameraSubsystem)
	REGISTER_SUBSYSTEM(_collection, RenderSubsystem)

	_collection.PostInitializeSubsystems();
}

void App::Start()
{
	const auto inputSubsystem = _collection.GetDependencySubsystem<InputSubsystem>();
	const auto cameraSubsystem = _collection.GetDependencySubsystem<CameraSubsystem>();

	_editor = std::make_shared<Editor>(inputSubsystem, cameraSubsystem);

	const auto renderSubsystem = _collection.GetDependencySubsystem<RenderSubsystem>();

	// TODO: Separate the editor from the editor rendering to avoid post-initialization from the rendering subsystem.
	renderSubsystem->InitializeEditor(_editor);

	AddInputDelegates();

	auto t = 0.0;
	constexpr auto fixedDeltaTime = 1.0 / 120.0;

	auto currentTime = std::chrono::high_resolution_clock::now();
	auto accumulator = 0.0;

	while (_isRunning)
	{
		auto newTime = std::chrono::high_resolution_clock::now();
		const auto frameTime = newTime - currentTime;
		const auto deltaTime = (std::chrono::duration_cast<std::chrono::milliseconds>(frameTime).count() / 1000.0);

		currentTime = newTime;
		accumulator += deltaTime;

		while (accumulator >= fixedDeltaTime)
		{
			t += fixedDeltaTime;
			accumulator -= fixedDeltaTime;
		}

		// Update all registered subsystems.
		_collection.Update(deltaTime);

		// Update the editor before we do any rendering in case we want to react to the input or modify data.
		_editor->Update(deltaTime);

		renderSubsystem->Render(deltaTime);
	}
}

void App::OnWindowResized(const SDL_Event& event)
{
	if (event.window.type == SDL_EVENT_WINDOW_RESIZED)
	{
		const auto renderSubsystem = _collection.GetDependencySubsystem<RenderSubsystem>();

		renderSubsystem->OnWindowResized();
	}
}

void App::OnQuit(const SDL_Event& event)
{
	_isRunning = false;
}

void App::AddInputDelegates()
{
	const auto inputSubsystem = _collection.GetDependencySubsystem<InputSubsystem>();

	inputSubsystem->AddEventDelegate(SDL_EVENT_WINDOW_RESIZED, [this]<typename T>(T&& event)
		{
			OnWindowResized(std::forward<T>(event));
		});
	inputSubsystem->AddEventDelegate(SDL_EVENT_QUIT, [this]<typename T>(T&& event)
		{
			OnQuit(std::forward<T>(event));
		});
}
