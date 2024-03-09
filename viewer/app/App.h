#pragma once

#include <memory>

#include <Core/Subsystem.h>

class Editor;
union SDL_Event;

class App
{
public:
	explicit App(SubsystemsCollection collection)
		: _collection(std::move(collection))
	{ }

	void Initialize();
	void Start();

private:
	void OnWindowResized(const SDL_Event& event);
	void OnQuit(const SDL_Event& event);

	void AddInputDelegates();

private:
	std::shared_ptr<Editor> _editor = nullptr;
	SubsystemsCollection _collection { SubsystemsCollection::Uninitialized };
	bool _isRunning = true;
};
