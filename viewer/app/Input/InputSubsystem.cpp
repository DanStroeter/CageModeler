#include <Input/InputSubsystem.h>
#include <UI/WindowSubsystem.h>
#include <Core/Types.h>

#include <SDL_mouse.h>
#include <algorithm>

void InputSubsystem::Initialize(const SubsystemsCollection& collection)
{
	float mouseX;
	float mouseY;

	_mouseStateMask = SDL_GetMouseState(&mouseX, &mouseY);
	_currentMousePosition = glm::vec2(mouseX, mouseY);
	_prevMousePosition = glm::vec2(mouseX, mouseY);
}

void InputSubsystem::Update(const double deltaTime)
{
	float mouseX;
	float mouseY;

	_mouseStateMask = SDL_GetMouseState(&mouseX, &mouseY);
	_prevMousePosition = _currentMousePosition;
	_currentMousePosition = glm::vec2(mouseX, mouseY);

	ProcessEvents();
}

void InputSubsystem::Deinitialize()
{

}

MouseButtonsState InputSubsystem::GetMouseButtonsState() const
{
	auto state = MouseButtonsState::None;

	if (SDL_BUTTON(_mouseStateMask) & SDL_BUTTON_LEFT)
	{
		state = state | MouseButtonsState::LeftPressed;
	}

	if (SDL_BUTTON(_mouseStateMask) & SDL_BUTTON_RIGHT)
	{
		state = state | MouseButtonsState::RightPressed;
	}

	if (SDL_BUTTON(_mouseStateMask) & SDL_BUTTON_MIDDLE)
	{
		state = state | MouseButtonsState::MiddlePressed;
	}

	return state;
}

void InputSubsystem::AddEventDelegate(const uint32_t eventType, EventDelegate delegate)
{
	const auto registeredDelegates = _registeredEventDelegates.find(eventType);

	if (registeredDelegates != _registeredEventDelegates.end())
	{
		registeredDelegates->second.push_back(std::move(delegate));
	}
	else
	{
		auto delegates = std::vector<EventDelegate>();
		delegates.push_back(std::move(delegate));

		_registeredEventDelegates.insert(std::make_pair(eventType, std::move(delegates)));
	}
}

void InputSubsystem::AddEventProcessedDelegate(EventDelegate delegate)
{
	_eventDelegate.push_back(std::move(delegate));
}

EventDelegateHandle InputSubsystem::RegisterInputActionMapping(InputActionMapping actionMapping)
{
#if BUILD_DEVELOPMENT
	// Do a linear search in the actions map for now.
	for (const auto& action : _actionsMap)
	{
		if (action._actionName == actionMapping._actionName)
		{
			CheckNoEntry("Action mapping already exists.");
		}
	}
#endif

	_actionsMap.push_back(std::move(actionMapping));

	return EventDelegateHandle();
}

EventDelegateHandle InputSubsystem::RegisterInputActionEntry(InputActionEntry actionEntry)
{
	const auto registeredDelegates = _actionsDelegateMap.find(actionEntry._actionName);

	if (registeredDelegates != _actionsDelegateMap.end())
	{
		registeredDelegates->second.push_back(std::move(actionEntry._delegate));
	}
	else
	{
		auto delegates = std::vector<InputEventDelegate>();
		delegates.push_back(std::move(actionEntry._delegate));

		_actionsDelegateMap.insert(std::make_pair(std::move(actionEntry._actionName), std::move(delegates)));
	}

	return EventDelegateHandle();
}

void InputSubsystem::ProcessEvents()
{
	SDL_Event event { };

	while (SDL_PollEvent(&event))
	{
		if (event.type == SDL_EVENT_KEY_DOWN)
		{
			_keysDown.insert(event.key.keysym.sym);
			_modifierKeys |= static_cast<SDL_Keymod>(event.key.keysym.mod);
		}
		else if (event.type == SDL_EVENT_KEY_UP)
		{
			_keysDown.erase(event.key.keysym.sym);
			_modifierKeys &= static_cast<SDL_Keymod>(event.key.keysym.mod);
		}

		// Copy the current set of actions that are down and remove from it as soon as we have another match during the current frame.
		// If we have any actions left in the set, we trigger them again, but with the key up type.
		auto keyPressedActions = _keyDownActions;
		auto keyUpActions = _keyDownActions;

		// Process all delegates in the actions map in case we matched the input keys with an action.
		for (const auto& actionMap : _actionsMap)
		{
			const auto containsAction = std::all_of(actionMap._inputKeys.begin(), actionMap._inputKeys.end(),
				[&keysDown = _keysDown](const auto index)
				{
					return std::find(keysDown.begin(), keysDown.end(), index) != keysDown.end();
				});

			if (containsAction)
			{
				float mouseX;
				float mouseY;
				const auto mouseStateMask = SDL_GetMouseState(&mouseX, &mouseY);
				const auto keyState = keyPressedActions.contains(actionMap._actionName) ? KeyState::KeyPressed : KeyState::KeyDown;

				// Just execute the delegates if we don't have any modifier keys requirements.
				if (actionMap._modifierKeys == 0 && actionMap._mouseInputMask == 0)
				{
					// Only if it's a first press we trigger a 'key down' type, otherwise it's going to be a 'key pressed'
					for (const auto& delegate : _actionsDelegateMap[actionMap._actionName])
					{
						delegate(InputActionParams { keyState, _modifierKeys });

						_keyDownActions.insert(actionMap._actionName);
						keyUpActions.erase(actionMap._actionName);
					}
				}
				else
				{
					// Check for the modifier keys and if they are active if the action requires it.
					if (IsSet(_modifierKeys, actionMap._modifierKeys) && static_cast<bool>(mouseStateMask & SDL_BUTTON(actionMap._mouseInputMask)))
					{
						for (const auto& delegate : _actionsDelegateMap[actionMap._actionName])
						{
							delegate(InputActionParams { keyState, _modifierKeys });

							_keyDownActions.insert(actionMap._actionName);
							keyUpActions.erase(actionMap._actionName);
						}
					}
				}
			}
		}

		// For every action left in the set we have to trigger the delegates again, but this time with key up type.
		for (const auto& actionName : keyUpActions)
		{
			for (const auto& delegate : _actionsDelegateMap[actionName])
			{
				delegate(InputActionParams { KeyState::KeyUp, _modifierKeys });
			}

			_keyDownActions.erase(actionName);
		}

		// Process all delegates after we read the event.
		for (const auto& delegate : _eventDelegate)
		{
			delegate(event);
		}

		const auto registeredDelegates = _registeredEventDelegates.find(event.type);

		// Process all delegates that are registered for the specific event type.
		if (registeredDelegates != _registeredEventDelegates.end())
		{
			for (const auto& delegate : registeredDelegates->second)
			{
				// Call the registered delegate with the event.
				delegate(event);
			}
		}
	}
}
