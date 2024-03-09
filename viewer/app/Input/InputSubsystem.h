#pragma once

#include <Core/Subsystem.h>

#include <SDL_events.h>
#include <SDL_keycode.h>
#include <unordered_set>
#include <bitset>
#include <glm/vec2.hpp>

class WindowSubsystem;
class UISubsystem;

enum class MouseButtonsState : uint8_t
{
	None = 0,
	LeftPressed = 1 << 0,
	RightPressed = 1 << 1,
	MiddlePressed = 1 << 2
};

enum class KeyState: uint8_t
{
	KeyDown,
	KeyPressed,
	KeyUp
};

struct InputActionParams
{
	KeyState _keyState { };
	SDL_Keymod _modifierKeys = SDL_KMOD_NONE;
};

/// Event delegate handle returned when we add a new receiver of an event. Has to be stored and freed when we are
/// no longer interested in receiving events for the given event type.
using EventDelegateHandle = std::size_t;

/// Event delegate type that is called after each event process in SDL.
using EventDelegate = std::function<void (SDL_Event)>;

/// Event delegate type that is called on each update if a key mapping was triggered.
using InputEventDelegate = std::function<void (InputActionParams)>;

/**
 * Input action mappings are uniquely registered with the input subsystem where the action name is used as a key.
 * You can also register multiple input keys for the same action name and both will be triggered in that case.
 */
struct InputActionMapping
{
	std::string _actionName;
	SDL_Keymod _modifierKeys;
	int32_t _mouseInputMask;
	std::vector<SDL_Keycode> _inputKeys;
};

/**
 * Input action entries are added to the list of delegates to be triggered when a specific action has been invoked.
 * They require that an existing mapping has already been added to the subsystem.
 */
struct InputActionEntry
{
	std::string _actionName;
	InputEventDelegate _delegate;
};

/**
 * Hashing function for sequences that require order-independent hashing.
 */
template <template <typename...> typename ElementHash = std::hash>
struct SymmetricRangeHash
{
	template <typename T>
	std::size_t operator()(T&& item) const
	{
		// New seed with the hash of 0.
		std::size_t r = ElementHash<int>{ }(0);

		for (auto&& x : item)
		{
			using ElementType = std::decay_t<decltype(x)>;

			const auto next = ElementHash<ElementType>{ }(x);
			r = r + next;
		}

		return r;
	}
};

/**
 * Equality operator for maps that require order-independent lookup of sequence types.
 */
struct UnorderedEqualComparator
{
	template <typename T>
	bool operator()(T&& lhs, T&& rhs) const
	{
		using KeyType = std::decay_t<decltype(std::begin(lhs))>;
		std::unordered_map<KeyType, std::size_t> counts;

		for (auto&& k : lhs)
		{
			++counts[k];
		}

		for (auto&& k : rhs)
		{
			--counts[k];
		}

		for (auto&& kv : counts)
		{
			if (kv.second != 0)
			{
				return false;
			}
		}

		return true;
	}
};

class InputSubsystem final : public Subsystem
{
	DECLARE_SUBSYSTEM(InputSubsystem)

public:
	//~BEGIN Subsystem
	void Initialize(const SubsystemsCollection& collection) override;
	void Update(const double deltaTime) override;
	void Deinitialize() override;
	//~END Subsystem

	[[nodiscard]] glm::vec2 GetPreviousMousePosition() const
	{
		return _prevMousePosition;
	}

	[[nodiscard]] glm::vec2 GetMousePosition() const
	{
		return _currentMousePosition;
	}

	[[nodiscard]] MouseButtonsState GetMouseButtonsState() const;

	/**
	 * @return The currently active key modifiers.
	 */
	[[nodiscard]] SDL_Keymod GetKeyModifiers() const
	{
		return _modifierKeys;
	}

	/**
	 * Adds a new delegate for the given event type.
	 */
	void AddEventDelegate(const uint32_t eventType, EventDelegate delegate);

	void AddEventProcessedDelegate(EventDelegate delegate);

	/**
	 * Registers the given action mapping with the subsystem. After this call we can add any delegate calls
	 * under the given action name.
	 */
	EventDelegateHandle RegisterInputActionMapping(InputActionMapping actionMapping);

	/**
	 * Registers a new input action entry which will make an action invoke all registered delegates.
	 */
	EventDelegateHandle RegisterInputActionEntry(InputActionEntry actionEntry);

private:
	void ProcessEvents();

private:
	struct ActionsMapValue
	{
		std::string _actionName;
		SDL_Keymod _modifierKeys;
	};

	glm::vec2 _currentMousePosition { };
	glm::vec2 _prevMousePosition { };
	uint32_t _mouseStateMask = 0;

	/// Currently pressed modifier keys.
	SDL_Keymod _modifierKeys;

	/// Registered key combinations map to specific actions, which are bound to vectors of delegate functions called in sequence.
	std::vector<InputActionMapping> _actionsMap;

	/// Registered delegates for the different actions.
	std::unordered_map<std::string, std::vector<InputEventDelegate>> _actionsDelegateMap;

	/// All actions that are currently active from having all keys down.
	std::unordered_set<std::string> _keyDownActions;

	/// Keeps track of all keys that are currently down, so we can send an event if we keep pressing.
	std::unordered_set<SDL_Keycode> _keysDown;

	/// Each event delegate is called during each update of the events processing.
	std::vector<EventDelegate> _eventDelegate;

	/// A very simple and inefficient way to handle input events and store callers.
	/// A map of event type to an array of bound function callbacks that would get called.
	std::unordered_map<uint32_t, std::vector<EventDelegate>> _registeredEventDelegates;
};
