#pragma once

#include <memory>
#include <map>
#include <string>

#include <Logging/Logging.h>

class Subsystem;

template <typename T>
concept IsSubsystem = std::is_base_of_v<Subsystem, T>;

/**
 * A collection of subsystems that exposes only registering and getting dependencies.
 * We usually only need 1 for the whole app.
 */
class SubsystemsCollection final
{
public:
	static struct UninitializedTag { } Uninitialized;

	SubsystemsCollection()
		: _impl(std::make_shared<PImpl>())
	{ }

	explicit SubsystemsCollection(UninitializedTag)
	{ }

	template <typename T> requires IsSubsystem<T>
	std::shared_ptr<T> GetDependencySubsystem() const
	{
		return _impl->GetTypedSubsystem<T>();
	}

	void Update(const double deltaTime) const
	{
		_impl->Update(deltaTime);
	}

	void PostInitializeSubsystems() const
	{
		_impl->PostInitializeSubsystems(*this);
	}

private:
	class PImpl final
	{
	public:
		PImpl() = default;

		PImpl(const PImpl& other) = default;
		PImpl& operator=(const PImpl& other) = default;

		PImpl(PImpl&& other)  noexcept
			: _subsystems(std::move(other._subsystems)) { }
		PImpl& operator=(PImpl&& other) noexcept
		{
			_subsystems = std::move(other._subsystems);

			return *this;
		}

		~PImpl();

		template <typename T> requires IsSubsystem<T>
		std::shared_ptr<T> RegisterSubsystem();

		template <typename T> requires IsSubsystem<T>
		std::shared_ptr<T> GetTypedSubsystem()
		{
			if (!_subsystems.contains(T::SubsystemIdentifier))
			{
				RegisterSubsystem<T>();
			}

			return std::static_pointer_cast<T>(_subsystems[T::SubsystemIdentifier]);
		}

		void Update(const double deltaTime);

		void PostInitializeSubsystems(const SubsystemsCollection& collection);

	private:
		/// We use an ordered map here, because we might want a specific order of updates between the subsystems.
		std::map<std::string, std::shared_ptr<Subsystem>> _subsystems;
	};

private:
	template <typename T> requires IsSubsystem<T>
	void RegisterSubsystem()
	{
		std::shared_ptr<T> subsystem = _impl->RegisterSubsystem<T>();
		subsystem->Initialize(*this);
	}

private:
	friend struct SubsystemCollectionHelper;

	std::shared_ptr<PImpl> _impl = nullptr;
};

/**
 * The base class for all subsystems.
 */
class Subsystem : public std::enable_shared_from_this<Subsystem>
{
public:
	Subsystem() = default;
	virtual ~Subsystem() = default;

	Subsystem(const Subsystem& other) = delete;
	Subsystem& operator=(const Subsystem& other) = delete;

	Subsystem(Subsystem&& other) = default;
	Subsystem& operator=(Subsystem&& other) = default;

	/**
	 * Called when the subsystem is initially created.
	 * @param collection A reference to the collection that created the subsystem. It can be used to query some dependencies.
	 */
	virtual void Initialize(const SubsystemsCollection& collection) = 0;

	/**
	 * Called after the subsystem is initialized.
	 * @param collection A reference to the collection that created the subsystem. It can be used to query some dependencies.
	 */
	virtual void PostInitialize(const SubsystemsCollection& collection) { }

	/**
	 * Called on each frame update, could be used for example for rendering in a render subsystem.
	 * @param deltaTime Delta time between the last frames.
	 */
	virtual void Update(const double deltaTime) = 0;

	/**
	 * Called before the subsystem is destroyed.
	 */
	virtual void Deinitialize() = 0;

	/**
	 * Gets the subsystem of type T from the collection.
	 * @tparam T The subsystem class.
	 * @param collection A subsystem collection.
	 * @return A subsystem of type T from the collection.
	 */
	template <typename T> requires IsSubsystem<T>
	std::shared_ptr<T> GetDependencySubsystem(const SubsystemsCollection& collection) const
	{
		return collection.GetDependencySubsystem<T>();
	}
};

template<typename T> requires IsSubsystem<T>
std::shared_ptr<T> SubsystemsCollection::PImpl::RegisterSubsystem()
{
	if (_subsystems.contains(T::SubsystemIdentifier))
	{
		return GetTypedSubsystem<T>();
	}

	std::shared_ptr<Subsystem> subsystem = std::make_shared<T>();
	_subsystems.insert(std::make_pair(T::SubsystemIdentifier, std::move(subsystem)));

	return GetTypedSubsystem<T>();
}

/// Helper struct to register subsystems.
struct SubsystemCollectionHelper
{
	template <typename T> requires IsSubsystem<T>
	static void RegisterSubsystem(SubsystemsCollection& collection)
	{
		collection.RegisterSubsystem<T>();

		LOG_INFO("Initialized {}.", T::SubsystemIdentifier);
	}
};

template <typename T> using SubsystemPtr = std::shared_ptr<T>;

/// Use DECLARE_SUBSYSTEM inside the class declaration with the class name to have it define the subsystem identifier.
#define DECLARE_SUBSYSTEM(SubsystemType) \
	static inline constexpr auto SubsystemIdentifier { #SubsystemType }; \
	friend class SubsystemsCollection; \
	friend struct SubsystemCollectionHelper;

/// Used in conjunction with DECLARE_SUBSYSTEM to register it in a specific collection. Usually we have 1 collection per app.
#define REGISTER_SUBSYSTEM(Collection, SubsystemType) \
	SubsystemCollectionHelper::RegisterSubsystem<SubsystemType>(Collection);
