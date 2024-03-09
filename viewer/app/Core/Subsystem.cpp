#include <Core/Subsystem.h>

SubsystemsCollection::PImpl::~PImpl()
{
	for (const auto& [name, subsystem] : _subsystems)
	{
		subsystem->Deinitialize();
	}
}

void SubsystemsCollection::PImpl::Update(const double deltaTime)
{
	for (const auto& [name, subsystem] : _subsystems)
	{
		subsystem->Update(deltaTime);
	}
}

void SubsystemsCollection::PImpl::PostInitializeSubsystems(const SubsystemsCollection& collection)
{
	for (const auto& [name, subsystem] : _subsystems)
	{
		subsystem->PostInitialize(collection);
	}
}
