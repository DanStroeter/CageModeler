#pragma once

#include <Core/Subsystem.h>

class LoggingSubsystem final : public Subsystem
{
	DECLARE_SUBSYSTEM(LoggingSubsystem)

public:
	LoggingSubsystem();

	//~BEGIN Subsystem
	void Initialize(const SubsystemsCollection& collection) override { }
	void Update(const double deltaTime) override { }
	void Deinitialize() override;
	//~END Subsystem
};
