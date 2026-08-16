#include <Logging/LoggingSubsystem.h>
#include <App.h>
#include <Editor/EvaluationOptions.h>

int main(int argc, char** argv)
{
	// Picks up "--eval-config <path>" so a generated evaluation config can be run without
	// copying it over the default one.
	EvaluationOptions::ParseCommandLine(argc, argv);

	std::cout << "Current working directory: " << std::filesystem::current_path() << "\n";
	// Initialize the storage for all subsystems.
	SubsystemsCollection subsystemsCollection;

	// Initialize logging in the beginning.
	REGISTER_SUBSYSTEM(subsystemsCollection, LoggingSubsystem)

	// Create the subsystems collection and initialize the app.
	const auto app = std::make_unique<App>(std::move(subsystemsCollection));
	app->Initialize();
	app->Start();

	return 0;
}