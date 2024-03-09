#include <Logging/LoggingSubsystem.h>
#include <Logging/Logging.h>

std::shared_ptr<spdlog::logger> LoggingHelper::_logger = nullptr;

LoggingSubsystem::LoggingSubsystem()
	: Subsystem()
{
	if (LoggingHelper::_logger != nullptr)
	{
		return;
	}

	const auto sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

#if BUILD_RELEASE
	constexpr auto level = spdlog::level::info;
#else
	constexpr auto level = spdlog::level::debug;
#endif

	// Create the logger and configure it.
	LoggingHelper::_logger = std::make_shared<spdlog::logger>("MeshCageDeformation-spdLogger", sink);
	LoggingHelper::_logger->info("Started logging with {} level.", to_string_view(level));
	LoggingHelper::_logger->set_level(level);
	LoggingHelper::_logger->set_pattern("[%H:%M:%S %z] [thread %t] %v");
}

void LoggingSubsystem::Deinitialize()
{
	if (LoggingHelper::_logger != nullptr)
	{
		LoggingHelper::_logger->flush();
	}

	LoggingHelper::_logger.reset();
}
