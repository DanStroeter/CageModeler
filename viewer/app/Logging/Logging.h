#pragma once

#include <memory>

#ifndef SPDLOG_ACTIVE_LEVEL
	#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

struct LoggingHelper
{
public:
	[[nodiscard]] static inline auto GetLogger()
	{
		return _logger.get();
	}

private:
	friend class LoggingSubsystem;

	/// A shared pointer to a static instance of the logger.
	static std::shared_ptr<spdlog::logger> _logger;
};

#define LOG_INFO(...)     SPDLOG_LOGGER_INFO(LoggingHelper::GetLogger(), __VA_ARGS__)
#define LOG_TRACE(...)    SPDLOG_LOGGER_TRACE(LoggingHelper::GetLogger(), __VA_ARGS__)
#define LOG_WARN(...)     SPDLOG_LOGGER_WARN(LoggingHelper::GetLogger(), __VA_ARGS__)
#define LOG_DEBUG(...)    SPDLOG_LOGGER_DEBUG(LoggingHelper::GetLogger(), __VA_ARGS__)
#define LOG_ERROR(...)    SPDLOG_LOGGER_ERROR(LoggingHelper::GetLogger(), __VA_ARGS__)
#define LOG_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(LoggingHelper::GetLogger(), __VA_ARGS__)
