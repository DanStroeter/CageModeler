#include <Editor/EvaluationOptions.h>

#include <cstdlib>
#include <string_view>

namespace EvaluationOptions
{
	namespace
	{
		constexpr std::string_view kEvaluationConfigFlag = "--eval-config";
		constexpr auto kEvaluationConfigEnvironmentVariable = "CAGEMODELER_EVAL_CONFIG";

		std::string GEvaluationConfigPath;
	}

	void ParseCommandLine(const int argc, char** argv)
	{
		for (auto i = 1; i < argc; ++i)
		{
			const std::string_view argument(argv[i]);
			if (!argument.starts_with(kEvaluationConfigFlag))
			{
				continue;
			}

			const auto remainder = argument.substr(kEvaluationConfigFlag.size());
			if (remainder.empty())
			{
				// "--eval-config <path>", the path is the next argument.
				if (i + 1 < argc)
				{
					GEvaluationConfigPath = argv[++i];
				}
			}
			else if (remainder.front() == '=')
			{
				GEvaluationConfigPath = remainder.substr(1);
			}
		}
	}

	std::string GetEvaluationConfigPath()
	{
		if (!GEvaluationConfigPath.empty())
		{
			return GEvaluationConfigPath;
		}

		if (const auto* const fromEnvironment = std::getenv(kEvaluationConfigEnvironmentVariable))
		{
			return fromEnvironment;
		}

		return { };
	}
}
