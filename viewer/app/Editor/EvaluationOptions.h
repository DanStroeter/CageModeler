#pragma once

#include <string>

/**
 * Process wide options of the evaluation run, so a generated evaluation config can be run
 * without copying it over the default 'evaluation/projects.json'.
 */
namespace EvaluationOptions
{
	/**
	 * Reads the evaluation options out of the command line. Accepts both
	 * "--eval-config <path>" and "--eval-config=<path>".
	 */
	void ParseCommandLine(int argc, char** argv);

	/**
	 * @return The evaluation config to run, or an empty string when the default config
	 * should be used. The command line takes precedence over the CAGEMODELER_EVAL_CONFIG
	 * environment variable.
	 */
	[[nodiscard]] std::string GetEvaluationConfigPath();
}
