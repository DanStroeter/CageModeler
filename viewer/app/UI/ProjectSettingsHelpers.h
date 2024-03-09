#pragma once

#include <filesystem>
#include <array>

struct ProjecSettingsHelpers
{
	static constexpr auto MaxFilepathCharacters = 45_sz;
	static constexpr auto ButtonSize = ImVec2(65.0f, 25.0f);

	static constexpr std::array DeformationMethodNames {
#ifdef WITH_SOMIGLIANA
		"MVC",
#endif
		"QMVC",
		"Harmonic",
		"BBW",
		"LBC",
		"MEC",
		"MLC",
		"Green",
		"QGC",
#ifdef WITH_SOMIGLIANA
		"Somigliana"
#endif
	};

	static constexpr std::array LBCWeightingSchemeNames = {
		"Constant",
		"Linear",
		"Square",
		"Square Root"
	};

#ifdef WITH_SOMIGLIANA
	static constexpr std::array SomiglianaBulgingTypeNames = {
		"Solid Angle",
		"Swept Volume"
	};
#endif

	[[nodiscard]] static std::filesystem::path SanitizeFilepath(const std::filesystem::path& filepath)
	{
		const auto& filepathString = filepath.string();
		const auto filepathLength = filepathString.size();

		if (filepathLength > MaxFilepathCharacters)
		{
			const auto filepathSuffix = filepathString.substr(std::max(0_sz, filepathLength - MaxFilepathCharacters - 3),
				std::max(0_sz, std::max(MaxFilepathCharacters, filepathLength) - 3));
			const auto sanitizedPath = "..." + filepathSuffix;

			return sanitizedPath;
		}
		else
		{
			return filepath;
		}
	}

	static void PushFileSelectionUI_RightAligned(std::optional<std::filesystem::path>& inOutFilepath,
		const char* buttonLabel,
		const std::vector<nfdfilteritem_t>& filterList)
	{
		const auto buttonSize = ImGui::CalcItemSize(ButtonSize, 0.0f, 0.0f);

		UIHelpers::SetRightAligned(buttonSize.x);

		if (ImGui::Button(buttonLabel, ButtonSize))
		{
			const auto newFilepath = UIHelpers::PresentSelectFilePopup(std::filesystem::current_path() / "assets" / "meshes", filterList);

			if (newFilepath.has_value())
			{
				inOutFilepath = newFilepath.value();
			}
		}
	}

	static void PushFileSelectionUI(std::optional<std::filesystem::path>& inOutFilepath,
		const char* buttonLabel,
		const float horizontalOffset,
		const std::vector<nfdfilteritem_t>& filterList)
	{
		const auto buttonSize = ImGui::CalcItemSize(ButtonSize, 0.0f, 0.0f);

		ImGui::SetNextItemWidth(horizontalOffset - buttonSize.x - ImGui::GetStyle().WindowPadding.x);

		if (inOutFilepath.has_value())
		{
			const auto meshFilepath = SanitizeFilepath(inOutFilepath.value());
			ImGui::TextEx(meshFilepath.string().c_str());
		}
		else
		{
			ImGui::TextEx("");
		}

		ImGui::SameLine();

		ImGui::SetCursorPosX(horizontalOffset - buttonSize.x + ImGui::GetStyle().WindowPadding.x);

		if (ImGui::Button(buttonLabel, ButtonSize))
		{
			const auto newFilepath = UIHelpers::PresentSelectFilePopup(std::filesystem::current_path() / "assets" / "meshes", filterList);

			if (newFilepath.has_value())
			{
				inOutFilepath = newFilepath.value();
			}
		}
	}
};