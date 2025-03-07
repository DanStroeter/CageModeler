#pragma once

#include <filesystem>
#include <array>

struct ProjecSettingsHelpers
{
	static constexpr auto MaxFilepathCharacters = 45_sz;
	static constexpr auto ButtonSize = ImVec2(65.0f, 24.0f);
	static constexpr auto ClearButtonSize = ImVec2(24.0f, 24.0f);

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
		const char* uniqueLabel,
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

		const auto oldCursorY = ImGui::GetCursorPosY();

		if (inOutFilepath.has_value())
		{
			ImGui::SetCursorPosY(oldCursorY - 4.0f);
			ImGui::SetCursorPosX(horizontalOffset - buttonSize.x - ClearButtonSize.x - 0.25f * ImGui::GetStyle().SeparatorTextPadding.x + ImGui::GetStyle().WindowPadding.x);
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));

			std::string clearButtonText(ICON_LC_CIRCLE_X);
			clearButtonText.append(uniqueLabel);

			if (ImGui::Button(clearButtonText.c_str(), ClearButtonSize))
			{
				inOutFilepath.reset();
			}
			ImGui::PopStyleColor();

			ImGui::SameLine();
		}

		ImGui::SetCursorPosY(oldCursorY - 2.0f);
		ImGui::SetCursorPosX(horizontalOffset - buttonSize.x + ImGui::GetStyle().WindowPadding.x);

		std::string selectButtonText("Select...");
		selectButtonText.append(uniqueLabel);

		if (ImGui::Button(selectButtonText.c_str(), ButtonSize))
		{
			const auto newFilepath = UIHelpers::PresentSelectFilePopup(std::filesystem::current_path() / "assets" / "meshes", filterList);

			if (newFilepath.has_value())
			{
				inOutFilepath = newFilepath.value();
			}
		}
	}
};