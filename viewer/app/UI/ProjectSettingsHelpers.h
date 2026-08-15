#pragma once

#include <UI/ProjectModel.h>

#include <algorithm>
#include <filesystem>
#include <array>
#include <string>

struct ProjecSettingsHelpers
{
	static constexpr auto MaxFilepathCharacters = 45_sz;
	static constexpr auto ButtonSize = ImVec2(65.0f, 24.0f);
	static constexpr auto ClearButtonSize = ImVec2(24.0f, 24.0f);

	static constexpr std::array DeformationMethodNames {
		"MVC",
		"QMVC",
		"Harmonic",
		"BBW",
		"LBC",
		"MEC",
		"MLC",
		"Green",
		"QGC",
		"Somigliana",
		"PMVC",
		"PMVCO"
	};

	static constexpr std::array LBCWeightingSchemeNames = {
		"Constant",
		"Linear",
		"Square",
		"Square Root"
	};

	static constexpr std::array CageGenerationMethodNames = {
		"CoarseBounding"
	};

	static constexpr std::array PMVCDistanceTypeNames {
		"Euclidean",
		"Interior"
	};

	/**
	 * The PMVC settings of a project, shared by the new project, project settings and
	 * project options panels.
	 *
	 * Everything runs through the Ring pipeline with a fixed cubemap size and ring size,
	 * and the offset variant is selected through the PMVCO coordinate type, so the only
	 * settings left are the distance type, the hit count and the weights of the three-hit
	 * variant.
	 *
	 * @param model The project model to edit, expected to be up to date with
	 *              ApplyPMVCPreset().
	 * @param idSuffix Suffix that keeps the widget IDs of the panels apart.
	 */
	static void PushPMVCSettingsUI(ProjectModelData& model, const char* idSuffix)
	{
		const auto isPMVC = DeformationTypeHelpers::IsPMVC(model._deformationType);
		const auto isOffsetVariant = (model._deformationType == DeformationType::PMVCO);

		ImGui::BeginDisabled(!isPMVC);
		{
			ImGui::TableNextRow();
			{
				ImGui::TableSetColumnIndex(0);
				ImGui::TextEx("Distance");
				ImGui::SameLine();
				UIHelpers::HelpMarker("Euclidean weights PMVC with the rasterized depth of the hit, Interior with the heat-method interior distance, which respects the interior of the cage. The offset variant (PMVCO) weights by solid angle alone and has no distance term.");

				ImGui::TableSetColumnIndex(1);
				UIHelpers::SetRightAligned(125.0f);

				ImGui::BeginDisabled(isOffsetVariant);
				{
					const auto distanceLabel = std::string("##PMVCDistance") + idSuffix;
					const auto selectedDistance = (!isOffsetVariant && model._pmvcUseInteriorDistance) ? 1 : 0;

					if (ImGui::BeginCombo(distanceLabel.c_str(), PMVCDistanceTypeNames[selectedDistance], ImGuiComboFlags_HeightRegular))
					{
						for (auto i = 0; i < PMVCDistanceTypeNames.size(); i++)
						{
							const auto isSelected = (selectedDistance == i);

							if (ImGui::Selectable(PMVCDistanceTypeNames[i], isSelected))
							{
								model._pmvcUseInteriorDistance = (i == 1);
							}

							if (isSelected)
							{
								ImGui::SetItemDefaultFocus();
							}
						}

						ImGui::EndCombo();
					}
				}
				ImGui::EndDisabled();
			}

			ImGui::TableNextRow();
			{
				ImGui::TableSetColumnIndex(0);
				ImGui::TextEx("Hit Count");
				ImGui::SameLine();
				UIHelpers::HelpMarker("Number of hits (depth peeling layers) sampled per mesh vertex. Every second hit carries the negative contributions and is always omitted, except for a hit count of three, where the first, second and third hit are weighted by alpha, beta and theta instead.");

				ImGui::TableSetColumnIndex(1);
				UIHelpers::SetRightAligned(100.0f);

				// The offset variant has no distance term to peel against and always runs
				// with a single hit.
				ImGui::BeginDisabled(isOffsetVariant);
				{
					const auto hitCountLabel = std::string("##PMVCHitCount") + idSuffix;

					if (ImGui::InputScalar(hitCountLabel.c_str(), ImGuiDataType_U64, &model._pmvcHitCount))
					{
						model._pmvcHitCount = std::max<uint64_t>(1, model._pmvcHitCount);
					}
				}
				ImGui::EndDisabled();
			}

			// The three-hit weights only exist for a hit count of exactly three.
			ImGui::BeginDisabled(!model.UsesThreeHitWeights());
			{
				ImGui::TableNextRow();
				{
					ImGui::TableSetColumnIndex(0);
					ImGui::TextEx("Alpha (hit 1)");
					ImGui::SameLine();
					UIHelpers::HelpMarker("Three-hit PMVC variant: weight of the first hit contributions.");

					ImGui::TableSetColumnIndex(1);
					UIHelpers::SetRightAligned(100.0f);

					const auto alphaLabel = std::string("##PMVCAlpha") + idSuffix;
					ImGui::InputFloat(alphaLabel.c_str(), &model._pmvcAlpha, 0.0f, 0.0f, "%.3f");
				}

				ImGui::TableNextRow();
				{
					ImGui::TableSetColumnIndex(0);
					ImGui::TextEx("Beta (hit 2)");
					ImGui::SameLine();
					UIHelpers::HelpMarker("Three-hit PMVC variant: weight of the second hit contributions (subtracted by default).");

					ImGui::TableSetColumnIndex(1);
					UIHelpers::SetRightAligned(100.0f);

					const auto betaLabel = std::string("##PMVCBeta") + idSuffix;
					ImGui::InputFloat(betaLabel.c_str(), &model._pmvcBeta, 0.0f, 0.0f, "%.3f");
				}

				ImGui::TableNextRow();
				{
					ImGui::TableSetColumnIndex(0);
					ImGui::TextEx("Theta (hit 3)");
					ImGui::SameLine();
					UIHelpers::HelpMarker("Three-hit PMVC variant: weight of the third hit contributions.");

					ImGui::TableSetColumnIndex(1);
					UIHelpers::SetRightAligned(100.0f);

					const auto thetaLabel = std::string("##PMVCTheta") + idSuffix;
					ImGui::InputFloat(thetaLabel.c_str(), &model._pmvcTheta, 0.0f, 0.0f, "%.3f");
				}
			}
			ImGui::EndDisabled();
		}
		ImGui::EndDisabled();
	}

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
