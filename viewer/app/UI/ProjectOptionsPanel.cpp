#include <UI/ProjectOptionsPanel.h>
#include <UI/ProjectSettingsHelpers.h>
#include <Mesh/Operations/MeshOperationSystem.h>

namespace
{
#if WITH_SOMIGLIANA
	constexpr auto PanelSize = ImVec2(420.0f, 630.0f);
#else
	constexpr auto PanelSize = ImVec2(420.0f, 410.0f);
#endif
}

ProjectOptionsPanel::ProjectOptionsPanel(const std::shared_ptr<ProjectModelData>& model,
	const std::shared_ptr<MeshOperationSystem>& meshOperationSystem,
	std::function<void (const bool)> influenceMapToggled,
	std::function<void ()> projectSettingsApplied)
	: _meshOperationSystem(meshOperationSystem)
	, _influenceMapToggled(std::move(influenceMapToggled))
	, _projectSettingsApplied(std::move(projectSettingsApplied))
{
	_model._projectData = model;
	_modifiedProjectModel = *model;

	UpdateAfterNewModel();
}

void ProjectOptionsPanel::Layout()
{
	const auto displaySize = ImGui::GetIO().DisplaySize;

	ImGui::SetNextWindowPos(ImVec2(displaySize.x - PanelSize.x - 25.0f, 45.0f));
	ImGui::SetNextWindowSize(PanelSize);

	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(15.0f, 15.0f));

	if (ImGui::Begin("MenuTabBar", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar))
	{
		constexpr auto descColumnWidth = 0.55f * PanelSize.x;
		constexpr auto settingsColumnWidth = 0.45f * PanelSize.x;

		const auto meshOperationSystem = _meshOperationSystem.lock();
		if (meshOperationSystem == nullptr)
		{
			return;
		}

		const auto hasRunningOperation = (meshOperationSystem->GetCurrentOperation() != nullptr);

		ImGui::BeginDisabled(hasRunningOperation);
		if (ImGui::BeginTable("##Project_SettingsTable", 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_SizingFixedSame))
		{
			constexpr auto columnFlags = ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize | ImGuiTableColumnFlags_NoReorder | ImGuiTableColumnFlags_NoHeaderLabel;

			ImGui::TableSetupColumn("##Project_Description", columnFlags, descColumnWidth);
			ImGui::TableSetupColumn("##Project_Settings", columnFlags, settingsColumnWidth);

			ImGui::TableNextRow();
			{
				ImGui::TableSetColumnIndex(0);
				ImGui::TextEx("Coordinates");
				ImGui::SameLine();

				ImGui::TableSetColumnIndex(1);

				UIHelpers::SetRightAligned(125.0f);

				if (ImGui::BeginCombo("##Deformation", ProjecSettingsHelpers::DeformationMethodNames[_selectedDeformationTypeIndex], ImGuiComboFlags_HeightRegular))
				{
					for (auto i = 0; i < ProjecSettingsHelpers::DeformationMethodNames.size(); i++)
					{
						const auto isSelected = (_selectedDeformationTypeIndex == i);

						if (ImGui::Selectable(ProjecSettingsHelpers::DeformationMethodNames[i], isSelected))
						{
							_selectedDeformationTypeIndex = i;
						}

						if (isSelected)
						{
							ImGui::SetItemDefaultFocus();
						}
					}

					_modifiedProjectModel._deformationType = static_cast<DeformationType>(_selectedDeformationTypeIndex);

					ImGui::EndCombo();
				}
			}

			ImGui::BeginDisabled(_modifiedProjectModel._deformationType != DeformationType::LBC);
			{
				ImGui::TableNextRow();
				{
					ImGui::TableSetColumnIndex(0);
					ImGui::TextEx("LBC Weighting Scheme");
					ImGui::SameLine();
					UIHelpers::HelpMarker("The weighting scheme for LBC.");
					ImGui::SameLine();

					ImGui::TableSetColumnIndex(1);

					UIHelpers::SetRightAligned(125.0f);

					if (ImGui::BeginCombo("##Project_WeightingScheme",
							ProjecSettingsHelpers::LBCWeightingSchemeNames[_selectedWeightingSchemeIndex],
							ImGuiComboFlags_HeightRegular))
					{
						for (auto i = 0; i < ProjecSettingsHelpers::LBCWeightingSchemeNames.size(); i++)
						{
							const auto isSelected = (_selectedWeightingSchemeIndex == i);

							if (ImGui::Selectable(ProjecSettingsHelpers::LBCWeightingSchemeNames[i], isSelected))
							{
								_selectedWeightingSchemeIndex = i;
							}

							if (isSelected)
							{
								ImGui::SetItemDefaultFocus();
							}
						}

						ImGui::EndCombo();
					}

					_modifiedProjectModel._LBCWeightingScheme = static_cast<LBC::DataSetup::WeightingScheme>(_selectedWeightingSchemeIndex);
				}
			}
			ImGui::EndDisabled();

			ImGui::Dummy(ImVec2(0.0f, 10.0f));

			ImGui::TableNextRow();
			{
				ImGui::TableSetColumnIndex(0);

				ImGui::PushFont(UIStyle::BoldFont);
				ImGui::SetWindowFontScale(1.05f);
				ImGui::TextEx("Project Settings");
				ImGui::SetWindowFontScale(1.0f);
				ImGui::PopFont();
			}

			ImGui::Dummy(ImVec2(0.0f, 5.0f));

			ImGui::TableNextRow();
			{
				ImGui::TableSetColumnIndex(0);
				ImGui::TextEx("Interpolate weights");
				ImGui::SameLine();
				UIHelpers::HelpMarker("Interpolate weights of model vertices from the embedding (embedding does not contain vertices of model).");

				ImGui::TableSetColumnIndex(1);
				UIHelpers::SetRightAligned(25.0f);
				ImGui::Checkbox("##Project_InterpolateWeights", &_modifiedProjectModel._interpolateWeights);
				ImGui::SameLine();
			}

			ImGui::TableNextRow();
			{
				ImGui::TableSetColumnIndex(0);
				ImGui::TextEx("Find offset");
				ImGui::SameLine();
				UIHelpers::HelpMarker("Search points of embedding for offset (requires equality of points).");

				ImGui::TableSetColumnIndex(1);
				UIHelpers::SetRightAligned(25.0f);
				ImGui::Checkbox("##Project_FindOffset", &_modifiedProjectModel._findOffset);
				ImGui::SameLine();
			}

			ImGui::TableNextRow();
			{
				ImGui::TableSetColumnIndex(0);
				ImGui::TextEx("No offset");
				ImGui::SameLine();
				UIHelpers::HelpMarker("Do not calculate an offset for model within the embedding (except the cage).");

				ImGui::TableSetColumnIndex(1);
				UIHelpers::SetRightAligned(25.0f);
				ImGui::Checkbox("##Project_NoOffset", &_modifiedProjectModel._noOffset);
				ImGui::SameLine();
			}

			ImGui::BeginDisabled(!_model._projectData->CanEditInfluenceMapSetting());
			{
				ImGui::TableNextRow();
				{
					ImGui::TableSetColumnIndex(0);
					ImGui::TextEx("Render Influence Map");
					ImGui::SameLine();
					UIHelpers::HelpMarker("Renders the mesh colors based on the computed influence map.");

					ImGui::TableSetColumnIndex(1);
					UIHelpers::SetRightAligned(25.0f);
					if (ImGui::Checkbox("##Project_InfluenceMap", &_model._projectData->_renderInfluenceMap))
					{
						_influenceMapToggled(_model._projectData->_renderInfluenceMap);
					}
					ImGui::SameLine();
				}
			}
			ImGui::EndDisabled();

			ImGui::BeginDisabled(_modifiedProjectModel._deformationType != DeformationType::BBW && _modifiedProjectModel._deformationType != DeformationType::LBC);
			{
				ImGui::TableNextRow();
				{
					ImGui::TableSetColumnIndex(0);

					ImGui::TextEx("BBW Iterations");
					ImGui::SameLine();
					UIHelpers::HelpMarker("The number of iterations for calculating BBW or LBC.");

					ImGui::TableSetColumnIndex(1);
					UIHelpers::SetRightAligned(100.0f);
					ImGui::InputInt("##Project_BBW_Iterations", &_modifiedProjectModel._numBBWSteps, 1, 10, ImGuiInputTextFlags_NoHorizontalScroll);
				}
			}
			ImGui::EndDisabled();
		}
		ImGui::EndDisabled();

#ifdef WITH_SOMIGLIANA
		ImGui::Dummy(ImVec2(0.0f, 8.0f));

		ImGui::TableNextRow();
		{
			ImGui::TableSetColumnIndex(0);

			ImGui::PushFont(UIStyle::BoldFont);
			ImGui::SetWindowFontScale(1.05f);
			ImGui::TextEx("Somigliana Settings");
			ImGui::SetWindowFontScale(1.0f);
			ImGui::PopFont();
		}

		ImGui::Dummy(ImVec2(0.0f, 5.0f));

		ImGui::BeginDisabled(_modifiedProjectModel._deformationType != DeformationType::Somigliana &&
			_modifiedProjectModel._deformationType != DeformationType::MVC);
		{
			ImGui::TableNextRow();
			{
				// Add the scale input at the bottom because it's valid for all meshes.
				ImGui::TableSetColumnIndex(0);
				ImGui::TextEx("Nu");
				ImGui::SameLine();
				UIHelpers::HelpMarker("The material parameter nu for somigliana deformer.");

				ImGui::TableSetColumnIndex(1);
				UIHelpers::SetRightAligned(100.0f);
				ImGui::InputDouble("##Project_SomigNu", &_modifiedProjectModel._somigNu);
			}

			ImGui::TableNextRow();
			{
				// Add the scale input at the bottom because it's valid for all meshes.
				ImGui::TableSetColumnIndex(0);
				ImGui::TextEx("Bulging");
				ImGui::SameLine();
				UIHelpers::HelpMarker("The bulging parameter gamma for somigliana deformer.");

				ImGui::TableSetColumnIndex(1);
				UIHelpers::SetRightAligned(100.0f);

				auto bulgingValue = _modifiedProjectModel._somigBulging.load();
				if (ImGui::InputDouble("##Project_SomigBulging", &bulgingValue))
				{
					_modifiedProjectModel._somigBulging = bulgingValue;
				}
			}

			ImGui::TableNextRow();
			{
				// Add the scale input at the bottom because it's valid for all meshes.
				ImGui::TableSetColumnIndex(0);
				ImGui::TextEx("Bulging factor");
				ImGui::SameLine();
				UIHelpers::HelpMarker("The blending factor for somigliana deformer interpolating between local and global boundary conditions.");

				ImGui::TableSetColumnIndex(1);
				UIHelpers::SetRightAligned(100.0f);

				auto blendFactorValue = _modifiedProjectModel._somigBlendFactor.load();
				if (ImGui::InputDouble("##Project_SomigBlendFactor", &blendFactorValue))
				{
					_modifiedProjectModel._somigBlendFactor = blendFactorValue;
				}
			}

			ImGui::TableNextRow();
			{
				ImGui::TableSetColumnIndex(0);
				ImGui::TextEx("Bulging Type");
				ImGui::SameLine();
				UIHelpers::HelpMarker("The bulging type for somigliana deformer.");
				ImGui::SameLine();

				ImGui::TableSetColumnIndex(1);

				UIHelpers::SetRightAligned(125.0f);

				if (ImGui::BeginCombo("##Project_BulgingType",
					ProjecSettingsHelpers::SomiglianaBulgingTypeNames[_selectedBulgingTypeIndex],
					ImGuiComboFlags_HeightRegular | ImGuiComboFlags_WidthFitPreview))
				{
					for (auto i = 0; i < ProjecSettingsHelpers::SomiglianaBulgingTypeNames.size(); i++)
					{
						const auto isSelected = (_selectedBulgingTypeIndex == i);

						if (ImGui::Selectable(ProjecSettingsHelpers::SomiglianaBulgingTypeNames[i], isSelected))
						{
							_selectedBulgingTypeIndex = i;
						}

						if (isSelected)
						{
							ImGui::SetItemDefaultFocus();
						}
					}

					ImGui::EndCombo();
				}

				_modifiedProjectModel._somigBulgingType = static_cast<BulgingType>(_selectedBulgingTypeIndex);
			}
		}
		ImGui::EndDisabled();
#endif

		ImGui::EndTable();

		ImGui::Dummy(ImVec2(0.0f, 15.0f));

		MeshElementsText meshElements { };

		if (_model._deformableMesh != nullptr)
		{
			meshElements = GetMeshElementsText("Mesh: ",
				_model._deformableMesh->GetNumVertices(),
				_model._deformableMesh->GetNumEdges(),
				_model._deformableMesh->GetNumFaces());
		}
		else
		{
			meshElements = GetMeshElementsText("Mesh: ", 0, 0, 0);
		}

		MeshElementsText cageElements { };

		if (_model._cageMesh != nullptr)
		{
			cageElements = GetMeshElementsText("Cage: ",
				_model._cageMesh->GetNumVertices(),
				_model._cageMesh->GetNumEdges(),
				_model._cageMesh->GetNumFaces());
		}
		else
		{
			cageElements = GetMeshElementsText("Cage: ", 0, 0, 0);
		}

		PushMeshElementsUI(meshElements);
		ImGui::NewLine();
		PushMeshElementsUI(cageElements);

		ImGui::Dummy(ImVec2(0.0f, 35.0f));

		const auto buttonSize = ImGui::CalcItemSize(ImVec2(100.0f, 0.0f), 0.0f, 0.0f);
		ImGui::SetCursorPosX(descColumnWidth + settingsColumnWidth - buttonSize.x - ImGui::GetStyle().ItemSpacing.x - ImGui::GetStyle().WindowPadding.x);

		ImGui::BeginDisabled(hasRunningOperation || _modifiedProjectModel == *_model._projectData);
		{
			if (ImGui::Button("Apply", buttonSize))
			{
				// Update the model we actually use.
				*_model._projectData = _modifiedProjectModel;

				_projectSettingsApplied();
			}
		}
		ImGui::EndDisabled();
	}

	ImGui::PopStyleVar();

	ImGui::End();
}

ProjectOptionsPanel::MeshElementsText ProjectOptionsPanel::GetMeshElementsText(const char* label,
	const std::size_t numVertices,
	const std::size_t numEdges,
	const std::size_t numFaces)
{
	return MeshElementsText { label,
		fmt::format("{} vertices", numVertices),
		fmt::format("{} edges", numEdges),
		fmt::format("{} faces", numFaces) };
}

void ProjectOptionsPanel::PushMeshElementsUI(const MeshElementsText& text)
{
	ImGui::PushFont(UIStyle::RegularFontSmall);
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.72f, 0.72f, 0.72f, 1.0f));
	{
		ImGui::SetNextItemWidth(100.0f);
		ImGui::TextEx(text._label.c_str());
		ImGui::SameLine();

		ImGui::TextEx(text._verticesCount.c_str());
		ImGui::SameLine();
		ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
		ImGui::SameLine();

		ImGui::TextEx(text._edgesCount.c_str());
		ImGui::SameLine();
		ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);
		ImGui::SameLine();

		ImGui::TextEx(text._facesCount.c_str());
		ImGui::SameLine();
	}
	ImGui::PopStyleColor();
	ImGui::PopFont();
}

void ProjectOptionsPanel::SetModel(const ProjectOptionsPanelModel& model)
{
	UIPanel::SetModel(model);

	UpdateAfterNewModel();
}

void ProjectOptionsPanel::SetModelData(const std::shared_ptr<ProjectModelData>& modelData)
{
	_model._projectData = modelData;

	UpdateAfterNewModel();
}

void ProjectOptionsPanel::SetDeformableMesh(const std::shared_ptr<PolygonMesh>& deformableMesh)
{
	_model._deformableMesh = deformableMesh;
}

void ProjectOptionsPanel::SetCageMesh(const std::shared_ptr<PolygonMesh>& cageMesh)
{
	_model._cageMesh = cageMesh;
}

void ProjectOptionsPanel::UpdateAfterNewModel()
{
	_selectedDeformationTypeIndex = static_cast<uint32_t>(_model._projectData->_deformationType);
	_selectedWeightingSchemeIndex = static_cast<uint32_t>(_model._projectData->_LBCWeightingScheme);

#ifdef WITH_SOMIGLIANA
	_selectedBulgingTypeIndex = static_cast<uint32_t>(_model._projectData->_somigBulgingType);
#endif

	_modifiedProjectModel = *_model._projectData;
}
