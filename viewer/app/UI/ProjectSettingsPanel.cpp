#include <UI/ProjectSettingsPanel.h>
#include <UI/ProjectSettingsHelpers.h>
#include <Mesh/Operations/MeshOperationSystem.h>
#include <Mesh/Operations/MeshGenerateCageOperation.h>

ProjectSettingsPanel::ProjectSettingsPanel(const std::shared_ptr<ProjectModelData>& model,
                                           const std::shared_ptr<MeshOperationSystem>& meshOperationSystem,
                                           std::function<void ()> cancelDelegate,
                                           std::function<void()> projectSettingsApplied)
    : _meshOperationSystem(meshOperationSystem)
    , _projectSettingsCancelled(std::move(cancelDelegate))
    , _projectSettingsApplied(std::move(projectSettingsApplied))
    , _applyButtonPressed(false)
    , _cancelButtonPressed(false)
{
    _modifiedProjectModel = *model;

    SetModel(model);
}

void ProjectSettingsPanel::Layout()
{
    if (_isModalVisible)
    {
        ImGui::OpenPopup("ProjectSettings");
    }

    const auto displaySize = ImGui::GetIO().DisplaySize;

    ImGui::SetNextWindowPos(ImVec2(displaySize.x / 2.0f, displaySize.y / 2.5f),
        ImGuiCond_Appearing,
        ImVec2(0.5f, 0.5f));

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(20.0f, 20.0f));

    if (ImGui::BeginPopupModal("ProjectSettings", &_isModalVisible,
        ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings))
    {
        const auto descColumnWidth = std::max(0.2f * displaySize.x, 150.0f);
        const auto settingsColumnWidth = std::max(0.3f * displaySize.x, 250.0f);
        const auto horizontalOffset = descColumnWidth + settingsColumnWidth;

        const auto meshOperationSystem = _meshOperationSystem.lock();
        if (meshOperationSystem == nullptr)
        {
            return;
        }

        const auto hasRunningOperation = (meshOperationSystem->GetCurrentOperation() != nullptr);

        ImGui::BeginDisabled(hasRunningOperation);
        if (ImGui::BeginTable("##SettingsTable", 2, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_SizingFixedSame))
        {
            constexpr auto columnFlags = ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize | ImGuiTableColumnFlags_NoReorder | ImGuiTableColumnFlags_NoHeaderLabel;

            ImGui::TableSetupColumn("##Description", columnFlags, descColumnWidth);
            ImGui::TableSetupColumn("##Settings", columnFlags, settingsColumnWidth);

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
                    UIHelpers::HelpMarker("The weighting scheme for LBC, which controls the level of locality.");
                    ImGui::SameLine();

                    ImGui::TableSetColumnIndex(1);

                    UIHelpers::SetRightAligned(125.0f);

                    if (ImGui::BeginCombo("##WeightingScheme", ProjecSettingsHelpers::LBCWeightingSchemeNames[_selectedWeightingSchemeIndex], ImGuiComboFlags_HeightRegular))
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
                ImGui::TextEx("Input Files");
                ImGui::SetWindowFontScale(1.0f);
                ImGui::PopFont();
            }
            static bool autoGenerate=false;
            ImGui::TableNextRow();
            {
                ImGui::TableSetColumnIndex(0);
                ImGui::TextEx("Mesh File");

                ImGui::TableSetColumnIndex(1);
                ProjecSettingsHelpers::PushFileSelectionUI(_modifiedProjectModel._meshFilepath,
                    "##Mesh",
                    horizontalOffset,
                    { nfdfilteritem_t { "Mesh (.obj,.fbx)", "obj,fbx" } });
            }
            

          const auto hasNoMesh1 = !_modifiedProjectModel._meshFilepath.has_value();
            ImGui::TableNextRow();
            {
                ImGui::TableSetColumnIndex(0);
                ImGui::TextEx("Cage File");
				ImGui::SameLine();
			
            ImGui::Checkbox("Auto", &autoGenerate);
               ImGui::TableSetColumnIndex(1);
                ProjecSettingsHelpers::PushFileSelectionUI(_modifiedProjectModel._cageFilepath,
                    "##Cage",
                    horizontalOffset,
                    { nfdfilteritem_t { "Mesh (.obj,.fbx)", "obj,fbx" } });
       
	   
	    //generate the cage
	  if(autoGenerate){
        
              ImGui::TableNextRow();
            {
                ImGui::TableSetColumnIndex(0);
                
                ImGui::TextEx("   Resolution = 2^N (select N)");
                ImGui::SameLine();
                UIHelpers::HelpMarker("Voxel Resolution");

                ImGui::TableSetColumnIndex(1);
                UIHelpers::SetRightAligned(100.0f);
                ImGui::InputInt("##Cage_voxel_resolution", &_modifiedProjectModel._voxelResolution, 1, 1, ImGuiInputTextFlags_NoHorizontalScroll);
                if (_modifiedProjectModel._voxelResolution < 5)
                    _modifiedProjectModel._voxelResolution = 5;
                else if (_modifiedProjectModel._voxelResolution > 7)
                    _modifiedProjectModel._voxelResolution = 7;
            }
            ImGui::TableNextRow();
            {
                ImGui::TableSetColumnIndex(0);

                ImGui::TextEx("   Target Number of Faces");

                ImGui::SameLine();
                UIHelpers::HelpMarker("The number of faces for cage");

                ImGui::TableSetColumnIndex(1);
                UIHelpers::SetRightAligned(100.0f);
                ImGui::InputInt("##Cage_target_faces", &_modifiedProjectModel._targetNumFaces, 50, 100, ImGuiInputTextFlags_NoHorizontalScroll);
                if (_modifiedProjectModel._targetNumFaces < 100)
                    _modifiedProjectModel._targetNumFaces = 100;
                else if (_modifiedProjectModel._targetNumFaces > 700)
                    _modifiedProjectModel._targetNumFaces = 700;
            }

            ImGui::TableNextRow();
            {
                ImGui::TableSetColumnIndex(0);

                ImGui::TextEx("   Cage Smooth Iterations");

                ImGui::SameLine();
                UIHelpers::HelpMarker("The number of iterations for cage smoothing");

                ImGui::TableSetColumnIndex(1);
                UIHelpers::SetRightAligned(100.0f);
                ImGui::InputInt("##Cage_Smooth_Iterations", &_modifiedProjectModel._smoothIterations, 2, 10, ImGuiInputTextFlags_NoHorizontalScroll);
                if (_modifiedProjectModel._smoothIterations < 0)
                    _modifiedProjectModel._smoothIterations = 0;
                else if (_modifiedProjectModel._smoothIterations > 20)
                    _modifiedProjectModel._smoothIterations = 20;
            
            }
            ImGui::TableNextRow();
            {
                ImGui::TableSetColumnIndex(0);
                ImGui::TextEx("   Tri_Quad Cage");
                ImGui::SameLine();
                UIHelpers::HelpMarker("Generated cage is tri_quad.");

                ImGui::TableSetColumnIndex(1);
                UIHelpers::SetRightAligned(25.0f);
                ImGui::Checkbox("##IsTriQuad", &_modifiedProjectModel._isTriQuad);
                ImGui::SameLine();
            }
    
	 	     ImGui::TableNextRow();{
            ImGui::TableSetColumnIndex(0);
                ImGui::BeginDisabled(hasNoMesh1 );
        {
            if (ImGui::Button("Generate Cage"))
            {
                _modifiedProjectModel._cageFilepath = GenerateCageFromMesh();
            }
        }
        ImGui::EndDisabled();

		 }
		  

	  }       
                ImGui::BeginDisabled(!DeformationTypeHelpers::RequiresEmbedding(_modifiedProjectModel._deformationType));
                {
                    ImGui::TableNextRow();

                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextEx("Embedded Mesh File");

                    ImGui::TableSetColumnIndex(1);
                    ProjecSettingsHelpers::PushFileSelectionUI(_modifiedProjectModel._embeddingFilepath,
                        "##Embedded",
                        horizontalOffset,
                        { nfdfilteritem_t { "Mesh (.msh)", "msh" } });
                }
                ImGui::EndDisabled();
            }

            ImGui::Dummy(ImVec2(0.0f, 4.0f));

            ImGui::TableNextRow();
            {
                // If we have loaded the mesh as an FBX file we have specified the deformed cage as well.
                const auto fbxLoaded = _modifiedProjectModel._meshFilepath.has_value() && _modifiedProjectModel.IsFBX();

                ImGui::BeginDisabled(fbxLoaded);
                {
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextEx("Deformed Cage File (Optional)");

                    ImGui::TableSetColumnIndex(1);
                    ProjecSettingsHelpers::PushFileSelectionUI(_modifiedProjectModel._deformedCageFilepath,
                        "##DeformedCage",
                        horizontalOffset,
                        { nfdfilteritem_t { "Mesh (.obj,.fbx)", "obj,fbx" } });
                }
                ImGui::EndDisabled();
            }

            ImGui::TableNextRow();
            {
                ImGui::TableSetColumnIndex(0);
                ImGui::TextEx("Parameters file (Optional)");

                // Specifies the *.param parameter file to control mesh deformation.
                ImGui::TableSetColumnIndex(1);
                ProjecSettingsHelpers::PushFileSelectionUI(_modifiedProjectModel._parametersFilepath,
                    "##Parameters",
                    horizontalOffset,
                    { nfdfilteritem_t { "Parameters (.param)", "param" } });
            }

            // Path to a *.dmat file.
            ImGui::BeginDisabled(!DeformationTypeHelpers::RequiresEmbedding(_modifiedProjectModel._deformationType));
            {
                ImGui::TableNextRow();
                {
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextEx("Pre-computed Weights (Optional)");

                    ImGui::TableSetColumnIndex(1);
                    ProjecSettingsHelpers::PushFileSelectionUI(_modifiedProjectModel._weightsFilepath,
                        "##Weights",
                        horizontalOffset,
                        { nfdfilteritem_t { "Weights (.dmat)", "dmat" } });
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
                // Add the scale input at the bottom because it's valid for all meshes.
                ImGui::TableSetColumnIndex(0);
                ImGui::TextEx("Scale");
                ImGui::SameLine();
                UIHelpers::HelpMarker("Scale model, embedding and cage by factor.");

                ImGui::TableSetColumnIndex(1);
                UIHelpers::SetRightAligned(100.0f);
                ImGui::InputFloat("##Scale", &_modifiedProjectModel._scalingFactor, 0.01f, 0.0f, "%.2f", ImGuiInputTextFlags_NoHorizontalScroll);
            }

            ImGui::TableNextRow();
            {
                ImGui::TableSetColumnIndex(0);
                ImGui::TextEx("Interpolate weights");
                ImGui::SameLine();
                UIHelpers::HelpMarker("Interpolate weights of model vertices from the embedding (embedding does not contain vertices of model).");

                ImGui::TableSetColumnIndex(1);
                UIHelpers::SetRightAligned(25.0f);
                ImGui::Checkbox("##InterpolateWeights", &_modifiedProjectModel._interpolateWeights);
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
                ImGui::Checkbox("##FindOffset", &_modifiedProjectModel._findOffset);
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
                ImGui::Checkbox("##NoOffset", &_modifiedProjectModel._noOffset);
                ImGui::SameLine();
            }

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
                    ImGui::InputInt("##BBW_Iterations", &_modifiedProjectModel._numBBWSteps, 1, 10, ImGuiInputTextFlags_NoHorizontalScroll);
                }
            }
            ImGui::EndDisabled();
        }
        ImGui::EndDisabled();

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

        ImGui::BeginDisabled(_modifiedProjectModel._deformationType != DeformationType::Somigliana && _modifiedProjectModel._deformationType != DeformationType::MVC);
        {
            ImGui::InputDouble("##SomigNu", &_modifiedProjectModel._somigNu);
            ImGui::SameLine();
            UIHelpers::HelpMarker("The material parameter nu for somigliana deformer.");

            /*auto bulgingValue = _modifiedProjectModel._somigBulging.load();
            if (ImGui::InputDouble("##SomigBulging", &bulgingValue)) {
                _modifiedProjectModel._somigBulging = bulgingValue;
            }
            ImGui::SameLine();
            UIHelpers::HelpMarker("The bulging parameter gamma for somigliana deformer.");

            auto blendFactorValue = _modifiedProjectModel._somigBlendFactor.load();
            if (ImGui::InputDouble("##SomigBlendFactor", &blendFactorValue))
            {
                _modifiedProjectModel._somigBlendFactor = blendFactorValue;
            }
            ImGui::SameLine();
            UIHelpers::HelpMarker("The blending factor for somigliana deformer interpolating between local and global boundary conditions.");

            if (ImGui::BeginCombo("Bulging Type",
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

            ImGui::SameLine();
            UIHelpers::HelpMarker("The bulging type for somigliana deformer.");*/
        }
        ImGui::EndDisabled();

        ImGui::EndTable();

        ImGui::Dummy(ImVec2(0.0f, 15.0f));

        const auto buttonSize = ImGui::CalcItemSize(ImVec2(100.0f, 0.0f), 0.0f, 0.0f);
        ImGui::SetCursorPosX(descColumnWidth + settingsColumnWidth - buttonSize.x * 2.0f - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().WindowPadding.x);

        // Cancel the creation and dismiss the popup.
        if (ImGui::Button("Cancel", buttonSize))
        {
            // No need to clear anything on the modified data since this is going to destroy the object anyways.

            Dismiss();
        }

        ImGui::SameLine();

        const auto hasNoMesh = !_modifiedProjectModel._meshFilepath.has_value() || !_modifiedProjectModel._cageFilepath.has_value();
        const auto hasNoDeformedCage = !_modifiedProjectModel._cageFilepath.has_value();
        const auto hasNoEmbedding = (DeformationTypeHelpers::RequiresEmbedding(_modifiedProjectModel._deformationType) && !_modifiedProjectModel._embeddingFilepath.has_value());
        const auto hasNoChanges = (_modifiedProjectModel == *_model);

        // Create the new project.
        ImGui::BeginDisabled(hasNoMesh || hasNoDeformedCage || hasNoEmbedding || hasRunningOperation || hasNoChanges);
        {
            if (ImGui::Button("Apply", buttonSize))
            {
                // Update the value of the actual model pointer.
                *_model = _modifiedProjectModel;

                _applyButtonPressed = true;
            }
        }
        ImGui::EndDisabled();

        ImGui::EndPopup();
    }

    ImGui::PopStyleVar();

    if (_cancelButtonPressed)
    {
        _projectSettingsCancelled();

        _cancelButtonPressed = false;
    }
    else if (_applyButtonPressed)
    {
        _projectSettingsApplied();

        _applyButtonPressed = false;
    }
}

void ProjectSettingsPanel::Present()
{
    _isModalVisible = true;
}

void ProjectSettingsPanel::Dismiss()
{
    if (_isModalVisible)
    {
        ImGui::CloseCurrentPopup();

        _isModalVisible = false;
    }
}


std::string ProjectSettingsPanel::GenerateCageFromMesh(){


_modifiedProjectModel._closingResult.clear();
const auto meshOperationSystem = _meshOperationSystem.lock();
        if (meshOperationSystem == nullptr)
        {
            return " ";
        }
            
    std::filesystem::path currentpath=__FILE__;
    std::filesystem::path upperpath=currentpath.parent_path().parent_path().parent_path().parent_path();
    std::string outputCageFile = upperpath.string();
#ifdef _WIN32
    outputCageFile += "\\models\\autoCage.obj";
#else
    outputCageFile += "/models/autoCage.obj";
#endif
     
    meshOperationSystem->ExecuteOperation<GenerateCageFromMeshOperation>(
        _modifiedProjectModel._meshFilepath.value().string(),
        outputCageFile,
        _modifiedProjectModel._scalingFactor,
        _modifiedProjectModel._smoothIterations,
        _modifiedProjectModel._targetNumFaces,
        _modifiedProjectModel._closingResult,
        _modifiedProjectModel._voxelResolution,
        _modifiedProjectModel._isTriQuad
    );

    std::ifstream input(outputCageFile);
     
    if(!input){

        //handle cage generatioin failed
        ImGui::OpenPopup("Cage Generation Failed!");
        return "";
    }
    return outputCageFile;


}

void ProjectSettingsPanel::SetModel(const std::shared_ptr<ProjectModelData>& model)
{
    UIPanel::SetModel(model);

    _selectedDeformationTypeIndex = static_cast<uint32_t>(_model->_deformationType);
    _selectedWeightingSchemeIndex = static_cast<uint32_t>(_model->_LBCWeightingScheme);

    //_selectedBulgingTypeIndex = static_cast<uint32_t>(_model->_somigBulgingType);

    _modifiedProjectModel = *_model;
}

