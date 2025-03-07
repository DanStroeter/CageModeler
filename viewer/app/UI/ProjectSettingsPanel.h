#pragma once

#include <UI/UIPanel.h>
#include <UI/ProjectModel.h>

class PolygonMesh;
class MeshOperationSystem;

class ProjectSettingsPanel final : public UIPanel<std::shared_ptr<ProjectModelData>>
{
public:
	ProjectSettingsPanel(const std::shared_ptr<ProjectModelData>& model,
		const std::shared_ptr<MeshOperationSystem>& meshOperationSystem,
		std::function<void ()> cancelDelegate,
		std::function<void ()> projectSettingsApplied);

	void Layout() override;

	void SetModel(const std::shared_ptr<ProjectModelData>& model) override;

	std::string GenerateCageFromMesh(const std::string& meshFilePath,int scale);

	void Present();

	void Dismiss();

	[[nodiscard]] bool IsModalPanelVisible() const
	{
		return _isModalVisible;
	}

private:
	std::weak_ptr<MeshOperationSystem> _meshOperationSystem;
	ProjectModelData _modifiedProjectModel;
	std::function<void ()> _projectSettingsCancelled;
	std::function<void ()> _projectSettingsApplied;

	uint32_t _selectedDeformationTypeIndex = -1;
	uint32_t _selectedWeightingSchemeIndex = -1;

#ifdef WITH_SOMIGLIANA
	uint32_t _selectedBulgingTypeIndex = -1;
#endif

	/// Keeps track inside ImGui whether the New Project popup should be displayed.
	bool _isModalVisible = false;

	uint32_t _applyButtonPressed : 1;
	uint32_t _cancelButtonPressed : 1;
};
