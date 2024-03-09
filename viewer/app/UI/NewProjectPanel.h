#pragma once

#include <Mesh/Operations/MeshOperation.h>
#include <Mesh/Operations/MeshWeightsParams.h>
#include <UI/UIPanel.h>
#include <UI/ProjectModel.h>

class ThreadPool;
class MeshOperationSystem;

class NewProjectPanel final : public UIPanel<std::shared_ptr<ProjectModelData>>
{
public:
	NewProjectPanel(const std::shared_ptr<MeshOperationSystem>& meshOperationSystem,
		std::function<void ()> cancelDelegate,
		std::function<void ()> createDelegate);

	void Layout() override;

	void SetModel(const std::shared_ptr<ProjectModelData>& model) override;

#if WITH_SOMIGLIANA
	[[nodiscard]] double GetSomiglianaBulging() const
	{
		return _model->GetSomiglianaBulging();
	}

	[[nodiscard]] double GetSomiglianaBlendFactor() const
	{
		return _model->GetSomiglianaBlendFactor();
	}

	[[nodiscard]] BulgingType GetSomiglianaBulgingType() const
	{
		return _model->GetSomiglianaBulgingType();
	}
#endif

	void Present();

	void Dismiss();

	[[nodiscard]] bool IsModalPanelVisible() const
	{
		return _isModalVisible;
	}

private:
	std::weak_ptr<MeshOperationSystem> _meshOperationSystem;

	std::function<void ()> _cancelDelegate;
	std::function<void ()> _createDelegate;

	uint32_t _selectedDeformationTypeIndex = -1;
	uint32_t _selectedWeightingSchemeIndex = -1;

#ifdef WITH_SOMIGLIANA
	uint32_t _selectedBulgingTypeIndex = -1;
#endif

	/// Keeps track inside ImGui whether the New Project popup should be displayed.
	bool _isModalVisible = false;

	uint32_t _createButtonPressed : 1;
	uint32_t _cancelButtonPressed : 1;
};
