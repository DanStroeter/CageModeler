#pragma once

#include <Mesh/Operations/MeshWeightsParams.h>
#include <UI/UIPanel.h>
#include <Editor/Gizmo.h>
#include <Input/InputSubsystem.h>
#include <Tools/ToolSystem.h>

class MeshOperationSystem;

struct ToolBarModel
{
	ToolBarModel() = default;

	explicit ToolBarModel(const std::shared_ptr<ProjectData>& projectData)
		: _projectData(projectData)
	{ }

	std::weak_ptr<ProjectData> _projectData;
};

enum class SelectionToolType : uint8_t
{
	Vertex = 0,
	Edge,
	Polygon,
	None
};

class ToolBar final : public UIPanel<std::shared_ptr<ToolBarModel>>
{
public:
	ToolBar(const SubsystemPtr<InputSubsystem>& inputSubsystem,
		const std::shared_ptr<MeshOperationSystem>& meshOperationSystem,
		const std::shared_ptr<ToolSystem>& toolSystem);

	void Layout() override;

	[[nodiscard]] GizmoType GetActiveGizmoType() const;

	[[nodiscard]] SelectionToolType GetSelectionToolType() const;

private:
	void ShowToolUI(const ToolType toolType, const ToolDescription& toolDescription) const;

	void OnToolChanged(const ToolType toolType);

	template <typename T>
	void RegisterTool()
	{
		_toolSystem->RegisterTool<T>([this]<typename U>(U&& toolType) { OnToolChanged(std::forward<U>(toolType)); });

		const auto toolType = T::GetToolType();
		const ToolBase* toolBase = _toolSystem->GetTool(toolType);
		_toolsDescriptions.insert(std::make_pair(toolType, toolBase->GetToolDescription()));

		_inputSubsystem->RegisterInputActionMapping(toolBase->GetActionMapping());
		_inputSubsystem->RegisterInputActionEntry(toolBase->GetActionEntry());
	}

private:
	SubsystemPtr<InputSubsystem> _inputSubsystem = nullptr;
	std::shared_ptr<MeshOperationSystem> _meshOperationSystem = nullptr;
	std::shared_ptr<ToolSystem> _toolSystem = nullptr;
	std::map<ToolType, ToolDescription> _toolsDescriptions;
};
