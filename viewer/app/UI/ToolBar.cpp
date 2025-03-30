#include <UI/ToolBar.h>
#include <UI/UIStyle.h>
#include <Mesh/Operations/MeshOperationSystem.h>
#include <Tools/TransformTool.h>

#include <imgui.h>
#include <imgui_internal.h>

namespace
{
	constexpr auto NumTools = 3;
}

ToolBar::ToolBar(const SubsystemPtr<InputSubsystem>& inputSubsystem,
	const std::shared_ptr<MeshOperationSystem>& meshOperationSystem,
	const std::shared_ptr<ToolSystem>& toolSystem)
	: _inputSubsystem(inputSubsystem)
	, _meshOperationSystem(meshOperationSystem)
	, _toolSystem(toolSystem)
{
	_model = std::make_shared<ToolBarModel>();

	RegisterTool<TranslateTool>();
	RegisterTool<RotateTool>();
	RegisterTool<ScaleTool>();
}

void ToolBar::Layout()
{
	const auto toolButtonSize = ImGui::CalcItemSize(ToolsConstants::ButtonSize, 0.0f, 0.0f);
	const auto toolBarWidth = toolButtonSize.x + 2.0f * UIStyle::WindowPadding.x;
	const auto toolButtonsHeight = 2.0f * UIStyle::WindowPadding.y + NumTools * toolButtonSize.y + NumTools * ImGui::GetStyle().ItemSpacing.y;

	ImGui::SetNextWindowPos(ImVec2(15.0f, 35.0f));
	ImGui::SetNextWindowSize(ImVec2(toolBarWidth, toolButtonsHeight));

	if (ImGui::Begin("Tools Bar", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar))
	{
		for (const auto& [toolType, toolDescription] : _toolsDescriptions)
		{
			ShowToolUI(toolType, toolDescription);
		}
		
		ImGui::End();
	}
}

GizmoType ToolBar::GetActiveGizmoType() const
{
	const auto activeToolType = _toolSystem->GetActiveToolType();

	if (activeToolType == Tools::Transform::Translate)
	{
		return GizmoType::Translate;
	}
	else if (activeToolType == Tools::Transform::Rotate)
	{
		return GizmoType::Rotate;
	}
	else if (activeToolType == Tools::Transform::Scale)
	{
		return GizmoType::Scale;
	}

	return GizmoType::MaxNum;
}

void ToolBar::ShowToolUI(const ToolType toolType, const ToolDescription& toolDescription) const
{
	const auto activeToolType = _toolSystem->GetActiveToolType();

	ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(0.5f, 0.5f));
	if (ImGui::Selectable(toolDescription._name.c_str(), activeToolType == toolType, 0, toolDescription._size))
	{
		_toolSystem->SetActiveTool(toolType);
	}
	ImGui::PopStyleVar();

	if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
	{
		ImGui::BeginTooltip();
		ImGui::PushTextWrapPos(450.0f);
		ImGui::Text("%s (%s)", toolDescription._toolTip.c_str(), toolDescription._keyboardShortcut.c_str());
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
}

void ToolBar::OnToolChanged(const ToolType toolType)
{
	_toolSystem->SetActiveTool(toolType);
}
