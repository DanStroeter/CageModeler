#include <UI/StatusBar.h>
#include <UI/UIStyle.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/Operations/MeshOperationSystem.h>

#include <imgui_internal.h>

namespace
{
	constexpr auto StatusBarHeight = 40.0f;
	constexpr auto StatusBarButtonSize = ImVec2(25.0f, 25.0f);
}

StatusBar::StatusBar(const std::shared_ptr<MeshOperationSystem>& meshOperationSystem)
	: _meshOperationSystem(meshOperationSystem)
{
	_model = std::make_shared<StatusBarModel>();
}

void StatusBar::SetModel(const std::shared_ptr<StatusBarModel>& model)
{
	UIPanel::SetModel(model);

	model->_selectionTypeChangedDelegate(model->_activeSelectionType);
}

void StatusBar::LayoutSelectionTool(const SelectionType selectionType, const char* label, const char* toolTip, const ImVec2& size) const
{
	ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(0.5f, 0.5f));
	if (ImGui::Selectable(label, _model->_activeSelectionType == selectionType, 0, size))
	{
		_model->_activeSelectionType = selectionType;
		_model->_selectionTypeChangedDelegate(selectionType);
	}
	ImGui::PopStyleVar();

	if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
	{
		ImGui::BeginTooltip();
		ImGui::PushTextWrapPos(450.0f);
		ImGui::TextUnformatted(toolTip);
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
}

void StatusBar::Layout()
{
	// Get the global display size.
	const ImGuiIO& io = ImGui::GetIO();
	const auto displaySize = io.DisplaySize;

	ImGui::SetNextWindowPos(ImVec2(0.0f, displaySize.y - StatusBarHeight));
	ImGui::SetNextWindowSize(ImVec2(displaySize.x, StatusBarHeight));

	if (ImGui::Begin("Status Bar", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar))
	{
		// Reset the error message on click.
		if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
		{
			_model->_error.reset();
		}

		ImGui::Dummy(ImVec2(4.0f, 0.0f));
		ImGui::SameLine();

		// Add the selection buttons to the bar.
		const auto toolButtonSize = ImGui::CalcItemSize(StatusBarButtonSize, 0.0f, 0.0f);
		LayoutSelectionTool(SelectionType::Vertex, ICON_LC_DOT, "Vertex", toolButtonSize);
		ImGui::SameLine();
		ImGui::Dummy(ImVec2(1.0f, 0.0f));
		ImGui::SameLine();
		LayoutSelectionTool(SelectionType::Edge, ICON_LC_TRIANGLE_RIGHT, "Edge", toolButtonSize);
		ImGui::SameLine();
		ImGui::Dummy(ImVec2(1.0f, 0.0f));
		ImGui::SameLine();
		LayoutSelectionTool(SelectionType::Polygon, ICON_LC_TRIANGLE, "Polygon", toolButtonSize);
		ImGui::SameLine();
		ImGui::Dummy(ImVec2(1.0f, 0.0f));
		ImGui::SameLine();

		const auto meshOperationSystem = _meshOperationSystem.lock();
		if (meshOperationSystem == nullptr)
		{
			ImGui::End();

			return;
		}

		const auto currentOperation = meshOperationSystem->GetCurrentOperation();
		if (currentOperation != nullptr)
		{
			const auto operationDescription = currentOperation->GetDescription();

			if (!operationDescription.empty())
			{
				const auto windowSize = ImGui::GetWindowSize();
				const auto actionTextSize = ImGui::CalcTextSize(operationDescription.c_str());

				// Set the initial position of the text.
				ImGui::SetCursorPosX(windowSize.x - actionTextSize.x - ImGui::GetStyle().ItemSpacing.x - 24.0f);

				ImGui::TextEx(operationDescription.c_str());
				ImGui::SameLine();

				ImGui::Spinner("##LoadingSpinner", 7.0f, 2, ImGui::GetColorU32(ImGuiCol_Text));
			}
		}
		else if (_model->_error.has_value())
		{
			const auto errorText = _model->_error->c_str();
			const auto windowSize = ImGui::GetWindowSize();
			const auto actionTextSize = ImGui::CalcTextSize(errorText);

			// Set the initial position of the text.
			ImGui::SetCursorPosX(windowSize.x - actionTextSize.x - ImGui::GetStyle().ItemSpacing.x);

			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.8f, 0.2f, 0.3f, 1.0f));
			{
				ImGui::TextEx(errorText);
			}
			ImGui::PopStyleColor();
		}

		ImGui::End();
	}
}
