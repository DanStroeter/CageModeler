#ifndef IMGUI_DEFINE_MATH_OPERATORS
	#define IMGUI_DEFINE_MATH_OPERATORS
#endif

#include <UI/StatusBar.h>
#include <UI/UIStyle.h>
#include <Mesh/PolygonMesh.h>
#include <Mesh/Operations/MeshOperationSystem.h>

#include <imgui_internal.h>

namespace
{
	constexpr auto StatusBarHeight = 40.0f;
	constexpr auto StatusBarButtonSize = ImVec2(25.0f, 25.0f);
	constexpr auto SequencerRightScreenOffset = 400.0f;
}

namespace ImGui
{
	constexpr std::array<uint32_t, 4> SubDivisionsMaxFrames { 10, 50, 100, 200 };
	constexpr std::array<uint32_t, 5> SubDivisions { 1, 2, 5, 10, 20 };

	static_assert(SubDivisionsMaxFrames.size() == SubDivisions.size() - 1, "Size mismatch.");

	constexpr auto NumMaxFramesDraggable = 200;
	constexpr auto MinimumDraggablePixels = 4.0f;
	constexpr auto SequencerFrameTextPadding = ImVec2(8.0f, 2.0f);
	constexpr auto MinimumLabelsDistanceApart = 10.0f;
	constexpr auto HalfHandleWidth = 4.0f;
	constexpr auto SmallDelimiterHeight = 6.0f;
	constexpr auto ThickDelimiterWidth = 4.0f;
	constexpr auto DelimiterWidth = 2.0f;

	inline float ComputeSnap(const float value, const float snapDistance)
	{
		if (snapDistance <= FLT_EPSILON)
		{
			return 0.0f;
		}

		const auto modulo = fmodf(value, snapDistance);
		const auto moduloRatio = fabsf(modulo) / snapDistance;

		if (moduloRatio < 0.5f)
		{
			return value - modulo;
		}
		else
		{
			const auto snapSign = ((value < 0.0f) ? -1.0f : 1.0f);

			return value - modulo + snapDistance * snapSign;
		}
	}

	inline std::size_t GetNumberOfFramesPerDivisionIndex(const uint32_t numFrames)
	{
		for (std::size_t i = 0; i < SubDivisionsMaxFrames.size(); ++i)
		{
			if (numFrames < SubDivisionsMaxFrames[i])
			{
				return i;
			}
		}

		return SubDivisionsMaxFrames.size();
	}

	inline uint32_t GetNumberOfFramesPerDivision(const uint32_t numFrames)
	{
		return SubDivisions[GetNumberOfFramesPerDivisionIndex(numFrames)];
	}

	inline uint32_t Sequencer(const char* label, const uint32_t frameIndex, const uint32_t numFrames, bool& inOutDraggingHandle)
	{
		auto window = GetCurrentWindow();
		if (window->SkipItems)
		{
			return false;
		}

		const auto cursorPos = window->DC.CursorPos;
		const auto canvasSize = GetContentRegionAvail() - ImVec2(SequencerRightScreenOffset, 0.0f);
		const ImRect boundingBox(cursorPos, cursorPos + canvasSize);

		ItemSize(boundingBox);
		if (!ItemAdd(boundingBox, 0))
		{
			return false;
		}

		const auto drawList = window->DrawList;

		const auto textColor = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_Text]);
		const auto inactiveColor = ColorConvertFloat4ToU32(ImVec4(0.75f, 0.75f, 0.75f, 1.0f));
		const auto activeColor = ColorConvertFloat4ToU32(ImVec4(0.9f, 0.9f, 0.9f, 1.0f));
		const auto lineColor = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_Separator]);
		const auto backgroundColor = ColorConvertFloat4ToU32(GImGui->Style.Colors[ImGuiCol_FrameBg]);

		// Compute how to subdivide the existing number of frames into regions.
		const auto maxNumFramesPerDivision = GetNumberOfFramesPerDivision(numFrames);
		auto numDivisions = static_cast<uint32_t>(ImFloor(static_cast<float>(numFrames) / static_cast<float>(maxNumFramesPerDivision)));
		const auto numRestDivisions = numFrames - numDivisions * maxNumFramesPerDivision;

		// Draw the background of the sequencer.
		drawList->AddRectFilled(cursorPos, cursorPos + canvasSize, backgroundColor, 2.0f);

		const auto timelineWidth = canvasSize.x - 2.0f * SequencerFrameTextPadding.x;
		const auto restTimelineWidth = canvasSize.x * (static_cast<float>(numRestDivisions) / static_cast<float>(numFrames));
		const auto labelsOffset = (timelineWidth - restTimelineWidth) / static_cast<float>(numDivisions);

		const auto addDivision = [&](const int32_t index)
		{
			char tmp[4];
			ImFormatString(tmp, sizeof(tmp), "%d", index * maxNumFramesPerDivision);

			const auto textSize = ImGui::CalcTextSize(tmp);
			const auto textOffset = SequencerFrameTextPadding + ImVec2(labelsOffset * static_cast<float>(index), 0.0f);

			// The text of the frame.
			drawList->AddText(cursorPos + textOffset, textColor, tmp);

			// Line to indicate the frame number.
			drawList->AddLine(cursorPos + textOffset + ImVec2(0.0f, canvasSize.y - SequencerFrameTextPadding.y),
				ImVec2(cursorPos + textOffset + ImVec2(0.0f, SequencerFrameTextPadding.y + textSize.y)),
				lineColor,
				ThickDelimiterWidth);
		};

		const auto addSubDivision = [&](const int32_t index, const int32_t divisionIndex)
		{
			const auto frac = static_cast<float>(index) / static_cast<float>(maxNumFramesPerDivision);
			const auto smallLineOffset = SequencerFrameTextPadding + ImVec2(labelsOffset * (static_cast<float>(divisionIndex) + frac), 0.0f);

			drawList->AddLine(cursorPos + smallLineOffset + ImVec2(0.0f, canvasSize.y - SequencerFrameTextPadding.y),
				ImVec2(cursorPos + smallLineOffset + ImVec2(0.0f, canvasSize.y - SequencerFrameTextPadding.y - SmallDelimiterHeight)),
				lineColor,
				DelimiterWidth);
		};

		// Draw all frame numbers.
		for (int32_t divisionIndex = 0; divisionIndex <= numDivisions; ++divisionIndex)
		{
			addDivision(divisionIndex);

			const auto thisNumRestDivisions = numFrames - divisionIndex * maxNumFramesPerDivision;
			const auto maxNumSubdivisions = (std::min)(thisNumRestDivisions, maxNumFramesPerDivision);

			// Intermediate lines between the big segments.
			for (int32_t subdivisionIndex = 1; subdivisionIndex <= maxNumSubdivisions; ++subdivisionIndex)
			{
				addSubDivision(subdivisionIndex, divisionIndex);
			}
		}

		uint32_t nextFrameIndex = frameIndex;

		BeginGroup();
		{
			// We have dragged since last frame, so we want to update the handle progress and the new frame.
			if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && inOutDraggingHandle)
			{
				const auto mouseDelta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);

				if (mouseDelta.x != 0)
				{
					// Calculate how much we have dragged if anything.
					const auto initialMouseClickPos = *GetIO().MouseClickedPos;

					auto minDragDistancePerFrame = timelineWidth / static_cast<float>(numFrames);
					auto minDragNumFrames = 1;

					const auto requiredDragDistancePerFrame = static_cast<float>(1) / NumMaxFramesDraggable * timelineWidth;
					if (numFrames > NumMaxFramesDraggable || minDragDistancePerFrame < requiredDragDistancePerFrame)
					{
						minDragNumFrames = ImCeil(MinimumDraggablePixels / requiredDragDistancePerFrame);
						minDragDistancePerFrame = requiredDragDistancePerFrame;
					}

					const auto minDragDistance = static_cast<float>(minDragNumFrames) * minDragDistancePerFrame;
					const auto currentXValue = initialMouseClickPos.x + mouseDelta.x - cursorPos.x - SequencerFrameTextPadding.x;
					const auto snappedMousePosX = ComputeSnap(ImClamp(currentXValue, 0.0f, timelineWidth), minDragDistance);

					nextFrameIndex = static_cast<uint32_t>(roundf(snappedMousePosX / timelineWidth * static_cast<float>(numFrames)));
				}
			}

			// Calculate the position of the handle.
			const auto handleProgress = static_cast<float>(nextFrameIndex) / static_cast<float>(numFrames);
			const auto handleCenterCoords = SequencerFrameTextPadding.x + handleProgress * timelineWidth;
			const ImVec2 handlePosition(cursorPos.x + handleCenterCoords, cursorPos.y);

			const ImRect handleBoundingBox(handlePosition + ImVec2(0.0f, 4.0f) - ImVec2(HalfHandleWidth, 0.0f), handlePosition + ImVec2(HalfHandleWidth, canvasSize.y));
			const ImGuiID handleId = window->GetID("SequencerHandle");

			PushID(handleId);

			ImRect buttonBoundingBox = handleBoundingBox;
			buttonBoundingBox.Expand(ImVec2(50.0f, 0.0f));
			bool hovered = false;
			bool held = false;
			const auto pressed = ButtonBehavior(buttonBoundingBox, handleId, &hovered, &held, ImGuiButtonFlags_MouseButtonLeft);

			inOutDraggingHandle = (held || inOutDraggingHandle);

			if (IsMouseReleased(ImGuiMouseButton_Left) && inOutDraggingHandle)
			{
				inOutDraggingHandle = false;
			}

			// Draw the handle.
			const auto col = (pressed || hovered || held || inOutDraggingHandle) ? activeColor : inactiveColor;
			drawList->AddRectFilled(handleBoundingBox.Min,
				handleBoundingBox.Max,
				col,
				4.0f,
				ImDrawFlags_RoundCornersTopLeft | ImDrawFlags_RoundCornersTopRight);

			PopID();
		}
		EndGroup();

		return nextFrameIndex;
	}

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

		if (_model->_selectionTypeChangedDelegate)
		{
			_model->_selectionTypeChangedDelegate(selectionType);
		}
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

		const auto oldCursorPositionY = ImGui::GetCursorPosY();
		const auto windowSize = ImGui::GetWindowSize();

		const auto projectData = _model->_projectData.lock();

		const auto prevFrameIndex = _model->_frameIndex;
		const auto wasDraggingSequencerHandle = _model->_isDraggingSequencerHandle;

		ImGui::BeginDisabled(projectData == nullptr);
		{
			if (projectData != nullptr)
			{
				// Draw the sequencer at the bottom.
				_model->_frameIndex = ImGui::Sequencer("Sequencer", _model->_frameIndex, projectData->_numSamples - 1, _model->_isDraggingSequencerHandle);
				_model->_frameIndex = std::clamp(_model->_frameIndex, 0u, static_cast<uint32_t>(projectData->_numSamples - 1));
			}
			else
			{
				// Draw the sequencer at the bottom.
				ImGui::Sequencer("Sequencer", 0, 1, _model->_isDraggingSequencerHandle);
			}

			if (prevFrameIndex != _model->_frameIndex)
			{
				_model->_frameIndexChangedDelegate(_model->_frameIndex);
			}

			if (wasDraggingSequencerHandle != _model->_isDraggingSequencerHandle)
			{
				if (wasDraggingSequencerHandle)
				{
					_model->_endDragDelegate();
				}
				else
				{
					_model->_startDragDelegate();
				}
			}

			const auto sequencerSize = ImGui::GetContentRegionAvail() - ImVec2(SequencerRightScreenOffset, 0.0f);

			ImGui::SetCursorPosY(oldCursorPositionY);
			ImGui::SameLine();

			ImGui::SeparatorEx(ImGuiSeparatorFlags_Vertical);

			ImGui::PushItemWidth(100.0f);
			ImGui::SetCursorPosX(sequencerSize.x + 25.0f);
			ImGui::SetCursorPosY(oldCursorPositionY);

			auto numSamples = static_cast<int32_t>(_model->_numSamples - 1);
			if (ImGui::InputInt("##Project_NumSamples", &numSamples, 1, 5, ImGuiInputTextFlags_CharsDecimal))
			{
				_model->_numSamples = static_cast<uint32_t>(numSamples + 1);
				_model->_frameIndex = std::min(_model->_frameIndex, static_cast<uint32_t>(_model->_numSamples));
				projectData->_numSamples = numSamples + 1;

				_model->_numFramesChangedDelegate(_model->_frameIndex, _model->_numSamples);
			}
			ImGui::PopItemWidth();

			ImGui::SameLine();
			UIHelpers::HelpMarker("Specifies the number of samples.");

			ImGui::SameLine();
		}
		ImGui::EndDisabled();

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
				const auto actionTextSize = ImGui::CalcTextSize(operationDescription.c_str());

				// Set the initial position of the text.
				ImGui::SetCursorPosX(windowSize.x - actionTextSize.x - ImGui::GetStyle().ItemSpacing.x - 24.0f);

				ImGui::TextEx(operationDescription.c_str());
				ImGui::SameLine();

				ImGui::SetCursorPosY(oldCursorPositionY + 2.0f);
				ImGui::Spinner("##LoadingSpinner", 7.0f, 2, ImGui::GetColorU32(ImGuiCol_Text));
			}
		}
		else if (_model->_error.has_value())
		{
			const auto errorText = _model->_error->c_str();
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
