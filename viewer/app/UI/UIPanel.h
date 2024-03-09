#pragma once

#include <UI/UIStyle.h>

#include <optional>
#include <filesystem>
#include <numeric>
#include <nfd.h>
#include <IconsLucide.h>
#include <imgui_internal.h>

struct UIHelpers
{
	static constexpr auto ExtensionsDelimiter = ";";

	static void HelpMarker(const char* desc)
	{
		ImGui::TextDisabled(ICON_LC_BADGE_INFO);

		if (ImGui::IsItemHovered())
		{
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::PushFont(UIStyle::BoldFont);
			ImGui::TextUnformatted(desc);
			ImGui::PopFont();
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
		}
	}

	static void SetRightAligned(const float elementWidth)
	{
		ImGui::SetNextItemWidth(elementWidth);
		const auto posX = ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x + ImGui::GetStyle().FramePadding.x - elementWidth - ImGui::GetScrollX() - ImGui::GetStyle().ItemSpacing.x;

		if (posX > ImGui::GetCursorPosX())
		{
			ImGui::SetCursorPosX(posX);
		}
	}

	static void SetRightAlignedText(const std::string& text)
	{
		const auto textSize = ImGui::CalcTextSize(text.c_str());

		SetRightAligned(textSize.x);
	}

	/**
	 * Pops a dialog to the user to select a file.
	 * @param defaultDirectory The default directory to show when displayed.
	 * @param filterList Allowed file extensions for export.
	 */
	static std::optional<std::filesystem::path> PresentSelectFilePopup(const std::filesystem::path& defaultDirectory,
		const std::vector<nfdfilteritem_t>& filterList)
	{
		using std::filesystem::path;

		nfdchar_t *outPath = nullptr;
		const auto result = NFD_OpenDialog(&outPath, filterList.data(), filterList.size(), defaultDirectory.string().c_str());

		std::optional<path> filepath;
		if (result == NFD_OKAY)
		{
			filepath = outPath;

			free(outPath);
		}
		else if (result == NFD_CANCEL)
		{

		}
		else
		{
			LOG_ERROR("Error: {}\n.", NFD_GetError());
		}

		return filepath;
	}

	/**
	 * Pops a dialog to the user to select an output file directory for export.
	 * @param filterList Allowed file extensions for export.
	 * @param defaultFilename The default filename to use.
	 */
	static std::optional<std::filesystem::path> PresentExportFilePopup(const std::vector<nfdfilteritem_t>& filterList, const std::string& defaultFilename)
	{
		using std::filesystem::path;

		const auto startDirectory = std::filesystem::current_path();

		nfdchar_t *outPath = nullptr;
		const auto result = NFD_SaveDialog(&outPath, filterList.data(), filterList.size(), startDirectory.string().c_str(), defaultFilename.c_str());

		std::optional<path> filepath;
		if (result == NFD_OKAY)
		{
			filepath = outPath;

			free(outPath);
		}
		else if (result == NFD_CANCEL)
		{
			return { };
		}
		else
		{
			LOG_ERROR("Error: {}\n.", NFD_GetError());

			return { };
		}

		return filepath;
	}
};

namespace ImGui
{
	inline void BufferingBar(const char* label, const float value, const ImVec2 barSize, const ImU32 backgroundColor, const ImU32 foregroundColor)
	{
		const auto window = GetCurrentWindow();
		if (window->SkipItems)
		{
			return;
		}

		auto& g = *GImGui;
		const ImGuiStyle& style = g.Style;
		const auto id = window->GetID(label);

		const auto pos = window->DC.CursorPos;
		ImVec2 size = barSize;
		size.x -= style.FramePadding.x * 2;

		const ImRect bb(pos, ImVec2(pos.x + size.x, pos.y + size.y));
		ItemSize(bb, style.FramePadding.y);
		if (!ItemAdd(bb, id))
		{
			return;
		}

		const auto circleStart = size.x * 0.7f;
		const auto circleEnd = size.x;
		const auto circleWidth = circleEnd - circleStart;

		window->DrawList->AddRectFilled(bb.Min, ImVec2(pos.x + circleStart, bb.Max.y), backgroundColor);
		window->DrawList->AddRectFilled(bb.Min, ImVec2(pos.x + circleStart * value, bb.Max.y), foregroundColor);

		const auto t = static_cast<float>(g.Time);
		const auto r = size.y / 2.0f;
		constexpr auto speed = 1.5f;

		constexpr auto a = speed * 0;
		constexpr auto b = speed * 0.333f;
		constexpr auto c = speed * 0.666f;

		const auto o1 = (circleWidth + r) * (t + a - speed * std::roundf((t + a) / speed)) / speed;
		const auto o2 = (circleWidth + r) * (t + b - speed * std::roundf((t + b) / speed)) / speed;
		const auto o3 = (circleWidth + r) * (t + c - speed * std::roundf((t + c) / speed)) / speed;

		window->DrawList->AddCircleFilled(ImVec2(pos.x + circleEnd - o1, bb.Min.y + r), r, backgroundColor);
		window->DrawList->AddCircleFilled(ImVec2(pos.x + circleEnd - o2, bb.Min.y + r), r, backgroundColor);
		window->DrawList->AddCircleFilled(ImVec2(pos.x + circleEnd - o3, bb.Min.y + r), r, backgroundColor);
	}

	inline void Spinner(const char* label, const float radius, const float thickness, const ImU32& color)
	{
		const auto window = GetCurrentWindow();
		if (window->SkipItems)
		{
			return;
		}

		auto& g = *GImGui;
		const ImGuiStyle& style = g.Style;
		const ImGuiID id = window->GetID(label);

		const auto pos = window->DC.CursorPos;
		const ImVec2 size((radius) * 2, (radius + style.FramePadding.y) * 2);

		const ImRect bb(pos, ImVec2(pos.x + size.x, pos.y + size.y));
		ItemSize(bb, style.FramePadding.y);
		if (!ItemAdd(bb, id))
		{
			return;
		}

		// Render
		window->DrawList->PathClear();

		constexpr auto numSegments = 30;
		const auto start = abs(ImSin(g.Time * 1.8f) * (numSegments - 5));

		const auto aMin = IM_PI * 2.0f * start / static_cast<float>(numSegments);
		constexpr auto aMax = IM_PI * 2.0f * (static_cast<float>(numSegments) - 3) / static_cast<float>(numSegments);

		const auto center = ImVec2(pos.x + radius, pos.y + radius + style.FramePadding.y);

		for (std::size_t i = 0; i < numSegments; ++i)
		{
			const auto a = aMin + (static_cast<float>(i) / static_cast<float>(numSegments)) * (aMax - aMin);
			window->DrawList->PathLineTo(ImVec2(center.x + ImCos(a + g.Time * 8.0f) * radius,
				center.y + ImSin(a + g.Time*8) * radius));
		}

		window->DrawList->PathStroke(color, false, thickness);
	}
}

template <typename ModelType>
class UIPanel
{
public:
	UIPanel() = default;

	virtual ~UIPanel() = default;

	/**
	 * Draw the panel on the screen.
	 */
	virtual void Layout() = 0;

	/**
	 * Resets the model of the panel to be a different one.
	 * @param model The new model to set.
	 */
	virtual void SetModel(const ModelType& model)
	{
		_model = model;
	}

protected:
	ModelType _model;
};
