#pragma once

#include <Tools/Tool.h>

#include <imgui_internal.h>
#include <IconsLucide.h>

class TranslateTool final : public ToolTemplated<TranslateTool>
{
public:
	using ToolTemplated::ToolTemplated;

	explicit TranslateTool(std::function<void (ToolType)> delegate)
		: _delegate(std::move(delegate))
	{ }

	static ToolType GetToolType()
	{
		return Tools::Transform::Translate;
	}

	void ActivateImpl()
	{

	}

	[[nodiscard]] ToolDescription GetToolDescriptionImpl() const
	{
		const auto toolButtonSize = ImGui::CalcItemSize(ToolsConstants::ButtonSize, 0.0f, 0.0f);

		return ToolDescription { ICON_LC_MOVE_3D, "Move", "W", toolButtonSize };
	}

	[[nodiscard]] InputActionMapping GetActionMappingImpl() const
	{
		return InputActionMapping { "Translate", SDL_KMOD_NONE, 0, { SDLK_w }};
	}

	[[nodiscard]] InputActionEntry GetActionEntryImpl() const
	{
		return InputActionEntry { "Translate",
			[this]<typename ParamsType>(ParamsType&& actionParams)
			{
				_delegate(GetToolType());
			}};
	}

private:
	std::function<void (ToolType)> _delegate;
};

class RotateTool final : public ToolTemplated<RotateTool>
{
public:
	explicit RotateTool(std::function<void (ToolType)> delegate)
		: _delegate(std::move(delegate))
	{ }

	static ToolType GetToolType()
	{
		return Tools::Transform::Rotate;
	}

	void ActivateImpl()
	{

	}

	[[nodiscard]] ToolDescription GetToolDescriptionImpl() const
	{
		const auto toolButtonSize = ImGui::CalcItemSize(ToolsConstants::ButtonSize, 0.0f, 0.0f);

		return ToolDescription { ICON_LC_ROTATE_3D, "Rotate", "E", toolButtonSize };
	}

	[[nodiscard]] InputActionMapping GetActionMappingImpl() const
	{
		return InputActionMapping { "Rotate", SDL_KMOD_NONE, 0, { SDLK_e }};
	}

	[[nodiscard]] InputActionEntry GetActionEntryImpl() const
	{
		return InputActionEntry { "Rotate",
			[this]<typename ParamsType>(ParamsType&& actionParams)
			{
				_delegate(GetToolType());
			}};
	}

private:
	std::function<void (ToolType)> _delegate;
};

class ScaleTool final : public ToolTemplated<ScaleTool>
{
public:
	explicit ScaleTool(std::function<void (ToolType)> delegate)
		: _delegate(std::move(delegate))
	{ }

	static ToolType GetToolType()
	{
		return Tools::Transform::Scale;
	}

	void ActivateImpl()
	{

	}

	[[nodiscard]] ToolDescription GetToolDescriptionImpl() const
	{
		const auto toolButtonSize = ImGui::CalcItemSize(ToolsConstants::ButtonSize, 0.0f, 0.0f);

		return ToolDescription { ICON_LC_SCALE_3D, "Scale", "R", toolButtonSize };
	}

	[[nodiscard]] InputActionMapping GetActionMappingImpl() const
	{
		return InputActionMapping { "Scale", SDL_KMOD_NONE, 0, { SDLK_r }};
	}

	[[nodiscard]] InputActionEntry GetActionEntryImpl() const
	{
		return InputActionEntry { "Scale",
			[this]<typename ParamsType>(ParamsType&& actionParams)
			{
				_delegate(GetToolType());
			}};
	}

private:
	std::function<void (ToolType)> _delegate;
};