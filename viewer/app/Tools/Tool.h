#pragma once

#include <Input/InputSubsystem.h>

#include <imgui.h>

class ToolType
{
public:
	ToolType()
		: ToolType(++_sId)
	{ }

	operator std::size_t () const
	{
		return _id;
	}

	friend bool operator==(const ToolType lhs, const ToolType rhs)
	{
		return lhs._id == rhs._id;
	}

	friend bool operator!=(const ToolType lhs, const ToolType rhs)
	{
		return !(lhs == rhs);
	}

	static const ToolType Invalid;

private:
	constexpr explicit ToolType(const std::size_t id)
		: _id(id)
	{
	}

private:
	static inline std::size_t _sId = 0;

	std::size_t _id;
};

template <>
struct std::hash<ToolType>
{
	std::size_t operator()(const ToolType toolType) const noexcept
	{
		return std::hash<std::size_t>()(toolType);
	}
};

namespace Tools
{
	namespace Transform
	{
		const auto inline Translate = ToolType();
		const auto inline Rotate = ToolType();
		const auto inline Scale = ToolType();
		
	}

	namespace Cage
	{
		const auto inline Brush=ToolType();
	}
}

struct ToolDescription
{
	std::string _name;
	std::string _toolTip;
	std::string _keyboardShortcut;
	ImVec2 _size;
};

namespace ToolsConstants
{
	constexpr auto ButtonSize = ImVec2(40.0f, 40.0f);
}

class ToolBase
{
public:
	ToolBase() = default;
	virtual ~ToolBase() = default;

	/**
	 * Activates the current tool.
	 */
	virtual void Activate() = 0;

	/**
	 * Gets the tool description to add to the UI.
	 * @return The tool description to populate the UI.
	 */
	[[nodiscard]] virtual ToolDescription GetToolDescription() const = 0;

	/**
	 * Returns the action mapping for the tool.
	 * The action mapping for the tool.
	 */
	[[nodiscard]] virtual InputActionMapping GetActionMapping() const = 0;

	/**
	 * Returns the action entry for the tool.
	 * The action entry for the tool.
	 */
	[[nodiscard]] virtual InputActionEntry GetActionEntry() const = 0;
};

template <typename T>
class ToolTemplated : public ToolBase
{
protected:
	using ToolBase::ToolBase;

	static ToolType GetToolType()
	{
		return T::GetToolType();
	}

	void Activate() override
	{
		static_cast<T&>(*this).ActivateImpl();
	}

	[[nodiscard]] ToolDescription GetToolDescription() const override
	{
		return static_cast<const T&>(*this).GetToolDescriptionImpl();
	}

	[[nodiscard]] InputActionMapping GetActionMapping() const override
	{
		return static_cast<const T&>(*this).GetActionMappingImpl();
	}

	[[nodiscard]] InputActionEntry GetActionEntry() const override
	{
		return static_cast<const T&>(*this).GetActionEntryImpl();
	}
};
