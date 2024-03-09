#pragma once

#include <Tools/Tool.h>

#include <unordered_map>

class ToolSystem : public std::enable_shared_from_this<ToolSystem>
{
public:
	template <typename T, typename... Args>
	void RegisterTool(Args&&... args)
	{
		_registeredTools.insert(std::make_pair(T::GetToolType(), std::make_unique<T>(std::forward<Args>(args)...)));
	}

	template <typename T>
	T* GetTool(const ToolType toolType) const
	{
		CheckFormat(_registeredTools.contains(toolType), "The requested tool type doesn't exist.");


		const auto foundTool = _registeredTools.find(toolType);

		return static_cast<T*>(foundTool->second.get());
	}

	ToolBase* GetTool(const ToolType toolType) const
	{
		CheckFormat(_registeredTools.contains(toolType), "The requested tool type doesn't exist.");

		const auto foundTool = _registeredTools.find(toolType);

		return foundTool->second.get();
	}

	void SetActiveTool(const ToolType toolType)
	{
		CheckFormat(_registeredTools.contains(toolType), "The requested tool type doesn't exist.");

		_activeToolType = toolType;

		const auto foundTool = _registeredTools.find(toolType);
		foundTool->second->Activate();

		_toolSelectionChanged(toolType);
	}

	ToolType GetActiveToolType() const
	{
		return _activeToolType;
	}

	void SetSelectionChangedDelegate(std::function<void (ToolType)> toolSelectionChanged)
	{
		_toolSelectionChanged = std::move(toolSelectionChanged);
	}

private:
	std::unordered_map<ToolType, std::unique_ptr<ToolBase>> _registeredTools;
	std::function<void (ToolType)> _toolSelectionChanged;
	ToolType _activeToolType = ToolType::Invalid;
};
