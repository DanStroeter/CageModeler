#pragma once

#include <Mesh/PolygonMesh.h>
#include <Mesh/Operations/MeshOperation.h>

template <typename OperationType>
using OperationResult = decltype(std::declval<OperationType>().Execute());

class MeshOperationSystem : public std::enable_shared_from_this<MeshOperationSystem>
{
public:
	MeshOperationSystem() = default;

	template <typename OperationType, typename = std::enable_if_t<std::is_void_v<OperationResult<OperationType>>>, typename... Args>
	void ExecuteOperation(Args&&... args)
	{
		auto newOperation = std::make_unique<OperationType>(typename OperationType::ConstructionParams(std::forward<Args>(args)...));
		_currentMeshOperation.store(newOperation.get(), std::memory_order_relaxed);
		newOperation->Execute();

		_currentMeshOperation.store(nullptr, std::memory_order_relaxed);
	}

	template <typename OperationType, typename = std::enable_if_t<!std::is_void_v<OperationResult<OperationType>>>, typename... Args>
	auto ExecuteOperation(Args&&... args) -> decltype(std::declval<OperationType>().Execute())
	{
		auto newOperation = std::make_unique<OperationType>(typename OperationType::ConstructionParams(std::forward<Args>(args)...));
		_currentMeshOperation.store(newOperation.get(), std::memory_order_relaxed);
		auto operationResult = newOperation->Execute();

		_currentMeshOperation.store(nullptr, std::memory_order_relaxed);

		return operationResult;
	}

	const MeshOperation* GetCurrentOperation() const
	{
		return _currentMeshOperation.load(std::memory_order_relaxed);
	}

private:
	/// Stores an atomic pointer to the currently executed mesh operation, so we can read data from it if needed.
	std::atomic<MeshOperation*> _currentMeshOperation = nullptr;
};
