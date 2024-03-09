#pragma once

#include <variant>

using MeshOperationError = std::string;

template <typename T>
struct MeshOperationResult
{
	MeshOperationResult(T&& result)
		: _result(std::forward<T>(result))
	{

	}

	MeshOperationResult(MeshOperationError result)
		: _result(std::move(result))
	{

	}

	[[nodiscard]] const T& GetValue() const
	{
		return std::get<T>(_result);
	}

	[[nodiscard]] T& GetValue()
	{
		return std::get<T>(_result);
	}

	[[nodiscard]] const MeshOperationError& GetError() const
	{
		return std::get<MeshOperationError>(_result);
	}

	[[nodiscard]] MeshOperationError& GetError()
	{
		return std::get<MeshOperationError>(_result);
	}

	[[nodiscard]] bool HasError() const
	{
		return (_result.index() == 1);
	}

	std::variant<T, MeshOperationError> _result;
};

class MeshOperation
{
public:
	virtual ~MeshOperation() = default;

	[[nodiscard]] virtual std::string GetDescription() const = 0;
};

template <typename ParamsType, typename ExecutionResultType>
class MeshOperationTemplated : public MeshOperation
{
public:
	using ConstructionParams = ParamsType;
	using ExecutionResult = MeshOperationResult<ExecutionResultType>;

	MeshOperationTemplated() = default;
	explicit MeshOperationTemplated(const ParamsType& params)
		: _params(params)
	{ }

protected:
	ParamsType _params;
};
