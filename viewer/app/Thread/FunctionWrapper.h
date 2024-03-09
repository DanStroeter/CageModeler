#pragma once

#include <thread>

class JoinThreads
{
public:
	explicit JoinThreads(std::vector<std::thread>& threads)
		: _threads(threads)
	{ }

	~JoinThreads()
	{
		for (auto& _thread : _threads.get())
		{
			if (_thread.joinable())
			{
				_thread.join();
			}
		}
	}

private:
	std::reference_wrapper<std::vector<std::thread>> _threads;
};

class FunctionWrapper
{
public:
	template <typename FunctionType>
	FunctionWrapper(FunctionType&& f)
		: _impl(std::make_unique<ImplType<FunctionType>>(std::forward<FunctionType>(f)))
	{ }

	FunctionWrapper() = default;

	FunctionWrapper(const FunctionWrapper& other) = delete;
	FunctionWrapper& operator=(const FunctionWrapper& other) = delete;

	FunctionWrapper(FunctionWrapper&& other) noexcept
		: _impl(std::move(other._impl))
	{ }

	FunctionWrapper& operator=(FunctionWrapper&& other) noexcept
	{
		_impl = std::move(other._impl);

		return *this;
	}

	void operator()() const
	{
		_impl->Call();
	}

private:
	struct ImplBase
	{
		virtual ~ImplBase() = default;
		virtual void Call() = 0;
	};

	template <typename FunctionType>
	struct ImplType final : ImplBase
	{
		ImplType(FunctionType&& f)
			: _func(std::forward<FunctionType>(f))
		{ }

		void Call() override
		{
			_func();
		}

		FunctionType _func;
	};

private:
	std::unique_ptr<ImplBase> _impl = nullptr;
};
