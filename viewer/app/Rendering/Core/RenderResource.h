#pragma once

/// Non thread-safe reference counter.
class RefCounter
{
public:
	void AddRef()
	{
		++_referenceCount;
	}

	bool Release()
	{
		return (--_referenceCount == 0);
	}

private:
	uint32_t _referenceCount = 1;
};

template <typename ResourceType>
class RenderResourceRef;

template <typename ResourceType, typename ReferenceCounterType = RefCounter>
class RenderResource
{
public:
	using RenderResourceType = RenderResourceRef<ResourceType>;
	using Base = ResourceType;
	using RefCounter = ReferenceCounterType;

	RenderResource() = default;

	RenderResource(const RenderResource& other) = delete;
	RenderResource& operator=(const RenderResource& other) = delete;

	RenderResource(RenderResource&& other) = delete;
	RenderResource& operator=(RenderResource&& other) = delete;

	auto GetReferenceImpl() const
	{
		return static_cast<const ResourceType&>(*this).GetReference();
	}

	void AddRefImpl()
	{
		static_cast<ResourceType&>(*this).AddRef();
	}

	bool ReleaseImpl()
	{
		const bool result = _refCounter.Release();

		if (result)
		{
			static_cast<ResourceType&>(*this).Release();
		}

		return result;
	}

protected:
	/**
	 * Used from any subclasses if we want to get a new instance of the object.
	 * @return A new instance pointing to the same object.
	 */
	RenderResourceRef<ResourceType> ReferenceFromThis()
	{
		AddRefImpl();

		return RenderResourceRef<ResourceType>(static_cast<ResourceType *>(this));
	}

private:
	ReferenceCounterType _refCounter;
};

template <typename ResourceType>
class RenderResourceRef
{
public:
	template <typename OtherResourceType> friend class RenderResourceRef;

	using ReferenceBase = RenderResource<
		typename ResourceType::Base,
		typename ResourceType::RefCounter>;

	RenderResourceRef() = default;
	~RenderResourceRef()
	{
		if (_data != nullptr)
		{
			_data = nullptr;
		}
	}

	explicit RenderResourceRef(ResourceType* data)
		: _data(data)
	{ }

	template <typename OtherResourceType>
	RenderResourceRef& operator=(const RenderResourceRef<OtherResourceType>& other)
	{
		Reset();

		_data = static_cast<ResourceType*>(other._data);

		if (_data != nullptr)
		{
			static_cast<ReferenceBase *>(_data)->AddRefImpl();
		}

		return *this;
	}

	RenderResourceRef& operator=(const RenderResourceRef& other)
	{
		if (this != &other)
		{
			Reset();

			_data = other._data;

			if (_data != nullptr)
			{
				static_cast<ReferenceBase *>(_data)->AddRefImpl();
			}
		}

		return *this;
	}

	template <typename OtherResourceType>
	explicit RenderResourceRef(const RenderResourceRef<OtherResourceType> &other)
	{
		*this = other;
	}

	RenderResourceRef(const RenderResourceRef& other) = default;

	template <typename OtherResourceType>
	RenderResourceRef &operator=(RenderResourceRef<OtherResourceType>&& other) noexcept
	{
		Reset();

		_data = other._data;
		other._data = nullptr;

		return *this;
	}

	RenderResourceRef &operator=(RenderResourceRef&& other) noexcept
	{
		if (this != &other)
		{
			Reset();

			_data = other._data;
			other._data = nullptr;
		}

		return *this;
	}

	template <typename OtherResourceType>
	RenderResourceRef(RenderResourceRef<OtherResourceType>&& other) noexcept
	{
		*this = std::move(other);
	}

	ResourceType& operator*()
	{
		return *_data;
	}

	const ResourceType& operator*() const
	{
		return *_data;
	}

	ResourceType* operator->()
	{
		return _data;
	}

	const ResourceType* operator->() const
	{
		return _data;
	}

	explicit operator bool() const
	{
		return _data != nullptr;
	}

	bool operator==(const RenderResourceRef& other) const
	{
		return _data == other._data;
	}

	bool operator!=(const RenderResourceRef& other) const
	{
		return _data != other._data;
	}

	void Reset()
	{
		if (_data != nullptr)
		{
			static_cast<ReferenceBase *>(_data)->ReleaseImpl();

			_data = nullptr;
		}
	}

	operator auto() const
	{
		return static_cast<ReferenceBase *>(_data)->GetReferenceImpl();
	}

private:
	ResourceType* _data = nullptr;
};

template <typename ResourceType, typename... Args>
static RenderResourceRef<ResourceType> CreateRenderResource(Args&&... args)
{
	return RenderResourceRef<ResourceType>(new ResourceType(std::forward<Args>(args)...));
}

template <typename BaseResourceType, typename DerivedResourceType, typename... Args>
static typename BaseResourceType::RenderResourceType CreateDerivedRenderResource(Args&&... args)
{
	return typename BaseResourceType::RenderResourceType(new DerivedResourceType(std::forward<Args>(args)...));
}
