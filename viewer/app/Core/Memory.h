#pragma once

// Wrapper functions for aligned memory allocation
// There is currently no standard for this in C++ that works across all platforms and vendors, so we abstract this
struct Memory
{
	static void* AlignedAlloc(const std::size_t size, const std::size_t alignment)
	{
		void *data = nullptr;

#if defined(_MSC_VER) || defined(__MINGW32__)
		data = _aligned_malloc(size, alignment);
#else
		const auto res = posix_memalign(&data, alignment, size);
		if (res != 0)
		{
			data = nullptr;
		}
#endif

		return data;
	}

	static void AlignedFree(void* data)
	{
#if	defined(_MSC_VER) || defined(__MINGW32__)
		_aligned_free(data);
#else
		free(data);
#endif
	}
};
