#pragma once

#include <Logging/Logging.h>

#pragma once

// Detect the current platform.
#if defined(__linux__)
	#define PLATFORM_LINUX 1
#elif defined(_WIN32) || defined(_WIN64)
	#define PLATFORM_WINDOWS 1
	#if defined(WINAPI_FAMILY)
		#include <winapifamily.h>
	#endif
#elif defined(__APPLE__)
	#define PLATFORM_MAC 1
#else
	#error "Unable to determine operating system"
#endif

// Define a debug break based on the platform and compiler.
#ifdef _MSC_VER
	inline void debug_break()
	{
		__debugbreak();
	}
#else
	#if defined(__i386__) || defined(__x86_64__) || defined(_M_X64)
		__attribute__((always_inline))
		__inline__ void debug_break()
		{
			__asm__ volatile("int $0x03");
		}
	#elif defined(__aarch64__) && defined(__APPLE__)
		__attribute__((always_inline))
		__inline__ void debug_break()
		{
			__builtin_debugtrap();
		}
	#endif
#endif

#ifdef _MSC_VER
	#define ALIGN_SIZE(N) __declspec(align(N))
#else
	#define ALIGN_SIZE(N) alignas(N)
#endif

// Define the build type based on the configuration.
#ifdef DNDEBUG
	#define BUILD_DEVELOPMENT 0
	#define BUILD_RELEASE 1
#else
	#define BUILD_DEVELOPMENT 1
	#define BUILD_RELEASE 0
#endif

#if BUILD_DEVELOPMENT
	#define DO_CHECK 1
#else
	#define DO_CHECK 0
#endif

// Define check functions as the usual asserts.
#if DO_CHECK
	#define Check(x)                           \
		do                                     \
		{                                      \
			if (!(x)) [[unlikely]]             \
			{                                  \
				LOG_ERROR("Check triggered."); \
				debug_break();                 \
			}                                  \
		} while(0)

	#define CheckFormat(x, ...)                \
		do                                     \
		{                                      \
			if (!(x)) [[unlikely]]             \
			{                                  \
				LOG_ERROR(__VA_ARGS__);        \
				debug_break();                 \
			}                                  \
		} while(0)
	#define CheckNoEntry(...) CheckFormat(true, __VA_ARGS__)
#else
	#define Check(x) { }
	#define CheckFormat(x, ...) { }
#endif

#if PLATFORM_LINUX
	#define BOOST_NO_EXCEPTIONS
	#include <boost/throw_exception.hpp>

	inline void boost::throw_exception(std::exception const & e)
	{
		// Do nothing, used to disable linker error when using -fno-exceptions
	}
#endif