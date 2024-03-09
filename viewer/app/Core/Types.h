#pragma once

template <typename T>
concept IsEnum = std::is_enum_v<T>;

template <typename T> requires IsEnum<T>
constexpr T operator|(const T a, const T b)
{
	return static_cast<T>(static_cast<std::underlying_type_t<T>>(a) | static_cast<std::underlying_type_t<T>>(b));
}

template <typename T> requires IsEnum<T>
constexpr T operator&(const T a, const T b)
{
	return static_cast<T>(static_cast<std::underlying_type_t<T>>(a) & static_cast<std::underlying_type_t<T>>(b));
}

template <typename T> requires IsEnum<T>
constexpr T operator^(const T a, const T b)
{
	return static_cast<T>(static_cast<std::underlying_type_t<T>>(a) ^ static_cast<std::underlying_type_t<T>>(b));
}

template <typename T> requires IsEnum<T>
constexpr T& operator|=(T& a, const T b)
{
	return reinterpret_cast<T&>(reinterpret_cast<std::underlying_type_t<T>&>(a) |= static_cast<std::underlying_type_t<T>>(b));
}

template <typename T> requires IsEnum<T>
constexpr T& operator&=(T& a, const T b)
{
	return reinterpret_cast<T&>(reinterpret_cast<std::underlying_type_t<T>&>(a) &= static_cast<std::underlying_type_t<T>>(b));
}

template <typename T> requires IsEnum<T>
constexpr T& operator^=(T& a, const T b)
{
	return reinterpret_cast<T&>(reinterpret_cast<std::underlying_type_t<T>&>(a) ^= static_cast<std::underlying_type_t<T>>(b) );
}

template <typename T, typename U> requires IsEnum<T> and IsEnum<U>
constexpr bool IsSet(const T a, const U b)
{
	return static_cast<bool>((static_cast<std::underlying_type_t<T>>(a) & static_cast<std::underlying_type_t<U>>(b)) == static_cast<std::underlying_type_t<T>>(b));
}

template <typename T, typename U> requires IsEnum<T> and IsEnum<U>
constexpr void Set(T& a, const U b)
{
	reinterpret_cast<std::underlying_type_t<T>&>(a) = reinterpret_cast<std::underlying_type_t<T>>(a) | static_cast<std::underlying_type_t<U>>(b);
}

static constexpr float Epsilon = 1e-4;
static constexpr float SmallEpsilon = 1e-8;

template <typename T>
static constexpr float Sign(const T value)
{
	return value < T(0.0) ? -1.0f : 1.0f;
}

constexpr std::size_t operator "" _sz (const unsigned long long n)
{
	return n;
}