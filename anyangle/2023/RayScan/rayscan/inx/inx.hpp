#ifndef GMLIB_INX_HPP_INCLUDED
#define GMLIB_INX_HPP_INCLUDED

#include <climits>
#include <cfloat>
#include <cstdint>
#include <cerrno>
#include <cassert>
#include <cstddef>
#include <exception>
#include <limits>
#include <type_traits>
#include <memory>
#include <utility>
#include <tuple>
#include <functional>

namespace inx {

#define INX_COMMA ,

#ifndef __FAST_MATH__
template <typename T>
inline constexpr T inf = std::numeric_limits<T>::infinity();
#else
template <typename T>
inline constexpr T inf = std::numeric_limits<T>::max();
#endif
template <typename T>
inline constexpr T pi = static_cast<T>(3.1415926535897932384626433);
template <typename T>
inline constexpr T epsilon = static_cast<T>(std::is_same_v<T, float> ? 1e-4 : 1e-8);

using int8 = std::int8_t;
using int16 = std::int16_t;
using int32 = std::int32_t;
using int64 = std::int64_t;
using intptr = std::intptr_t;

using uint8 = std::uint8_t;
using uint16 = std::uint16_t;
using uint32 = std::uint32_t;
using uint64 = std::uint64_t;
using uintptr = std::intptr_t;

using size_t = std::size_t;
using ssize_t = std::make_signed_t<size_t>;
using ptrdiff_t = std::ptrdiff_t;

template <bool B, auto T, auto F>
struct conditional_value;
template <auto T, auto F>
struct conditional_value<true, T, F> : std::integral_constant<decltype(T), T>
{ };
template <auto T, auto F>
struct conditional_value<false, T, F> : std::integral_constant<decltype(F), F>
{ };
template <bool B, auto T, auto F>
inline constexpr auto conditional_value_v = conditional_value<B, T, F>::value;

template <typename T, size_t I>
struct count_pointer_aux : std::integral_constant<size_t, I>
{ };
template <typename T, size_t I>
struct count_pointer_aux<T*, I> : count_pointer_aux<T, I+1>
{ };
template <typename T>
struct count_pointer : count_pointer_aux<T, 0>
{ };
template <typename T>
inline constexpr std::size_t count_pointer_v = count_pointer<T>::value;

template <typename T, size_t I>
struct add_const_pointer_aux;
template <typename T, size_t I>
struct add_const_pointer_aux<T*, I>
{
	using type = typename add_const_pointer_aux<T, I - 1>::type*;
};
template <typename T, size_t I>
struct add_const_pointer_aux<T*const, I>
{
	using type = typename add_const_pointer_aux<T, I - 1>::type* const;
};
template <typename T, size_t I>
struct add_const_pointer_aux<T*volatile, I>
{
	using type = typename add_const_pointer_aux<T, I - 1>::type* volatile;
};
template <typename T, size_t I>
struct add_const_pointer_aux<T*const volatile, I>
{
	using type = typename add_const_pointer_aux<T, I - 1>::type* const volatile;
};
template <typename T>
struct add_const_pointer_aux<T, 0>
{
	using type = std::add_const_t<T>;
};
template <typename T, size_t I>
struct add_const_pointer
{
	static_assert(I <= count_pointer_v<T>, "I must fall within count_poiner");
	using type = typename add_const_pointer_aux<T, count_pointer_v<T> - I>::type;
};
template <typename T, size_t I>
using add_const_pointer_t = typename add_const_pointer<T, I>::type;

template <typename L, typename T>
struct _Apply_Each;

template <typename T, typename L>
void apply_each(L&& la)
{
	_Apply_Each<L, T>::apply(std::forward<L>(la));
}

template <typename L, typename T, T... Ints>
struct _Apply_Each<L, std::integer_sequence<T, Ints...>>
{
	static void apply(L&& la)
	{
		apply<Ints...>(std::forward<L>(la));
	}
	template <T I, T... IS>
	static void apply(L&& la)
	{
		la(I);
		if constexpr (sizeof...(IS) > 0)
			apply<IS...>(std::forward<L>(la));
	}
};

}

#endif // GMLIB_INX_HPP_INCLUDED
