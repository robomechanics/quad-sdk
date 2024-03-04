#ifndef VEG_CONSTRUCTIBLE_HPP_D6CRVBJYS
#define VEG_CONSTRUCTIBLE_HPP_D6CRVBJYS

#include "proxsuite/linalg/veg/type_traits/core.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

#include <new>

#if !VEG_HAS_BUILTIN(__has_trivial_destructor) ||                              \
  !VEG_HAS_BUILTIN(__is_trivially_constructible) ||                            \
  !VEG_HAS_BUILTIN(__is_constructible) ||                                      \
  !VEG_HAS_BUILTIN(__is_nothrow_constructible) ||                              \
  !VEG_HAS_BUILTIN(__is_trivially_copyable) ||                                 \
  !VEG_HAS_BUILTIN(__is_trivial) || !VEG_HAS_BUILTIN(__is_final) ||            \
  !VEG_HAS_BUILTIN(__is_empty)
#include <type_traits>
#endif

namespace proxsuite {
namespace linalg {
namespace veg {
namespace concepts {

#if VEG_HAS_BUILTIN(__is_final) || defined(VEG_WITH_CXX14_SUPPORT)
VEG_DEF_CONCEPT_FROM_BUILTIN_OR_STD(typename T, final, T);
#else
VEG_DEF_CONCEPT(typename T, final, (sizeof(T) < 0));
#endif
VEG_DEF_CONCEPT_FROM_BUILTIN_OR_STD(typename T, empty, T);

VEG_DEF_CONCEPT(typename T,
                nothrow_destructible,
                noexcept(static_cast<T*>(nullptr)->~T()));

VEG_DEF_CONCEPT(
  typename T,
  trivially_destructible,
  VEG_HAS_BUILTIN_OR(__has_trivial_destructor,
                     ((_detail::assert_complete<_detail::Wrapper<T>>(),
                       __has_trivial_destructor(T))),
                     (std::is_trivially_destructible<T>::value)));

VEG_DEF_CONCEPT_FROM_BUILTIN_OR_STD(typename T, trivially_copyable, T);

VEG_DEF_CONCEPT_FROM_BUILTIN_OR_TRAIT(typename T,
                                      trivially_default_constructible,
                                      is_trivially_constructible,
                                      T);

VEG_DEF_CONCEPT_FROM_BUILTIN_OR_TRAIT(typename T,
                                      trivially_copy_constructible,
                                      is_trivially_constructible,
                                      T,
                                      T const&);
VEG_DEF_CONCEPT_FROM_BUILTIN_OR_TRAIT(typename T,
                                      trivially_move_constructible,
                                      is_trivially_constructible,
                                      T,
                                      T&&);

VEG_CONCEPT_EXPR((typename T, typename... Ts),
                 (T, Ts...),
                 inplace_constructible,
                 new(static_cast<void*>(nullptr)) T(VEG_DECLVAL(Ts&&)...),
                 true);

VEG_DEF_CONCEPT_FROM_BUILTIN_OR_STD((typename T, typename... Ts),
                                    constructible,
                                    T,
                                    Ts&&...);

VEG_DEF_CONCEPT_FROM_BUILTIN_OR_STD((typename T, typename... Ts),
                                    nothrow_constructible,
                                    T,
                                    Ts&&...);

VEG_DEF_CONCEPT_FROM_BUILTIN_OR_STD((typename From, typename To),
                                    convertible,
                                    From&&,
                                    To);

VEG_DEF_CONCEPT((typename T, typename U),
                implicitly_constructible,
                VEG_CONCEPT(convertible<U&&, T>));

VEG_DEF_CONCEPT(typename T,
                movable,
                VEG_HAS_BUILTIN_OR(__is_constructiblex,
                                   __is_constructible(T, T&&),
                                   (VEG_CONCEPT(constructible<T, T&&>))));
VEG_DEF_CONCEPT(
  typename T,
  nothrow_movable,
  VEG_HAS_BUILTIN_OR(__is_nothrow_constructiblex,
                     __is_nothrow_constructible(T, T&&),
                     (VEG_CONCEPT(nothrow_constructible<T, T&&>))));

VEG_DEF_CONCEPT(typename T, copyable, VEG_CONCEPT(constructible<T, T const&>));
VEG_DEF_CONCEPT(typename T,
                nothrow_copyable,
                VEG_CONCEPT(nothrow_constructible<T, T const&>));

} // namespace concepts
namespace cpo {
template<typename T>
struct is_trivially_constructible;

template<typename T>
struct is_trivially_relocatable;
} // namespace cpo

namespace _detail {
namespace _cpo {

template<bool, template<typename> class Trait, typename T>
struct extract_members_deduce_trait_impl : meta::false_type
{};

template<template<typename> class Trait, typename Tuple>
struct member_trait_and;

template<template<typename> class Trait,
         usize... Is,
         typename... Bases,
         typename... Ts>
struct member_trait_and<
  Trait,
  SimpleITuple<_meta::integer_sequence<usize, Is...>, Ts Bases::*...>>
  : meta::bool_constant<VEG_ALL_OF(Trait<Ts>::value)>
{};

template<template<typename> class Trait, typename T>
struct extract_members_deduce_trait_impl<true, Trait, T>
  : member_trait_and<
      Trait,
      decltype(_detail::member_extract_access<T>::Type::member_pointers())>
{};

template<template<typename> class Trait, typename T>
struct extract_members_deduce_trait
  : extract_members_deduce_trait_impl<_detail::member_extract_access<T>::value,
                                      Trait,
                                      T>
{};

} // namespace _cpo
} // namespace _detail

namespace cpo {
template<typename T>
struct is_trivially_constructible
  : meta::if_t<
      VEG_CONCEPT(trivially_default_constructible<T>),
      meta::true_type,
      _detail::_cpo::extract_members_deduce_trait<is_trivially_relocatable, T>>
{};

template<typename T>
struct is_trivially_relocatable
  : meta::if_t<
      VEG_CONCEPT(trivially_copyable<T>) &&
        VEG_CONCEPT(trivially_move_constructible<T>),
      meta::true_type,
      _detail::_cpo::extract_members_deduce_trait<is_trivially_relocatable, T>>
{};
} // namespace cpo

namespace _detail {
template<typename T>
struct DefaultFn
{
  VEG_INLINE constexpr auto operator()() const&& VEG_NOEXCEPT_IF(
    VEG_CONCEPT(nothrow_constructible<T>)) -> T
  {
    return T();
  }
};
template<typename T>
struct MoveFn
{
  T&& value;
  VEG_INLINE constexpr auto operator()() const&& VEG_NOEXCEPT_IF(
    VEG_CONCEPT(nothrow_movable<T>)) -> T
  {
    return T(VEG_FWD(value));
  }
};
template<typename T>
struct CopyFn
{
  T const& value;
  VEG_INLINE constexpr auto operator()() const&& VEG_NOEXCEPT_IF(
    VEG_CONCEPT(nothrow_copyable<T>)) -> T
  {
    return T(value);
  }
};

template<typename Fn, typename T>
struct WithArg
{
  Fn&& fn;
  T&& arg;
  VEG_INLINE constexpr auto operator()() const&& -> decltype(VEG_FWD(fn)(
    VEG_FWD(arg)))
  {
    return VEG_FWD(fn)(VEG_FWD(arg));
  }
};

} // namespace _detail
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_CONSTRUCTIBLE_HPP_D6CRVBJYS */
