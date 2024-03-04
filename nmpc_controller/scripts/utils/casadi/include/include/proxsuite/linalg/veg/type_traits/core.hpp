#ifndef VEG_CORE_HPP_GJCBNFLAS
#define VEG_CORE_HPP_GJCBNFLAS

#include "proxsuite/linalg/veg/internal/typedefs.hpp"
#include "proxsuite/linalg/veg/internal/integer_seq.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

#if defined(VEG_WITH_CXX20_SUPPORT) &&                                         \
  !VEG_HAS_BUILTIN(__builtin_is_constant_evaluated)
#include <type_traits>
#define VEG_HAS_CONSTEVAL 1
#define VEG_IS_CONSTEVAL() ::std::is_constant_evaluated()
#elif VEG_HAS_BUILTIN(__builtin_is_constant_evaluated)
#define VEG_HAS_CONSTEVAL 1
#define VEG_IS_CONSTEVAL() __builtin_is_constant_evaluated()
#else
#define VEG_HAS_CONSTEVAL 0
#endif

namespace proxsuite {
namespace linalg {
namespace veg {
namespace meta {
namespace nb {
struct is_consteval
{
  constexpr auto operator()() const noexcept -> bool
  {
#if VEG_HAS_CONSTEVAL
    return VEG_IS_CONSTEVAL();
#else
    return true;
#endif
  }
};
} // namespace nb
VEG_NIEBLOID(is_consteval);
} // namespace meta

namespace _detail {
namespace _meta {
template<bool B>
struct conditional_;
template<>
struct conditional_<true>
{
  template<typename T, typename F>
  using type = T;
};
template<>
struct conditional_<false>
{
  template<typename T, typename F>
  using type = F;
};
struct none
{};
} // namespace _meta
} // namespace _detail

namespace meta {

template<template<typename...> class F, typename... Ts>
struct meta_apply
{
  using type = F<Ts...>;
};
template<template<typename...> class F>
struct apply_wrapper
{
  template<typename... Ts>
  using type = F<Ts...>;
};
template<typename T>
struct type_identity
{
  using type = T;
};
template<typename T>
using type_identity_t = typename type_identity<T>::type;

template<bool B, typename T, typename F>
using if_t = typename _detail::_meta::conditional_<B>::template type<T, F>;

template<typename...>
using void_t = void;

template<typename... Preds>
struct disjunction;
template<typename... Preds>
struct conjunction;

template<>
struct disjunction<> : false_type
{};
template<>
struct conjunction<> : true_type
{};

template<typename First, typename... Preds>
struct disjunction<First, Preds...>
  : if_t<First::value, First, disjunction<Preds...>>
{};

template<typename First, typename... Preds>
struct conjunction<First, Preds...>
  : if_t<First::value, conjunction<Preds...>, First>
{};
} // namespace meta

namespace _detail {
namespace _meta {

template<typename T>
struct decay_helper : meta::type_identity<T>
{};
template<typename Ret, typename... Args>
struct decay_helper<Ret(Args...)> : meta::type_identity<Ret (*)(Args...)>
{};
#if defined(VEG_WITH_CXX17_SUPPORT)
template<typename Ret, typename... Args>
struct decay_helper<Ret(Args...) noexcept>
  : meta::type_identity<Ret (*)(Args...) noexcept>
{};
#endif
template<typename T, usize N>
struct decay_helper<T[N]> : meta::type_identity<T*>
{};

using namespace meta;

#if defined(VEG_WITH_CXX14_SUPPORT)

template<typename Enable, template<typename...> class Ftor, typename... Args>
constexpr bool _is_detected_v = false;
template<template<typename...> class Ftor, typename... Args>
constexpr bool _is_detected_v<meta::void_t<Ftor<Args...>>, Ftor, Args...> =
  true;

template<template<typename...> class Op, typename... Args>
constexpr bool is_detected_v = _is_detected_v<void, Op, Args...>;

#else

template<typename Enable, template<typename...> class Ftor, typename... Args>
struct _detector
{
  using value_type = false_type;
};
template<template<typename...> class Ftor, typename... Args>
struct _detector<meta::void_t<Ftor<Args...>>, Ftor, Args...>
{
  using value_type = true_type;
};

template<typename Default, template<typename...> class Ftor, typename... Args>
struct detector : _detector<void, Ftor, Args...>
{};

template<template<typename...> class Op, typename... Args>
using is_detected = typename detector<_meta::none, Op, Args...>::value_type;

#endif

template<typename T>
struct is_pointer
  : false_type
  , type_identity<T>
{};
template<typename T>
struct is_pointer<T*>
  : true_type
  , type_identity<T>
{};

template<typename Base>
struct baseof_wrapper : wrapper_base
{
  using wrapper_base::test;
  static auto test(Base const volatile*) -> true_type;
};

template<typename Base, typename Derived>
using is_base_of =
  decltype(baseof_wrapper<Base>::test(static_cast<Derived>(nullptr)));

template<typename T>
struct is_lvalue_reference : false_type
{};
template<typename T>
struct is_lvalue_reference<T&> : true_type
{};
template<typename T>
struct is_rvalue_reference : false_type
{};
template<typename T>
struct is_rvalue_reference<T&&> : true_type
{};

template<typename T>
struct is_const : false_type
{};
template<typename T>
struct is_const<T const> : true_type
{};

template<typename T, typename = true_type>
struct is_complete : false_type
{};
template<typename T>
struct is_complete<T, bool_constant<sizeof(T) == sizeof(T)> /* NOLINT */>
  : true_type
{};

} // namespace _meta
} // namespace _detail

namespace concepts {
using meta::conjunction;
using meta::disjunction;

#if defined(VEG_WITH_CXX20_SUPPORT)
VEG_DEF_CONCEPT((template<typename...> class Op, typename... Args),
                detected,
                requires { typename Op<Args...>; });
#elif defined(VEG_WITH_CXX14_SUPPORT)
VEG_DEF_CONCEPT((template<typename...> class Op, typename... Args),
                detected,
                _detail::_meta::is_detected_v<Op, Args...>);
#else
VEG_DEF_CONCEPT((template<typename...> class Op, typename... Args),
                detected,
                _detail::_meta::is_detected<Op, Args...>::value);
#endif

VEG_DEF_CONCEPT_BUILTIN_OR_INTERNAL((typename T, typename U), base_of, T, U);

VEG_DEF_CONCEPT(typename T,
                const_type,
                VEG_HAS_BUILTIN_OR(__is_const,
                                   __is_const(T),
                                   (_detail::_meta::is_const<T>::value)));
VEG_DEF_CONCEPT(typename T,
                void_type,
                VEG_CONCEPT(same<void const volatile, T const volatile>));

// can't use __is_pointer because of <bits/cpp_type_traits.h> header
VEG_DEF_CONCEPT(typename T, pointer, _detail::_meta::is_pointer<T>::value);

VEG_DEF_CONCEPT_BUILTIN_OR_INTERNAL(typename T, lvalue_reference, T);
VEG_DEF_CONCEPT_BUILTIN_OR_INTERNAL(typename T, rvalue_reference, T);
VEG_DEF_CONCEPT(typename T,
                reference,
                (VEG_CONCEPT(lvalue_reference<T>) ||
                 VEG_CONCEPT(rvalue_reference<T>)));

#if VEG_HAS_BUILTIN(__is_complete_type)
VEG_DEF_CONCEPT(typename T, complete, __is_complete_type(T));
#else
VEG_DEF_CONCEPT(typename T, complete, _detail::_meta::is_complete<T>::value);
#endif

} // namespace concepts

namespace meta {

template<typename T>
using unref_t = typename _detail::_meta::unref<T&>::type;
template<typename T>
using unptr_t = typename _detail::_meta::is_pointer<T>::type;

template<typename Default, template<typename...> class Op, typename... Args>
using detected_or_t = typename meta::if_t<VEG_CONCEPT(detected<Op, Args...>),
                                          meta::meta_apply<Op, Args...>,
                                          meta::type_identity<Default>>::type;

template<template<typename...> class Op, typename... Args>
using detected_t = detected_or_t<_detail::_meta::none, Op, Args...>;

#ifdef __clang__
template<template<typename...> class Op, typename... Args>
using detected_return_t = Op<Args...>;
#else
template<template<typename...> class Op, typename... Args>
using detected_return_t = detected_or_t<_detail::_meta::none, Op, Args...>;
#endif

template<typename T>
using decay_t = typename _detail::_meta::decay_helper<uncvref_t<T>>::type;
} // namespace meta
template<typename T>
using DoNotDeduce = meta::type_identity_t<T>;
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_CORE_HPP_GJCBNFLAS */
