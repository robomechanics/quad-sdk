#ifndef VEG_GET_HPP_QRS8GNSHS
#define VEG_GET_HPP_QRS8GNSHS

#include "proxsuite/linalg/veg/type_traits/core.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
namespace meta {
template<typename T>
struct is_bounded_array : false_type
{};
template<typename T, usize N>
struct is_bounded_array<T[N]> : true_type
{};

template<typename T>
struct array_extent : constant<usize, 0>
{};
template<usize N, typename T>
struct array_extent<T[N]> : constant<usize, N>
{};
} // namespace meta

namespace _detail {
namespace _meta {

template<typename T>
void
get() = delete;

struct array_get
{
  template<typename I, typename T>
  using result_type = decltype(VEG_DECLVAL(T &&)[I::value]);

  template<usize I, typename T>
  VEG_INLINE static constexpr auto apply(T&& arr)
    VEG_NOEXCEPT->decltype(VEG_FWD(arr)[I])
  {
    return VEG_FWD(arr)[I];
  }
};

struct member_get
{
  template<typename I, typename T>
  using result_type = decltype(VEG_DECLVAL(T &&).template get<I::value>());
  template<usize I, typename T>
  VEG_INLINE static constexpr auto apply(T&& arg)
    VEG_DEDUCE_RET(VEG_FWD(arg).template get<I>());
};
struct adl_get
{
  template<typename I, typename T>
  using result_type = decltype(get<I::value>(VEG_DECLVAL(T &&)));

  template<usize I, typename T>
  VEG_INLINE static constexpr auto apply(T&& arg)
    VEG_DEDUCE_RET(get<I>(VEG_FWD(arg)));
};

struct none_found : true_type
{
  template<typename I, typename T>
  using result_type = void;
};

template<typename I, typename T>
using member_get_expr = decltype(VEG_DECLVAL(T &&).template get<I::value>());
template<typename I, typename T>
using adl_get_expr = decltype(get<I::value>(VEG_DECLVAL(T &&)));

template<usize I, typename T>
struct has_array_get
  : meta::bool_constant<meta::is_bounded_array<meta::uncvref_t<T>>::value>
  , array_get
{};

template<usize I, typename T>
struct has_member_get
  : bool_constant<VEG_CONCEPT(
      detected<member_get::result_type, constant<usize, I>, T&&>)>
  , member_get
{};
template<usize I, typename T>
struct has_adl_get
  : bool_constant<VEG_CONCEPT(
      detected<adl_get::result_type, constant<usize, I>, T&&>)>
  , adl_get
{};

} // namespace _meta
} // namespace _detail

namespace concepts {
namespace aux {
VEG_DEF_CONCEPT((usize I, usize N), less_than, (I < N));
VEG_DEF_CONCEPT(typename T, bounded_array, meta::is_bounded_array<T>::value);
} // namespace aux
VEG_DEF_CONCEPT_CONJUNCTION(
  (usize I, typename T),
  array_gettable,
  ((aux::, bounded_array<uncvref_t<T>>),
   (aux::, less_than<I, meta::array_extent<uncvref_t<T>>::value>)));
VEG_DEF_CONCEPT(
  (usize I, typename T),
  member_gettable,
  VEG_CONCEPT(
    detected<_detail::_meta::member_get_expr, meta::constant<usize, I>, T>));
VEG_DEF_CONCEPT(
  (usize I, typename T),
  adl_gettable,
  VEG_CONCEPT(
    detected<_detail::_meta::adl_get_expr, meta::constant<usize, I>, T>));

VEG_DEF_CONCEPT_DISJUNCTION((usize I, typename T),
                            gettable,
                            ((, array_gettable<I, T>),
                             (, member_gettable<I, T>),
                             (, adl_gettable<I, T>)));
} // namespace concepts

namespace nb {
template<usize I>
struct get
{
  VEG_TEMPLATE(
    (typename T,
     typename Impl = meta::disjunction<_detail::_meta::has_array_get<I, T>,
                                       _detail::_meta::has_member_get<I, T>,
                                       _detail::_meta::has_adl_get<I, T>,
                                       _detail::_meta::none_found>),
    requires(VEG_CONCEPT(gettable<I, T>)),
    VEG_INLINE constexpr auto
    operator(),
    (arg, T&&))
  const VEG_NOEXCEPT_IF(VEG_IS_NOEXCEPT(Impl::template apply<I>(VEG_FWD(arg))))
    ->meta::detected_t<Impl::template result_type, meta::constant<usize, I>, T>
  {
    return Impl::template apply<I>(VEG_FWD(arg));
  }
};
} // namespace nb

VEG_NIEBLOID_TEMPLATE(usize I, get, I);
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_GET_HPP_QRS8GNSHS */
