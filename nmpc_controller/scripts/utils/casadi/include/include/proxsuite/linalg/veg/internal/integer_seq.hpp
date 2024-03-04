#ifndef VEG_INTEGER_SEQ_HPP_JBT0EKAQS
#define VEG_INTEGER_SEQ_HPP_JBT0EKAQS

#include "proxsuite/linalg/veg/internal/typedefs.hpp"
#include "proxsuite/linalg/veg/internal/macros.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
namespace meta {

template<typename T, T N>
using make_integer_sequence = _detail::_meta::make_integer_sequence<T, N>*;
template<usize N>
using make_index_sequence = _detail::_meta::make_integer_sequence<usize, N>*;

template<typename T, T... Nums>
using integer_sequence = _detail::_meta::integer_sequence<T, Nums...>*;
template<usize... Nums>
using index_sequence = integer_sequence<usize, Nums...>;
template<typename... Ts>
using type_sequence = _detail::_meta::type_sequence<Ts...>*;

template<typename Seq, typename... Bs>
struct and_test : false_type
{};
template<typename Seq, typename... Bs>
struct or_test : true_type
{};

template<usize Is, typename T>
using indexed = T;

template<typename... Ts>
struct pack_size
{
  static constexpr usize value = sizeof...(Ts);
};
template<usize... Is>
struct and_test<index_sequence<Is...>, indexed<Is, true_type>...> : true_type
{};
template<usize... Is>
struct or_test<index_sequence<Is...>, indexed<Is, false_type>...> : false_type
{};

} // namespace meta
namespace _detail {
namespace _meta {
using namespace meta;

template<typename ISeq, typename... Ts>
struct all_same_impl : false_type
{};
template<usize... Is, typename T>
struct all_same_impl<meta::index_sequence<Is...>,
                     discard_1st<decltype(Is), T>...> : true_type
{};

template<>
struct all_same_impl<meta::index_sequence<>> : true_type
{};
} // namespace _meta
} // namespace _detail

namespace concepts {
VEG_DEF_CONCEPT(
  typename... Ts,
  all_same,
  _detail::_meta::all_same_impl<meta::make_index_sequence<sizeof...(Ts)>,
                                Ts...>::value);
} // namespace concepts

namespace _detail {
namespace _meta {
template<template<typename...> class F, typename Seq>
struct apply_type_seq;
template<template<typename...> class F, typename... Ts>
struct apply_type_seq<F, meta::type_sequence<Ts...>>
{
  using type = F<Ts...>;
};

template<typename Valid, template<typename...> class F, typename... Seqs>
struct concat_type_seq;

template<typename Valid, template<typename...> class F, typename... Seqs>
struct zip_type_seq;
template<template<typename...> class F, typename... Seqs>
struct zip_type_seq2;

template<template<typename...> class F>
struct zip_type_seq<meta::true_type, F>
{
  using type = F<>;
};

template<template<typename...> class F, typename... Ts>
struct zip_type_seq<meta::true_type, F, F<Ts...>>
{
  using type = F<F<Ts>...>;
};

template<template<typename...> class F, typename... Ts, typename... Zipped>
struct zip_type_seq2<F, F<Ts...>, F<Zipped...>>
{
  using type =
    F<typename concat_type_seq<true_type, F, F<Ts>, Zipped>::type...>;
};

template<template<typename...> class F, typename T>
struct specializes : meta::false_type
{};
template<template<typename...> class F, typename... Ts>
struct specializes<F, F<Ts...>> : meta::true_type
{};

template<template<typename...> class F, typename T>
struct specialize_len : meta::constant<usize, 0>
{};
template<template<typename...> class F, typename... Ts>
struct specialize_len<F, F<Ts...>> : meta::constant<usize, sizeof...(Ts)>
{};

template<template<typename...> class F,
         typename... Ts,
         typename Seq,
         typename... Seqs>
struct zip_type_seq<meta::true_type, F, F<Ts...>, Seq, Seqs...>
{
  using type = typename zip_type_seq2<
    F,
    F<Ts...>,
    typename zip_type_seq<meta::true_type, F, Seq, Seqs...>::type>::type;
};

template<template<typename...> class F>
struct concat_type_seq<true_type, F>
{
  using type = F<>;
};

template<template<typename...> class F, typename... Ts>
struct concat_type_seq<true_type, F, F<Ts...>>
{
  using type = F<Ts...>;
};

template<template<typename...> class F, typename... Ts, typename... Us>
struct concat_type_seq<true_type, F, F<Ts...>, F<Us...>>
{
  using type = F<Ts..., Us...>;
};

template<template<typename...> class F,
         typename... Ts,
         typename... Us,
         typename... Vs,
         typename... Seqs>
struct concat_type_seq<true_type, F, F<Ts...>, F<Us...>, F<Vs...>, Seqs...>
{
  using type = typename concat_type_seq<
    true_type,
    F,
    F<Ts..., Us..., Vs...>,
    typename concat_type_seq<true_type, F, Seqs...>::type>::type;
};
} // namespace _meta
} // namespace _detail
namespace meta {
template<template<typename... F> class F, typename... Seqs>
using type_sequence_cat = typename _detail::_meta::concat_type_seq<
  bool_constant<VEG_ALL_OF(_detail::_meta::specializes<F, Seqs>::value)>,
  F,
  Seqs...>::type;

template<template<typename...> class F, typename... Seqs>
using type_sequence_zip = typename _detail::_meta::zip_type_seq<
  meta::bool_constant<
    VEG_ALL_OF(_detail::_meta::specializes<F, Seqs>::value) &&
    VEG_CONCEPT(
      all_same<
        constant<usize, _detail::_meta::specialize_len<F, Seqs>::value>...>)>,
  F,
  Seqs...>::type;

template<template<typename...> class F, typename Seq>
using type_sequence_apply =
  typename _detail::_meta::apply_type_seq<F, Seq>::type;
} // namespace meta

namespace _detail {

template<usize I, typename T>
struct HollowLeaf
{};

template<typename ISeq, typename... Ts>
struct HollowIndexedTuple;
template<usize... Is, typename... Ts>
struct HollowIndexedTuple<meta::index_sequence<Is...>, Ts...>
  : HollowLeaf<Is, Ts>...
{};

template<usize I, typename T>
auto
get_type(HollowLeaf<I, T> const*) VEG_NOEXCEPT->T;
template<typename T, usize I>
auto
get_idx(HollowLeaf<I, T> const*) VEG_NOEXCEPT->meta::constant<usize, I>;

template<usize I>
struct pack_ith_elem
{
  template<typename... Ts>
  using Type = decltype(_detail::get_type<I>(
    static_cast<
      HollowIndexedTuple<meta::make_index_sequence<sizeof...(Ts)>, Ts...>*>(
      nullptr)));
};

template<typename T>
struct pack_idx_elem
{
  template<typename... Ts>
  using Type = decltype(_detail::get_idx<T>(
    static_cast<
      HollowIndexedTuple<meta::make_index_sequence<sizeof...(Ts)>, Ts...>*>(
      nullptr)));
};
} // namespace _detail

template<typename T, typename... Ts>
using position_of = typename _detail::pack_idx_elem<T>::template Type<Ts...>;

#if VEG_HAS_BUILTIN(__type_pack_element)
template<usize I, typename... Ts>
using ith = __type_pack_element<I, Ts...>;
#else
template<usize I, typename... Ts>
using ith = typename _detail::pack_ith_elem<I>::template Type<Ts...>;
#endif
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_INTEGER_SEQ_HPP_JBT0EKAQS */
