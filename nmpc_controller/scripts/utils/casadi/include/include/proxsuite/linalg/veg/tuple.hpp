#ifndef VEG_TUPLE_HPP_B8PHUNWES
#define VEG_TUPLE_HPP_B8PHUNWES

#include "proxsuite/linalg/veg/type_traits/assignable.hpp"
#include "proxsuite/linalg/veg/internal/dbg.hpp"
#include "proxsuite/linalg/veg/type_traits/invocable.hpp"
#include "proxsuite/linalg/veg/util/get.hpp"
#include "proxsuite/linalg/veg/internal/fix_index.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

#if defined(__GLIBCXX__)
namespace std /* NOLINT */ {
_GLIBCXX_BEGIN_NAMESPACE_VERSION
template<typename T>
struct tuple_size;
template<::proxsuite::linalg::veg::usize, typename T>
struct tuple_element;
_GLIBCXX_END_NAMESPACE_VERSION
} // namespace std
#else
#include <utility> // std::tuple_{size,element}
#endif

/******************************************************************************/
#define __VEG_IMPL_BIND(I, Tuple, Identifier) /* NOLINT */                     \
  auto&& Identifier /* NOLINT */ =                                             \
    ::proxsuite::linalg::veg::nb::get<I>{}(VEG_FWD(Tuple));

#define __VEG_IMPL_BIND_ID_SEQ(/* NOLINT */                                    \
                               CV_Auto,                                        \
                               Identifiers,                                    \
                               Tuple,                                          \
                               Tuple_Size,                                     \
                               TupleId)                                        \
  CV_Auto TupleId = Tuple;                                                     \
  static_assert(                                                               \
    ::std::tuple_size<typename ::proxsuite::linalg::veg::meta::uncvref_t<      \
        decltype(TupleId)>>::value == (Tuple_Size),                            \
    "wrong number of identifiers");                                            \
  __VEG_PP_TUPLE_FOR_EACH_I(__VEG_IMPL_BIND, TupleId, Identifiers)             \
  VEG_NOM_SEMICOLON

// example: difference vs c++17 structure bindings
// auto get() -> tuple<A, B&, C&&>;
//
// auto [a, b, c] = get();
// VEG_BIND(auto, (x, y, z), get());
// decltype({a,b,c}) => {A,B&,C&&}     same as tuple_element<i, E>
// decltype({x,y,z}) => {A&&,B&,C&&}   always a reference, lvalue if initializer
//                                     expression or tuple_element<i, E> is an
//                                     lvalue, rvalue otherwise.
//
#define VEG_BIND(CV_Auto, Identifiers, Tuple)                                  \
  __VEG_IMPL_BIND_ID_SEQ(CV_Auto,                                              \
                         Identifiers,                                          \
                         Tuple,                                                \
                         __VEG_PP_TUPLE_SIZE(Identifiers),                     \
                         __VEG_PP_CAT(_dummy_tuple_variable_id_, __LINE__))
/******************************************************************************/

namespace proxsuite {
namespace linalg {
namespace veg {
template<typename T, usize I>
using inner_ith = decltype(VEG_DECLVAL(T)[Fix<isize{ I }>{}]);

template<typename... Ts>
struct Tuple;

namespace tuple {
namespace nb {
struct tuplify
{
  template<typename... Ts>
  VEG_NODISCARD VEG_INLINE constexpr auto operator()(Ts... args) const
    VEG_NOEXCEPT->proxsuite::linalg::veg::Tuple<Ts...>
  {
    return { tuplify{}, Ts(VEG_FWD(args))... };
  }
};
} // namespace nb
VEG_NIEBLOID(tuplify);
} // namespace tuple

inline namespace tags {
using Tuplify = tuple::nb::tuplify;
using tuple::tuplify;
} // namespace tags

namespace tuple {
using proxsuite::linalg::veg::Tuple;
} // namespace tuple

namespace tuple {
template<typename ISeq, typename... Ts>
struct IndexedTuple
{};

#if VEG_HAS_NO_UNIQUE_ADDRESS
#define __VEG_IMPL_LEAF(Tuple, I, ...) /* NOLINT */                            \
  (static_cast<                                                                \
     ::proxsuite::linalg::veg::tuple::TupleLeaf<I, __VA_ARGS__> const&>(       \
     (Tuple).inner)                                                            \
     .leaf)
#define __VEG_IMPL_LEAF_MUT(Tuple, I, ...) /* NOLINT */                        \
  (static_cast<::proxsuite::linalg::veg::tuple::TupleLeaf<I, __VA_ARGS__>&>(   \
     (Tuple).inner)                                                            \
     .leaf)
#define __VEG_IMPL_LEAF_ONCE(Tuple, I, ...) /* NOLINT */                       \
  (static_cast<__VA_ARGS__&&>(                                                 \
    static_cast<::proxsuite::linalg::veg::tuple::TupleLeaf<I, __VA_ARGS__>&&>( \
      (Tuple).inner)                                                           \
      .leaf))

template<usize I, typename T>
struct TupleLeaf
{
  VEG_NO_UNIQUE_ADDRESS T leaf;
};
#else

#define __VEG_IMPL_LEAF(Tuple, I, ...) /* NOLINT */                            \
  (static_cast<__VA_ARGS__ const&>(                                            \
    static_cast<                                                               \
      ::proxsuite::linalg::veg::tuple::TupleLeaf<I, __VA_ARGS__> const&>(      \
      (Tuple).inner)                                                           \
      .leaf_get()))

#define __VEG_IMPL_LEAF_MUT(Tuple, I, ...) /* NOLINT */                        \
  (static_cast<::proxsuite::linalg::veg::tuple::TupleLeaf<I, __VA_ARGS__>&>(   \
     (Tuple).inner)                                                            \
     .leaf_get())

#define __VEG_IMPL_LEAF_ONCE(Tuple, I, ...) /* NOLINT */                       \
  (static_cast<__VA_ARGS__&&>(                                                 \
    static_cast<::proxsuite::linalg::veg::tuple::TupleLeaf<I, __VA_ARGS__>&&>( \
      (Tuple).inner)                                                           \
      .leaf_get()))

template<typename T, bool = (VEG_CONCEPT(empty<T>) && !VEG_CONCEPT(final<T>))>
struct TupleLeafImpl;

template<typename T>
struct TupleLeafImpl<T, true> : T
{
  template<typename Fn>
  VEG_INLINE constexpr TupleLeafImpl(InPlace<void> /*tag*/, Fn fn)
    VEG_NOEXCEPT_LIKE(VEG_FWD(fn)())
    : T{ VEG_FWD(fn)() }
  {
  }
  TupleLeafImpl() = default;
  VEG_INLINE constexpr auto leaf_get() const VEG_NOEXCEPT->T&
  {
    return const_cast<T&>(static_cast<T const&>(*this));
  }
};
template<typename T>
struct TupleLeafImpl<T, false>
{
  T leaf;

  template<typename Fn>
  VEG_INLINE constexpr TupleLeafImpl(InPlace<void> /*tag*/, Fn fn)
    VEG_NOEXCEPT_LIKE(VEG_FWD(fn)())
    : leaf{ VEG_FWD(fn)() }
  {
  }
  TupleLeafImpl() = default;

  VEG_INLINE constexpr auto leaf_get() const VEG_NOEXCEPT->T&
  {
    return const_cast<T&>(leaf);
  }
};

template<usize I, typename T>
struct TupleLeaf : TupleLeafImpl<T>
{
  using TupleLeafImpl<T>::TupleLeafImpl;
};
#endif

namespace nb {
struct unpack
{
  VEG_TEMPLATE(
    (typename Fn, typename... Ts, usize... Is),
    requires(VEG_CONCEPT(
      fn_once<Fn,
              proxsuite::linalg::veg::meta::invoke_result_t<Fn, Ts&&...>,
              Ts&&...>)),
    VEG_INLINE constexpr auto
    operator(),
    (args,
     IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                  Ts...>&&),
    (fn, Fn))
  const VEG_NOEXCEPT_IF(
    VEG_CONCEPT(nothrow_fn_once<
                Fn,
                proxsuite::linalg::veg::meta::invoke_result_t<Fn, Ts&&...>,
                Ts&&...>))
    ->proxsuite::linalg::veg::meta::invoke_result_t<Fn, Ts&&...>
  {

    return VEG_FWD(fn)(__VEG_IMPL_LEAF_ONCE(args, Is, Ts)...);
  }
};

struct for_each_i
{
  VEG_TEMPLATE(
    (typename Fn, typename... Ts, usize... Is),
    requires(VEG_ALL_OF(VEG_CONCEPT(fn_once<inner_ith<Fn&, Is>, void, Ts>))),
    VEG_INLINE VEG_CPP14(constexpr) void
    operator(),
    (args,
     IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                  Ts...>&&),
    (fn, Fn))
  const VEG_NOEXCEPT_IF(
    VEG_ALL_OF(VEG_CONCEPT(nothrow_fn_once<inner_ith<Fn, Is>, void, Ts>)))
  {
    VEG_EVAL_ALL(fn[Fix<isize{ Is }>{}](__VEG_IMPL_LEAF_ONCE(args, Is, Ts)));
  }
};

struct for_each
{
  VEG_TEMPLATE(
    (typename Fn, typename... Ts, usize... Is),
    requires(VEG_ALL_OF(VEG_CONCEPT(fn_mut<Fn, void, Ts&&>))),
    VEG_INLINE VEG_CPP14(constexpr) void
    operator(),
    (args,
     IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                  Ts...>&&),
    (fn, Fn))
  const VEG_NOEXCEPT_IF(VEG_ALL_OF(VEG_CONCEPT(nothrow_fn_mut<Fn, void, Ts&&>)))
  {
    VEG_EVAL_ALL(fn(__VEG_IMPL_LEAF_ONCE(args, Is, Ts)));
  }
};

struct map_i
{
  VEG_TEMPLATE(
    (typename Fn, typename... Ts, usize... Is),
    requires(VEG_ALL_OF(VEG_CONCEPT(
      fn_once< //
        inner_ith<Fn&, Is>,
        proxsuite::linalg::veg::meta::invoke_result_t<inner_ith<Fn&, Is>, Ts>,
        Ts>))),
    VEG_NODISCARD VEG_INLINE VEG_CPP14(constexpr) auto
    operator(),
    (args,
     IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                  Ts...>&&),
    (fn, Fn))
  const VEG_NOEXCEPT_IF(
    VEG_ALL_OF(VEG_CONCEPT(
      nothrow_fn_once< //
        inner_ith<Fn&, Is>,
        proxsuite::linalg::veg::meta::invoke_result_t<inner_ith<Fn&, Is>, Ts>,
        Ts>)))
    ->Tuple<
      proxsuite::linalg::veg::meta::invoke_result_t<inner_ith<Fn&, Is>, Ts>...>
  {
    return { inplace[tuplify{}],
             _detail::WithArg<inner_ith<Fn&, Is>, Ts>{
               fn[Fix<isize{ Is }>{}],
               __VEG_IMPL_LEAF_ONCE(args, Is, Ts) }... };
  }
};

struct map
{
  VEG_TEMPLATE(
    (typename Fn, typename... Ts, usize... Is),
    requires(VEG_ALL_OF(VEG_CONCEPT(
      fn_mut<Fn,
             proxsuite::linalg::veg::meta::invoke_result_t<Fn&, Ts&&>,
             Ts&&>))),
    VEG_NODISCARD VEG_INLINE VEG_CPP14(constexpr) auto
    operator(),
    (args,
     IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                  Ts...>&&),
    (fn, Fn))
  const VEG_NOEXCEPT_IF(
    VEG_ALL_OF(VEG_CONCEPT(
      nothrow_fn_mut<Fn,
                     proxsuite::linalg::veg::meta::invoke_result_t<Fn&, Ts&&>,
                     Ts&&>)))
    ->Tuple<proxsuite::linalg::veg::meta::invoke_result_t<Fn&, Ts&&>...>
  {
    return {
      inplace[tuplify{}],
      _detail::WithArg<Fn&, Ts&&>{
        fn,
        __VEG_IMPL_LEAF_ONCE(args, Is, Ts),
      }...,
    };
  }
};
} // namespace nb

template<usize... Is, typename... Ts>
struct IndexedTuple<meta::index_sequence<Is...>, Ts...>
{
  struct _ : TupleLeaf<Is, Ts>...
  {
#if !defined(VEG_WITH_CXX17_SUPPORT)
#if VEG_HAS_NO_UNIQUE_ADDRESS
    template<typename... Fns>
    VEG_INLINE constexpr _(InPlace<void> /* unused */, Fns... fns) noexcept(
      VEG_ALL_OF(VEG_CONCEPT(nothrow_fn_once<Fns, Ts>)))
      : TupleLeaf<Is, Ts>{ VEG_FWD(fns)() }...
    {
    }
#else
    template<typename... Fns>
    VEG_INLINE constexpr _(InPlace<void> /*unused*/, Fns... fns) noexcept(
      VEG_ALL_OF(VEG_CONCEPT(nothrow_fn_once<Fns, Ts>)))
      : TupleLeaf<Is, Ts>{ inplace, VEG_FWD(fns) }...
    {
    }
#endif
    _() = default;
#endif
  } inner;

  IndexedTuple() = default;

  VEG_INLINE constexpr IndexedTuple(Tuplify /*tag*/, Ts... args) VEG_NOEXCEPT
    : inner
  {
#if !defined(VEG_WITH_CXX17_SUPPORT)
    inplace, _detail::MoveFn<Ts>{ VEG_FWD(args) }...,
#else
#if VEG_HAS_NO_UNIQUE_ADDRESS
    TupleLeaf<Is, Ts>{ Ts(VEG_FWD(args)) }...
#else
    TupleLeaf<Is, Ts>{ inplace, _detail::MoveFn<Ts>{ VEG_FWD(args) } }...
#endif
#endif
  }
  {
  }

  VEG_TEMPLATE((typename _, typename... Fns),
               requires(VEG_CONCEPT(same<_, Tuplify>) &&
                        VEG_ALL_OF(VEG_CONCEPT(fn_once<Fns, Ts>))),
               VEG_INLINE constexpr IndexedTuple,
               (/*tag*/, InPlace<_>),
               (... fns, Fns))

  VEG_NOEXCEPT_IF(VEG_ALL_OF(VEG_CONCEPT(nothrow_fn_once<Fns, Ts>)))
    : inner
  {
#if !defined(VEG_WITH_CXX17_SUPPORT)
    inplace, VEG_FWD(fns)...
#else
#if VEG_HAS_NO_UNIQUE_ADDRESS
    TupleLeaf<Is, Ts>{ VEG_FWD(fns)() }...
#else
    TupleLeaf<Is, Ts>{ inplace, VEG_FWD(fns) }...
#endif
#endif
  }
  {
  }

  VEG_EXPLICIT_COPY(IndexedTuple);

  VEG_NODISCARD VEG_INLINE VEG_CPP14(constexpr) auto as_ref() const
    & VEG_NOEXCEPT->Tuple<Ref<Ts>...>
  {
    return {
      tuplify,
      ref(__VEG_IMPL_LEAF(*this, Is, Ts))...,
    };
  }
  VEG_NODISCARD VEG_INLINE VEG_CPP14(constexpr) auto as_mut()
    VEG_NOEXCEPT->Tuple<RefMut<Ts>...>
  {
    return {
      tuplify,
      mut(__VEG_IMPL_LEAF_MUT(*this, Is, Ts))...,
    };
  }

  template<isize I>
  void operator[](Fix<I> /*arg*/) const&& = delete;

  VEG_TEMPLATE((isize I),
               requires(static_cast<usize>(I) < sizeof...(Ts)),
               VEG_NODISCARD VEG_INLINE VEG_CPP14(constexpr) auto
               operator[],
               (/*arg*/, Fix<I>)) &&
    VEG_NOEXCEPT_IF(
      VEG_CONCEPT(nothrow_movable<ith<static_cast<usize>(I), Ts...>>))
      -> ith<static_cast<usize>(I), Ts...>
  {
    return __VEG_IMPL_LEAF_ONCE(
      *this, static_cast<usize>(I), ith<static_cast<usize>(I), Ts...>);
  }

  VEG_TEMPLATE((isize I),
               requires(static_cast<usize>(I) < sizeof...(Ts)),
               VEG_NODISCARD VEG_INLINE VEG_CPP14(constexpr) auto
               operator[],
               (/*arg*/, Fix<I>)) &
    VEG_NOEXCEPT->ith<static_cast<usize>(I), Ts...>&
  {
    return __VEG_IMPL_LEAF_MUT(
      *this, static_cast<usize>(I), ith<static_cast<usize>(I), Ts...>);
  }

  VEG_TEMPLATE((isize I),
               requires(static_cast<usize>(I) < sizeof...(Ts)),
               VEG_NODISCARD VEG_INLINE VEG_CPP14(constexpr) auto
               operator[],
               (/*arg*/, Fix<I>))
  const & VEG_NOEXCEPT->ith<static_cast<usize>(I), Ts...> const&
  {
    return __VEG_IMPL_LEAF(
      *this, static_cast<usize>(I), ith<static_cast<usize>(I), Ts...>);
  }
};
} // namespace tuple

namespace _detail {
namespace meta_ {

struct NonTupleBaseInfoImpl
{
  static constexpr bool is_tuple = false;
  static constexpr usize size = 0;
  template<usize I>
  using ith = void;
  using seq = void;
};

template<typename... Ts>
struct TupleBaseInfoImpl
{
  static constexpr bool is_tuple = true;
  static constexpr usize size = sizeof...(Ts);
  template<usize I>
  using ith = proxsuite::linalg::veg::ith<I, Ts...>;
  using seq = meta::type_sequence<Ts...>;
  using Tuple = proxsuite::linalg::veg::Tuple<Ts...>;
  using IndexedTuple = proxsuite::linalg::veg::tuple::
    IndexedTuple<meta::make_index_sequence<sizeof...(Ts)>, Ts...>;
};

struct is_tuple_helper
{
  static auto test(void*) -> NonTupleBaseInfoImpl;
  template<usize... Is, typename... Ts>
  static auto test(tuple::IndexedTuple<meta::index_sequence<Is...>, Ts...>*)
    -> TupleBaseInfoImpl<Ts...>;
};

template<typename T>
struct IndexedToTuple;

template<usize... Is, typename... Ts>
struct IndexedToTuple<tuple::IndexedTuple<meta::index_sequence<Is...>, Ts...>>
{
  using Type = Tuple<Ts...>;
};
} // namespace meta_
} // namespace _detail

namespace tuple {
namespace meta {
template<typename T>
using TupleBaseInfo =
  decltype(_detail::meta_::is_tuple_helper::test(static_cast<T*>(nullptr)));

template<typename T>
using is_tuple =
  proxsuite::linalg::veg::meta::bool_constant<TupleBaseInfo<T>::is_tuple>;
template<typename T>
using tuple_size =
  proxsuite::linalg::veg::meta::constant<usize, TupleBaseInfo<T>::size>;

template<usize I, typename T>
using tuple_element = typename TupleBaseInfo<T>::template ith<I>;

} // namespace meta
} // namespace tuple

namespace concepts {
namespace tuple {
VEG_DEF_CONCEPT(typename T,
                tuple,
                proxsuite::linalg::veg::tuple::meta::is_tuple<T>::value);
} // namespace tuple
} // namespace concepts

template<typename... Ts>
struct Tuple
  : tuple::IndexedTuple<meta::make_index_sequence<sizeof...(Ts)>, Ts...>
{

  using Indexed =
    tuple::IndexedTuple<meta::make_index_sequence<sizeof...(Ts)>, Ts...>;

  using Indexed::Indexed;

  VEG_EXPLICIT_COPY(Tuple);
};

VEG_CPP17(template<typename... Ts> Tuple(Tuplify, Ts...) -> Tuple<Ts...>;)
namespace tuple {

template<usize I, usize... Is, typename... Ts>
VEG_NODISCARD VEG_INLINE constexpr auto
get(tuple::IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                        Ts...> const& tup) VEG_NOEXCEPT->ith<I, Ts...> const&
{
  return __VEG_IMPL_LEAF(tup, I, ith<I, Ts...>);
}
template<usize I, usize... Is, typename... Ts>
VEG_NODISCARD VEG_INLINE constexpr auto
get(tuple::IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                        Ts...>& tup) VEG_NOEXCEPT->ith<I, Ts...>&
{
  return __VEG_IMPL_LEAF_MUT(tup, I, ith<I, Ts...>);
}
template<usize I, usize... Is, typename... Ts>
VEG_NODISCARD VEG_INLINE constexpr auto
get(tuple::IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                        Ts...> const&& tup) VEG_NOEXCEPT->ith<I, Ts...> const&&
{
  return static_cast<ith<I, Ts...> const&&>(
    __VEG_IMPL_LEAF(tup, I, ith<I, Ts...>));
}
template<usize I, usize... Is, typename... Ts>
VEG_NODISCARD VEG_INLINE constexpr auto
get(tuple::IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                        Ts...>&& tup) VEG_NOEXCEPT->ith<I, Ts...>&&
{
  return __VEG_IMPL_LEAF_ONCE(tup, I, ith<I, Ts...>);
}

} // namespace tuple

namespace _detail {
namespace _tuple {
template<usize... Is, typename... Ts>
VEG_INLINE static constexpr auto
tuple_fwd(tuple::IndexedTuple<meta::index_sequence<Is...>, Ts...>&& tup)
  VEG_NOEXCEPT->Tuple<Ts&&...>
{
  return {
    ((void)(tup), tuplify),
    __VEG_IMPL_LEAF_ONCE(tup, Is, Ts)...,
  };
}

} // namespace _tuple
} // namespace _detail

namespace tuple {
namespace nb {
struct with
{
  VEG_TEMPLATE(
    typename... Fns,
    requires(VEG_ALL_OF(VEG_CONCEPT(
      fn_once<Fns, proxsuite::linalg::veg::meta::invoke_result_t<Fns>>))),
    VEG_NODISCARD VEG_INLINE constexpr auto
    operator(),
    (... args, Fns))
  const VEG_NOEXCEPT_IF(
    VEG_ALL_OF(VEG_CONCEPT(
      nothrow_fn_once<Fns,
                      proxsuite::linalg::veg::meta::invoke_result_t<Fns>>)))
    ->proxsuite::linalg::veg::Tuple<
      proxsuite::linalg::veg::meta::invoke_result_t<Fns>...>
  {
    return { inplace[tuplify{}], VEG_FWD(args)... };
  }
};

struct zip
{

  template<typename... Tuples>
  using PreZip =
    proxsuite::linalg::veg::meta::type_sequence_zip<Tuple, Tuples...>;

  template<typename... Tuples>
  using Zip = proxsuite::linalg::veg::meta::
    detected_t<PreZip, typename meta::TupleBaseInfo<Tuples>::Tuple...>;

  VEG_TEMPLATE(
    (typename... Tuples),
    requires(VEG_ALL_OF(VEG_CONCEPT(tuple::tuple<Tuples>)) &&
             VEG_CONCEPT(all_same<tuple::meta::tuple_size<Tuples>...>)),
    VEG_NODISCARD VEG_INLINE constexpr auto
    operator(),
    (... tups, Tuples))
  const VEG_NOEXCEPT->Zip<Tuples...>
  {
    return zip::apply(
      static_cast<typename meta::TupleBaseInfo<Tuples>::IndexedTuple&&>(
        tups)...);
  }

private:
  template<typename... Tuples>
  VEG_INLINE static constexpr auto pre_apply(
    proxsuite::linalg::veg::meta::true_type /*unused*/,
    Tuples&&... tups) VEG_NOEXCEPT->Zip<Tuples...>
  {
    return zip::apply(VEG_FWD(tups)...);
  }
  template<typename... Tuples>
  VEG_INLINE static constexpr auto pre_apply(
    proxsuite::linalg::veg::meta::false_type /*unused*/,
    Tuples&&... tups) VEG_NOEXCEPT->Zip<Tuples...>
  {
    return zip::from_ref_to_result(
      Tag<proxsuite::linalg::veg::meta::type_sequence_zip<
        Tuple,
        typename _detail::meta_::IndexedToTuple<Tuples>::Type...>>{},
      zip::apply(_detail::_tuple::tuple_fwd(VEG_FWD(tups))...));
  }

  VEG_INLINE static auto apply() VEG_NOEXCEPT->Tuple<> { return {}; }

  template<usize I, typename T>
  struct Helper
  {
    template<typename... Ts>
    using Type = Tuple<T, meta::tuple_element<I, Ts>...>;

    template<typename... Ts>
    VEG_INLINE constexpr auto apply(Ts&&... tups) const
      VEG_NOEXCEPT->Type<Ts...>
    {
      return {
        tuplify{},
        VEG_FWD(first),
        __VEG_IMPL_LEAF_ONCE(tups, I, meta::tuple_element<I, Ts>)...,
      };
    }
    T&& first;
  };

  template<usize... Is, typename... Ts, typename... Tuples>
  VEG_INLINE static constexpr auto apply(
    IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>, Ts...>
      first,
    Tuples... rest) VEG_NOEXCEPT->Tuple< //
    typename Helper<Is, Ts>::            //
    template Type<Tuples...>...>
  {
    return {
      ((void)first, tuplify{}),
      Helper<Is, Ts>{ __VEG_IMPL_LEAF_ONCE(first, Is, Ts) }
        .template apply<Tuples...>(VEG_FWD(rest)...)...,
    };
  }

  template<typename ISeq, typename... InnerTargets>
  struct ConverterImpl;
  template<typename OuterTarget>
  struct Converter;

  template<usize... Is, typename... InnerTargets>
  struct ConverterImpl<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                       InnerTargets...>
  {
    IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                 InnerTargets&&...>&& refs;

    VEG_INLINE constexpr auto operator()() const
      && VEG_NOEXCEPT->Tuple<InnerTargets...>
    {

      return {
        inplace[tuplify{}],
        _detail::MoveFn<InnerTargets>{
          __VEG_IMPL_LEAF_ONCE(refs, Is, InnerTargets) }...,
      };
    }
  };

  template<typename... InnerTargets>
  struct Converter<Tuple<InnerTargets...>>
  {
    using Type =
      ConverterImpl<proxsuite::linalg::veg::meta::make_index_sequence<sizeof...(
                      InnerTargets)>,
                    InnerTargets...>;
  };

  template<usize... Is, typename... Tups, typename... OuterTargets>
  VEG_INLINE static constexpr auto from_ref_to_result(
    Tag<Tuple<OuterTargets...>> /*tag*/,
    IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>, Tups...>
      zipped_refs) VEG_NOEXCEPT->Tuple<OuterTargets...>
  {
    return {
      ((void)zipped_refs, inplace[tuplify{}]),
      typename Converter<OuterTargets>::Type{
        __VEG_IMPL_LEAF_ONCE(zipped_refs, Is, Tups),
      }...,
    };
  }
};

struct cat
{

  template<typename... Tuples>
  using PreConcat =
    proxsuite::linalg::veg::meta::type_sequence_cat<Tuple, Tuples...>;
  template<typename... Tuples>
  using Concat = proxsuite::linalg::veg::meta::
    detected_t<PreConcat, typename meta::TupleBaseInfo<Tuples>::Tuple...>;

  VEG_TEMPLATE((typename... Tuples),
               requires(VEG_ALL_OF(VEG_CONCEPT(tuple::tuple<Tuples>))),
               VEG_NODISCARD VEG_INLINE constexpr auto
               operator(),
               (... tups, Tuples))
  const VEG_NOEXCEPT->Concat<Tuples...>
  {
    return cat::apply(
      static_cast<typename meta::TupleBaseInfo<Tuples>::IndexedTuple&&>(
        tups)...);
  }

private:
  template<typename... Tuples>
  VEG_INLINE static constexpr auto pre_apply(
    proxsuite::linalg::veg::meta::true_type /*unused*/,
    Tuples&&... tups) VEG_NOEXCEPT->Concat<Tuples...>
  {
    return cat::apply(VEG_FWD(tups)...);
  }

  template<typename... Tuples>
  VEG_INLINE static constexpr auto pre_apply(
    proxsuite::linalg::veg::meta::false_type /*unused*/,
    Tuples&&... tups) VEG_NOEXCEPT->Concat<Tuples...>
  {
    return cat::template from_ref_to_result(
      Tag<proxsuite::linalg::veg::meta::type_sequence_cat<Tuple, Tuples...>>{},
      cat::apply(_detail::_tuple::tuple_fwd(VEG_FWD(tups))...));
  }

  template<typename... Targets, usize... Is, typename... Refs>
  VEG_INLINE static constexpr auto from_ref_to_result(
    Tag<Tuple<Targets...>> /*tag*/,
    IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>, Refs...>
      refs) VEG_NOEXCEPT->proxsuite::linalg::veg::Tuple<Targets...>
  {
    return {
      inplace[tuplify{}],
      _detail::MoveFn<Targets>{ __VEG_IMPL_LEAF_ONCE(refs, Is, Targets) }...,
    };
  }

  VEG_INLINE static auto apply() VEG_NOEXCEPT->Tuple<> { return {}; }

  template<usize... Is, typename... Ts, typename... Tuples>
  VEG_INLINE static constexpr auto apply(
    IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>, Ts...>&&
      first,
    Tuples&&... rest)
    VEG_NOEXCEPT->proxsuite::linalg::veg::meta::type_sequence_cat<
      Tuple,
      Tuple<Ts...>,
      typename _detail::meta_::IndexedToTuple<Tuples>::Type...>
  {
    return cat::apply2(VEG_FWD(first), cat::apply(VEG_FWD(rest)...));
  }

  template<usize... Is, typename... Ts, usize... Js, typename... Us>
  VEG_INLINE static constexpr auto apply2(
    IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>, Ts...>&&
      first,
    IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Js...>, Us...>&&
      second) VEG_NOEXCEPT->Tuple<Ts..., Us...>
  {
    return {
      tuplify{},
      __VEG_IMPL_LEAF_ONCE(first, Is, Ts)...,
      __VEG_IMPL_LEAF_ONCE(second, Js, Us)...,
    };
  }
};

struct deref_assign
{
  VEG_TEMPLATE(
    (typename... Ts, typename... Us, usize... Is),
    requires(VEG_ALL_OF(VEG_CONCEPT(assignable<Ts&, Us const&>))),
    VEG_INLINE VEG_CPP14(constexpr) void
    operator(),
    (ts,
     IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                  RefMut<Ts>...>),
    (us,
     IndexedTuple<proxsuite::linalg::veg::meta::index_sequence<Is...>,
                  Ref<Us>...>))
  const VEG_NOEXCEPT_IF(
    VEG_ALL_OF(VEG_CONCEPT(nothrow_assignable<Ts&, Us const&>)))
  {
    VEG_EVAL_ALL(__VEG_IMPL_LEAF_MUT(ts, Is, RefMut<Ts>).get() =
                   __VEG_IMPL_LEAF(us, Is, Ref<Us>).get());
  }
};
} // namespace nb
VEG_NIEBLOID(with);

VEG_NIEBLOID(zip);
VEG_NIEBLOID(cat);

VEG_NIEBLOID(unpack);

VEG_NIEBLOID(for_each);
VEG_NIEBLOID(for_each_i);
VEG_NIEBLOID(map);
VEG_NIEBLOID(map_i);

VEG_NIEBLOID(deref_assign);
} // namespace tuple

namespace cpo {
template<usize... Is, typename... Ts>
struct is_trivially_relocatable<
  tuple::IndexedTuple<meta::index_sequence<Is...>, Ts...>>
  : meta::bool_constant<(VEG_ALL_OF(is_trivially_relocatable<Ts>::value))>
{};
template<usize... Is, typename... Ts>
struct is_trivially_constructible<
  tuple::IndexedTuple<meta::index_sequence<Is...>, Ts...>>
  : meta::bool_constant<(VEG_ALL_OF(is_trivially_constructible<Ts>::value))>
{};

template<typename... Ts>
struct is_trivially_relocatable<tuple::Tuple<Ts...>>
  : meta::bool_constant<(VEG_ALL_OF(is_trivially_relocatable<Ts>::value))>
{};
template<typename... Ts>
struct is_trivially_constructible<tuple::Tuple<Ts...>>
  : meta::bool_constant<(VEG_ALL_OF(is_trivially_constructible<Ts>::value))>
{};
} // namespace cpo
} // namespace veg
} // namespace linalg
} // namespace proxsuite

template<typename... Ts>
struct std::tuple_size<proxsuite::linalg::veg::Tuple<Ts...>>
  : ::proxsuite::linalg::veg::meta::constant<proxsuite::linalg::veg::usize,
                                             sizeof...(Ts)>
{};
template<proxsuite::linalg::veg::usize I, typename... Ts>
struct std::tuple_element<I, proxsuite::linalg::veg::Tuple<Ts...>>
{
  using type = proxsuite::linalg::veg::ith<I, Ts...>;
};

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_TUPLE_HPP_B8PHUNWES */
