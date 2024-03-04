#ifndef VEG_PLACEMENT_HPP_LP0HMLTPS
#define VEG_PLACEMENT_HPP_LP0HMLTPS

#include "proxsuite/linalg/veg/type_traits/constructible.hpp"
#include "proxsuite/linalg/veg/internal/std.hpp"
#include "proxsuite/linalg/veg/type_traits/invocable.hpp"
#include "proxsuite/linalg/veg/memory/address.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"
#include <new>

#if defined(VEG_WITH_CXX20_SUPPORT) && !VEG_HAS_BUILTIN(__builtin_bit_cast)
#include <bit>
#define VEG_HAS_BITCAST 1
#define VEG_BITCAST(T, x) ::std::bit_cast<T>(x)
#define VEG_BITCAST_CONSTEXPR constexpr
#elif VEG_HAS_BUILTIN(__builtin_bit_cast)
#define VEG_HAS_BITCAST 1
#define VEG_BITCAST(T, x) __builtin_bit_cast(T, x)
#define VEG_BITCAST_CONSTEXPR constexpr
#else
#include <cstring>
#define VEG_HAS_BITCAST 0
#define VEG_BITCAST_CONSTEXPR
#endif

// construct_at
#if defined(VEG_WITH_CXX20_SUPPORT)

// std::construct_at
#if __VEG_HAS_INCLUDE(<bits / stl_construct.h>) &&                             \
  __VEG_HAS_INCLUDE(<bits / stl_iterator_base_types.h>) &&                     \
  __VEG_HAS_INCLUDE(<bits / stl_iterator_base_funcs.h>)
#include <bits/stl_iterator_base_types.h>
#include <bits/stl_iterator_base_funcs.h>
#include <bits/stl_construct.h>
#else
#include <memory>
#endif

#endif

#if VEG_HAS_BUILTIN(__builtin_launder) || __GNUC__ >= 7
#define VEG_LAUNDER(p) (__builtin_launder(p))
#elif defined(VEG_WITH_CXX17_SUPPORT)
#include <new>
#define VEG_LAUNDER(p) (::std::launder(p))
#else
#define VEG_LAUNDER(p) (+p)
#endif

namespace proxsuite {
namespace linalg {
namespace veg {
namespace mem {
namespace nb {
struct launder
{
  VEG_TEMPLATE(typename T,
               requires(VEG_CONCEPT(complete<T>)),
               VEG_INLINE constexpr auto
               operator(),
               (mem, T*))
  const VEG_NOEXCEPT->T* { return VEG_LAUNDER(mem); }
};
#undef VEG_LAUNDER

struct construct_at
{
  VEG_TEMPLATE((typename T, typename... Args),
               requires(!VEG_CONCEPT(const_type<T>) &&
                        VEG_CONCEPT(inplace_constructible<T, Args...>)),
               VEG_INLINE VEG_CPP20(constexpr) auto
               operator(),
               (mem, T*),
               (... args, Args&&))
  const VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_inplace_constructible<T, Args...>))
    ->T*
  {
#if defined(VEG_WITH_CXX20_SUPPORT)
    return ::std::construct_at(mem, VEG_FWD(args)...);
#else
    return ::new (mem) T(VEG_FWD(args)...);
#endif
  }
};

struct construct_with
{
  VEG_TEMPLATE((typename T, typename Fn),
               requires(!VEG_CONCEPT(const_type<T>) &&
                        VEG_CONCEPT(fn_once<Fn, T>)),
               VEG_INLINE VEG_CPP20(constexpr) auto
               operator(),
               (mem, T*),
               (fn, Fn&&))
  const VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_fn_once<Fn, T>))->T*
  {
#if defined(VEG_WITH_CXX20_SUPPORT)
    struct Convertor
    {
      Fn&& fn;
      VEG_INLINE constexpr operator T() const&& VEG_NOEXCEPT_IF(
        VEG_CONCEPT(nothrow_fn_once<Fn, T>))
      {
        return VEG_FWD(fn)();
      }
    };

    return ::std::construct_at(mem, Convertor{ VEG_FWD(fn) });
#else
    return ::new (mem) T(VEG_FWD(fn)());
#endif
  }
};

struct destroy_at
{
  VEG_TEMPLATE((typename T),
               requires(VEG_CONCEPT(complete<T>)),
               VEG_INLINE VEG_CPP14(constexpr) void
               operator(),
               (mem, T*))
  const VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_destructible<T>)) { mem->~T(); }
};
struct align_next
{
  auto operator()(usize alignment, void* ptr) const VEG_NOEXCEPT->void*
  {
    using byte_ptr = unsigned char*;
    std::uintptr_t lo_mask = alignment - 1U;
    std::uintptr_t hi_mask = ~lo_mask;
    auto const intptr = reinterpret_cast<std::uintptr_t>(ptr);
    auto* const byteptr = static_cast<byte_ptr>(ptr);
    auto offset = ((intptr + alignment - 1U) & hi_mask) - intptr;

    return byteptr + offset;
  }
  auto operator()(usize alignment,
                  void const* ptr) const VEG_NOEXCEPT->void const*
  {
    return this->operator()(alignment, const_cast<void*>(ptr));
  }
};
struct align_prev
{
  auto operator()(usize alignment, void* ptr) const VEG_NOEXCEPT->void*
  {
    using byte_ptr = unsigned char*;
    std::uintptr_t lo_mask = alignment - 1U;
    std::uintptr_t hi_mask = ~lo_mask;
    auto const intptr = reinterpret_cast<std::uintptr_t>(ptr);
    auto* const byteptr = static_cast<byte_ptr>(ptr);
    auto offset = ((intptr)&hi_mask) - intptr;

    return byteptr + offset;
  }
  auto operator()(usize alignment,
                  void const* ptr) const VEG_NOEXCEPT->void const*
  {
    return this->operator()(alignment, const_cast<void*>(ptr));
  }
};
} // namespace nb
VEG_NIEBLOID(align_next);
VEG_NIEBLOID(align_prev);
VEG_NIEBLOID(launder);
VEG_NIEBLOID(construct_at);
VEG_NIEBLOID(construct_with);
VEG_NIEBLOID(destroy_at);

namespace nb {
template<typename To>
struct bit_cast
{
  VEG_TEMPLATE(typename From,
               requires((VEG_CONCEPT(trivially_copyable<From>) && //
                         VEG_CONCEPT(trivially_copyable<To>) &&   //
                         (sizeof(From) == sizeof(To)))),
               VEG_INLINE VEG_BITCAST_CONSTEXPR auto
               operator(),
               (from, From const&))
  const VEG_NOEXCEPT->To
  {
#if VEG_HAS_BITCAST
    return VEG_BITCAST(To, from);
#else
    alignas(To) unsigned char buf[sizeof(To)];
    To* ptr = reinterpret_cast<To*>(static_cast<unsigned char*>(buf));
    ::std::memcpy(ptr, nb::addressof{}(from), sizeof(To));
    return To(static_cast<To&&>(*nb::launder{}(ptr)));
#endif
  }
};
} // namespace nb
VEG_NIEBLOID_TEMPLATE(typename To, bit_cast, To);
} // namespace mem
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_PLACEMENT_HPP_LP0HMLTPS */
