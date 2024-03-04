#ifndef VEG_REF_HPP_VQCOSD7RS
#define VEG_REF_HPP_VQCOSD7RS

#include "proxsuite/linalg/veg/type_traits/tags.hpp"
#include "proxsuite/linalg/veg/type_traits/constructible.hpp"
#include "proxsuite/linalg/veg/memory/address.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
namespace cmp {
namespace ref {
struct RefBase
{};
} // namespace ref
namespace mut {
struct RefMutBase
{};
} // namespace mut
} // namespace cmp

namespace nb {
struct ref;
struct mut;
} // namespace nb

template<typename T>
struct Ref : cmp::ref::RefBase
{
private:
  T const* ptr{};
  VEG_INLINE constexpr Ref() = default;
  VEG_INLINE constexpr Ref(T const* p) noexcept
    : ptr{ p }
  {
  }
  friend struct nb::ref;

public:
  VEG_NODISCARD VEG_INLINE constexpr auto get() const noexcept -> T const&
  {
    return *ptr;
  }
  VEG_NODISCARD VEG_INLINE constexpr auto operator*() const noexcept -> T const&
  {
    return *ptr;
  }
  VEG_NODISCARD VEG_INLINE constexpr auto operator->() const noexcept
    -> T const*
  {
    return ptr;
  }
};
namespace nb {
struct ref
{
  template<typename T>
  constexpr auto operator()(T const& r) const noexcept -> Ref<T>
  {
    return mem::addressof(r);
  }
};
} // namespace nb

template<typename T>
struct RefMut : cmp::mut::RefMutBase
{
private:
  T* ptr;
  VEG_INLINE constexpr RefMut() = default;
  VEG_INLINE constexpr RefMut(T* p) noexcept
    : ptr{ p }
  {
  }

  friend struct nb::mut;

public:
  VEG_INLINE constexpr auto as_const() const noexcept -> Ref<T>
  {
    return nb::ref{}(*ptr);
  }
  VEG_NODISCARD VEG_INLINE constexpr auto get() const noexcept -> T&
  {
    return *ptr;
  }
  VEG_NODISCARD VEG_INLINE constexpr auto operator*() const noexcept -> T&
  {
    return *ptr;
  }
  VEG_NODISCARD VEG_INLINE constexpr auto operator->() const noexcept -> T*
  {
    return ptr;
  }
};

namespace nb {
struct deref
{
  template<typename T>
  constexpr auto operator()(Ref<T> r) const noexcept -> T const&
  {
    return r.get();
  }
};
struct deref_mut
{
  template<typename T>
  constexpr auto operator()(RefMut<T> r) const noexcept -> T&
  {
    return r.get();
  }
};

struct mut
{
  VEG_TEMPLATE(typename T,
               requires(!VEG_CONCEPT(const_type<meta::unref_t<T>>)),
               constexpr auto
               operator(),
               (r, T&&))
  const noexcept -> RefMut<meta::uncvref_t<T>> { return mem::addressof(r); }
};
struct clone
{
  VEG_TEMPLATE(typename T,
               requires(VEG_CONCEPT(copyable<T>)),
               VEG_INLINE constexpr auto
               operator(),
               (arg, Ref<T>))
  const VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_copyable<T>))->T
  {
    return T(arg.get());
  }
};
} // namespace nb
VEG_NIEBLOID(clone);
VEG_NIEBLOID(deref);
VEG_NIEBLOID(deref_mut);
VEG_NIEBLOID(ref);
VEG_NIEBLOID(mut);
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_REF_HPP_VQCOSD7RS */
