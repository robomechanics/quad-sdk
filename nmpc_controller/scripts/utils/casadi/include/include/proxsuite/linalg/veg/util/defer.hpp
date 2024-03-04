#ifndef VEG_DEFER_HPP_SQPONLRGS
#define VEG_DEFER_HPP_SQPONLRGS

#include "proxsuite/linalg/veg/type_traits/constructible.hpp"
#include "proxsuite/linalg/veg/type_traits/invocable.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
template<typename Fn>
struct VEG_NODISCARD Defer
{
  Fn fn;
  constexpr Defer(Fn _fn) VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_movable<Fn>))
    : fn(VEG_FWD(_fn))
  {
  }
  Defer(Defer const&) = delete;
  Defer(Defer&&) VEG_NOEXCEPT = delete;
  auto operator=(Defer const&) -> Defer& = delete;
  auto operator=(Defer&&) VEG_NOEXCEPT->Defer& = delete;
  VEG_CPP20(constexpr)
  VEG_INLINE ~Defer()
    VEG_NOEXCEPT_IF(VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_fn_once<Fn, void>)))
  {
    VEG_FWD(fn)();
  }
};
VEG_CPP17(

  template<typename Fn> Defer(Fn) -> Defer<Fn>;

)

namespace nb {
struct defer
{
  VEG_TEMPLATE(typename Fn,
               requires(VEG_CONCEPT(fn_once<Fn, void>)),
               VEG_INLINE VEG_CPP20(constexpr) auto
               operator(),
               (fn, Fn&&))
  const VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_movable<Fn>))
    ->proxsuite::linalg::veg::Defer<Fn>
  {
    return { VEG_FWD(fn) };
  }
};
} // namespace nb
VEG_NIEBLOID(defer);
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_DEFER_HPP_SQPONLRGS */
