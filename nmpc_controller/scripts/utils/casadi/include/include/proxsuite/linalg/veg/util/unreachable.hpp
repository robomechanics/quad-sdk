#ifndef VEG_UNREACHABLE_HPP_JNCM31VSS
#define VEG_UNREACHABLE_HPP_JNCM31VSS

#include "proxsuite/linalg/veg/internal/macros.hpp"
#include "proxsuite/linalg/veg/internal/terminate.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
namespace meta {
namespace nb {
struct unreachable
{
  [[noreturn]] VEG_INLINE void operator()() const VEG_NOEXCEPT
  {
#ifdef NDEBUG
    HEDLEY_UNREACHABLE();
#else
    _detail::terminate();
#endif
  }
};

struct unreachable_if
{
  VEG_INLINE constexpr auto operator()(bool Cond) const VEG_NOEXCEPT->bool
  {
    return (Cond ? unreachable{}() : (void)0), Cond;
  }
};
} // namespace nb
VEG_NIEBLOID(unreachable);
VEG_NIEBLOID(unreachable_if);
} // namespace meta
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_UNREACHABLE_HPP_JNCM31VSS */
