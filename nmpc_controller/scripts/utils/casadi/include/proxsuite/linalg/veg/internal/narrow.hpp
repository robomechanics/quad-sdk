#ifndef VEG_NARROW_HPP_H0EXKJTAS
#define VEG_NARROW_HPP_H0EXKJTAS

#include "proxsuite/fwd.hpp"
#include "proxsuite/linalg/veg/util/assert.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"
#include "proxsuite/helpers/common.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
namespace nb {
template<typename To>
struct narrow
{
  VEG_TEMPLATE((typename From),
               requires VEG_CONCEPT(integral<From>) &&
                 VEG_CONCEPT(integral<To>),
               constexpr auto
               operator(),
               (from, From))
  const VEG_NOEXCEPT->To
  {
#if defined(VEG_WITH_CXX14_SUPPORT)

    To to = static_cast<To>(from);
    PROXSUITE_MAYBE_UNUSED From roundtrip_from =
      static_cast<From>(static_cast<To>(from));
    VEG_INTERNAL_ASSERT_PRECONDITION(roundtrip_from == from);
    return to;

#else
    return VEG_INTERNAL_ASSERT_PRECONDITION(
             static_cast<From>(static_cast<To>(from)) == from),
           static_cast<To>(from);
#endif
  }
};
} // namespace nb
VEG_NIEBLOID_TEMPLATE(typename To, narrow, To);
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_NARROW_HPP_H0EXKJTAS */
