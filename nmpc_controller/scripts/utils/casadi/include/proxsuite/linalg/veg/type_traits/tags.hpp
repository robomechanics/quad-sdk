#ifndef VEG_TAGS_HPP_FYDE7Q6ZS
#define VEG_TAGS_HPP_FYDE7Q6ZS

#include "proxsuite/linalg/veg/type_traits/core.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
inline namespace tags {
VEG_TAG_TEMPLATE(typename T, tag, Tag, T);
VEG_TAG(as_ref, AsRef);
VEG_TAG(as_mut, AsMut);
VEG_TAG(from, From);
VEG_TAG(from_i, FromI);
VEG_TAG(as_ref_once, AsRefOnce);

VEG_TAG(from_alloc, FromAlloc);
VEG_TAG(from_alloc_and_value, FromAllocAndValue);

VEG_TAG(from_slice, FromSlice);
VEG_TAG(from_slice_mut, FromSliceMut);
template<typename Tag>
struct InPlace
{
  InPlace() = default;
};

template<>
struct InPlace<void>
{
  InPlace() = default;
  template<typename Tag>
  VEG_INLINE constexpr auto operator[](Tag /*tag*/) const noexcept
    -> InPlace<Tag>
  {
    return InPlace<Tag>{};
  }
};
VEG_INLINE_VAR(inplace, InPlace<void>);
} // namespace tags
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_TAGS_HPP_FYDE7Q6ZS */
