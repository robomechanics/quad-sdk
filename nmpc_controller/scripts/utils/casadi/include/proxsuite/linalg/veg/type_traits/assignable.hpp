#ifndef VEG_ASSIGNABLE_HPP_4ZNCRTO7S
#define VEG_ASSIGNABLE_HPP_4ZNCRTO7S

#include "proxsuite/linalg/veg/type_traits/core.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

#if !VEG_HAS_BUILTIN(__is_trivially_assignable) ||                             \
  !VEG_HAS_BUILTIN(__is_assignable) ||                                         \
  !VEG_HAS_BUILTIN(__is_nothrow_assignable)
#include <type_traits>
#endif

namespace proxsuite {
namespace linalg {
namespace veg {
namespace concepts {

VEG_DEF_CONCEPT_FROM_BUILTIN_OR_STD((typename T, typename U), assignable, T, U);
VEG_DEF_CONCEPT_FROM_BUILTIN_OR_STD((typename T, typename U),
                                    nothrow_assignable,
                                    T&&,
                                    U&&);

VEG_DEF_CONCEPT_FROM_BUILTIN_OR_TRAIT(typename T,
                                      trivially_copy_assignable,
                                      is_trivially_assignable,
                                      T&,
                                      T const&);
VEG_DEF_CONCEPT_FROM_BUILTIN_OR_TRAIT(typename T,
                                      trivially_move_assignable,
                                      is_trivially_assignable,
                                      T&,
                                      T&&);

VEG_DEF_CONCEPT(typename T, move_assignable, VEG_CONCEPT(assignable<T&, T&&>));
VEG_DEF_CONCEPT(typename T,
                nothrow_move_assignable,
                VEG_CONCEPT(nothrow_assignable<T&, T&&>));

VEG_DEF_CONCEPT(typename T,
                copy_assignable,
                VEG_CONCEPT(assignable<T&, T const&>));
VEG_DEF_CONCEPT(typename T,
                nothrow_copy_assignable,
                VEG_CONCEPT(nothrow_assignable<T&, T const&>));

} // namespace concepts
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_ASSIGNABLE_HPP_4ZNCRTO7S */
