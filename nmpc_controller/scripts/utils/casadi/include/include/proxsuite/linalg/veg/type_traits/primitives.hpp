#ifndef VEG_PRIMITIVES_HPP_A8O6GDV1S
#define VEG_PRIMITIVES_HPP_A8O6GDV1S

#include "proxsuite/linalg/veg/type_traits/core.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

#if !VEG_HAS_BUILTIN(__is_integral) ||                                         \
  !VEG_HAS_BUILTIN(__is_floating_point) || !(VEG_HAS_BUILTIN(__is_enum))
#include <type_traits>
#endif

namespace proxsuite {
namespace linalg {
namespace veg {
namespace concepts {
namespace aux {
VEG_DEF_CONCEPT(typename T, no_wraps_around, (T(-1) <= T(0)));
VEG_DEF_CONCEPT(typename T, wraps_around, (T(-1) > T(0)));
} // namespace aux

VEG_DEF_CONCEPT(typename T,
                enum_type,
                VEG_HAS_BUILTIN_OR(__is_enum,
                                   __is_enum(T),
                                   std::is_enum<T>::value));
VEG_DEF_CONCEPT_FROM_BUILTIN_OR_STD(typename T, integral, T);
VEG_DEF_CONCEPT_FROM_BUILTIN_OR_STD(typename T, floating_point, T);
VEG_DEF_CONCEPT_DISJUNCTION(typename T,
                            arithmetic,
                            ((, integral<T>), (, floating_point<T>)));
VEG_DEF_CONCEPT_CONJUNCTION(typename T,
                            signed_integral,
                            ((, integral<T>), (aux::, no_wraps_around<T>)));
VEG_DEF_CONCEPT_CONJUNCTION(typename T,
                            unsigned_integral,
                            ((, integral<T>), (aux::, wraps_around<T>)));
} // namespace concepts
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_PRIMITIVES_HPP_A8O6GDV1S */
