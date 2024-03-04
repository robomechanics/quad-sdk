#ifndef VEG_INVOCABLE_HPP_GVSWRKAYS
#define VEG_INVOCABLE_HPP_GVSWRKAYS

#include "proxsuite/linalg/veg/type_traits/core.hpp"
#include "proxsuite/linalg/veg/type_traits/constructible.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
namespace _detail {
namespace _meta {
template<typename Fn, typename... Args>
using call_expr = decltype(VEG_DECLVAL(Fn &&)(VEG_DECLVAL(Args &&)...));
} // namespace _meta
} // namespace _detail
namespace meta {
template<typename Fn, typename... Args>
using invoke_result_t =
  meta::detected_t<_detail::_meta::call_expr, Fn&&, Args&&...>;
} // namespace meta

namespace concepts {
VEG_CONCEPT_EXPR((typename Fn, typename Ret, typename... Args),
                 (Fn, Ret, Args...),
                 fn_once,
                 VEG_DECLVAL(Fn&&)(VEG_DECLVAL(Args&&)...),
                 VEG_CONCEPT(same<ExprType, Ret>));

VEG_CONCEPT_EXPR((typename Fn, typename Ret, typename... Args),
                 (Fn, Ret, Args...),
                 fn_mut,
                 VEG_DECLVAL(Fn&)(VEG_DECLVAL(Args&&)...),
                 VEG_CONCEPT(same<ExprType, Ret>));

VEG_CONCEPT_EXPR((typename Fn, typename Ret, typename... Args),
                 (Fn, Ret, Args...),
                 fn,
                 VEG_DECLVAL(Fn const&)(VEG_DECLVAL(Args&&)...),
                 VEG_CONCEPT(same<ExprType, Ret>));
} // namespace concepts
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_INVOCABLE_HPP_GVSWRKAYS */
