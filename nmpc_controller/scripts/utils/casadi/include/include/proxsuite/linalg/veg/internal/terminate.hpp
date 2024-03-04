#ifndef VEG_TERMINATE_HPP_YMTONE4HS
#define VEG_TERMINATE_HPP_YMTONE4HS

#include "proxsuite/linalg/veg/internal/prologue.hpp"
#include <exception>

namespace proxsuite {
namespace linalg {
namespace veg {
namespace _detail {
[[noreturn]] inline void
terminate() noexcept
{
  std::terminate();
}
} // namespace _detail
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_TERMINATE_HPP_YMTONE4HS */
