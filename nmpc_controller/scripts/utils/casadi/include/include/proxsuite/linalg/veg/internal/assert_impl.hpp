#ifndef VEG_ASSERT_HPP_VQDAJ2IBS
#define VEG_ASSERT_HPP_VQDAJ2IBS

#include "proxsuite/linalg/veg/internal/typedefs.hpp"
#include "proxsuite/linalg/veg/util/defer.hpp"
#include "proxsuite/linalg/veg/internal/dbg.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"
#include <cassert>

#define VEG_ASSERT(...) assert((__VA_ARGS__))

#define VEG_ASSERT_ALL_OF(...)                                                 \
  assert(::proxsuite::linalg::veg::_detail::all_of({ __VA_ARGS__ }))

#define VEG_UNIMPLEMENTED()                                                    \
  VEG_ASSERT(false);                                                           \
  HEDLEY_UNREACHABLE()

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_ASSERT_HPP_VQDAJ2IBS */
