#ifndef VEG_DELETE_SPECIAL_MEMBERS_HPP_2HKRCVWUS
#define VEG_DELETE_SPECIAL_MEMBERS_HPP_2HKRCVWUS

#include "proxsuite/linalg/veg/internal/prologue.hpp"
#include "proxsuite/linalg/veg/internal/typedefs.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
namespace _detail {
struct NoDefaultCtor
{
  NoDefaultCtor() = delete;
};
struct NoMoveAssign
{
  NoMoveAssign() = default;
  ~NoMoveAssign() = default;
  NoMoveAssign(NoMoveAssign const&) = default;
  NoMoveAssign(NoMoveAssign&&) = default;
  auto operator=(NoMoveAssign const&) -> NoMoveAssign& = default;
  auto operator=(NoMoveAssign&&) -> NoMoveAssign& = delete;
};
struct NoCopyAssign
{
  NoCopyAssign() = default;
  ~NoCopyAssign() = default;
  NoCopyAssign(NoCopyAssign const&) = default;
  NoCopyAssign(NoCopyAssign&&) = default;
  auto operator=(NoCopyAssign const&) -> NoCopyAssign& = delete;
  auto operator=(NoCopyAssign&&) -> NoCopyAssign& = default;
};
struct NoMoveCtor
{
  NoMoveCtor() = default;
  ~NoMoveCtor() = default;
  NoMoveCtor(NoMoveCtor const&) = default;
  NoMoveCtor(NoMoveCtor&&) = delete;
  auto operator=(NoMoveCtor const&) -> NoMoveCtor& = default;
  auto operator=(NoMoveCtor&&) -> NoMoveCtor& = default;
};
struct NoCopyCtor
{
  NoCopyCtor() = default;
  ~NoCopyCtor() = default;
  NoCopyCtor(NoCopyCtor const&) = delete;
  NoCopyCtor(NoCopyCtor&&) = default;
  auto operator=(NoCopyCtor const&) -> NoCopyCtor& = default;
  auto operator=(NoCopyCtor&&) -> NoCopyCtor& = default;
};

struct NoMove
{
  NoMove() = default;
  ~NoMove() = default;
  NoMove(NoMove const&) = default;
  NoMove(NoMove&&) = delete;
  auto operator=(NoMove const&) -> NoMove& = default;
  auto operator=(NoMove&&) -> NoMove& = delete;
};
struct NoCopy
{
  NoCopy() = default;
  ~NoCopy() = default;
  NoCopy(NoCopy const&) = delete;
  NoCopy(NoCopy&&) = default;
  auto operator=(NoCopy const&) -> NoCopy& = delete;
  auto operator=(NoCopy&&) -> NoCopy& = default;
};

} // namespace _detail
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_DELETE_SPECIAL_MEMBERS_HPP_2HKRCVWUS */
