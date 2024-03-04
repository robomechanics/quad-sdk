#ifndef VEG_CONTAINER_ALGOS_HPP_SGBCMQAYS
#define VEG_CONTAINER_ALGOS_HPP_SGBCMQAYS

#include "proxsuite/linalg/veg/memory/alloc.hpp"
#include "proxsuite/linalg/veg/util/defer.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
namespace _detail {
namespace _collections {
template<bool IsNoExcept>
struct DestroyImpl;

template<>
struct DestroyImpl<true>
{
  template<typename T, typename A, typename C>
  VEG_INLINE static VEG_CPP14(constexpr) void fn(RefMut<A> alloc,
                                                 RefMut<C> cloner,
                                                 T* ptr,
                                                 T* ptr_end) VEG_NOEXCEPT
  {
    while (true) {
      if (ptr_end <= ptr) {
        break;
      }
      --ptr_end;
      mem::Cloner<C>::destroy( //
        RefMut<C>(cloner),
        static_cast<T*>(ptr_end),
        RefMut<A>(alloc));
    }
  }
};

template<typename T, typename A, typename C>
struct Cleanup
{
  RefMut<A> alloc;
  RefMut<C> cloner;
  T* ptr;
  T* ptr_end;

  VEG_CPP14(constexpr) void operator()() noexcept
  {
    DestroyImpl<true>::fn(alloc, cloner, ptr, ptr_end);
  }
};

template<>
struct DestroyImpl<false>
{
  template<typename T, typename A, typename C>
  VEG_INLINE static VEG_CPP20(constexpr) void fn(RefMut<A> alloc,
                                                 RefMut<C> cloner,
                                                 T* ptr,
                                                 T* ptr_end)
    VEG_NOEXCEPT_IF(false)
  {

    Defer<Cleanup<T, A, C>> _{ { alloc, cloner, ptr, ptr_end } };

    while (true) {
      if (_.fn.ptr_end <= _.fn.ptr) {
        break;
      }
      --_.fn.ptr_end;
      mem::Cloner<C>::destroy( //
        RefMut<C>(_.fn.cloner),
        static_cast<T*>(_.fn.ptr_end),
        RefMut<A>(_.fn.alloc));
    }
  }
};

template<typename T, typename A, typename C>
VEG_CPP14(constexpr)
void backward_destroy(RefMut<A> alloc, RefMut<C> cloner, T* ptr, T* ptr_end)
  VEG_NOEXCEPT_IF(VEG_CONCEPT(alloc::nothrow_destroy<C, T, A>))
{
  DestroyImpl<VEG_CONCEPT(alloc::nothrow_destroy<C, T, A>)>::fn(
    alloc, cloner, ptr, ptr_end);
}
} // namespace _collections
} // namespace _detail
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_CONTAINER_ALGOS_HPP_SGBCMQAYS */
