#ifndef VEG_ALLOC_HPP_QHYOV5XDS
#define VEG_ALLOC_HPP_QHYOV5XDS

#include "proxsuite/linalg/veg/internal/has_asan.hpp"
#include "proxsuite/linalg/veg/ref.hpp"
#include "proxsuite/linalg/veg/type_traits/constructible.hpp"
#include "proxsuite/linalg/veg/type_traits/assignable.hpp"
#include "proxsuite/linalg/veg/internal/typedefs.hpp"
#include "proxsuite/linalg/veg/internal/macros.hpp"
#include "proxsuite/linalg/veg/memory/placement.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"
#include <cstring>

namespace proxsuite {
namespace linalg {
namespace veg {
namespace mem {
template<typename T>
struct Alloc
{};
template<typename T>
struct Cloner
{};
} // namespace mem
namespace _detail {

#if VEG_HAS_ASAN
extern "C" void __sanitizer_annotate_contiguous_container /* NOLINT */ (
  const void* begin,
  const void* end,
  const void* old_mid,
  const void* new_mid);
#endif

[[noreturn]] HEDLEY_NEVER_INLINE void
throw_bad_alloc();
[[noreturn]] inline void
terminate() noexcept;
namespace _mem {
struct DeferUnreachable /* NOLINT */
{
  bool is_unreachable;
  VEG_INLINE ~DeferUnreachable()
  {
    if (is_unreachable) {
      HEDLEY_UNREACHABLE();
    }
  }
};
} // namespace _mem
} // namespace _detail

namespace mem {
inline auto
memmove(void* dest, void const* src, usize nbytes) noexcept -> void*
{
  if (dest != nullptr) {
    return std::memmove(dest, src, nbytes);
  } else {
    return dest;
  }
}

using byte = unsigned char;
struct AllocBlock
{
  void* data;
  usize byte_cap;
};
struct Layout
{
  usize byte_size;
  usize align;
};
struct RelocFn
{
  void* (*fn)(void*, void const*, usize);

  VEG_INLINE void operator()(void* dst, void* src, usize n) const noexcept
  {
    _detail::_mem::DeferUnreachable _{ true };
    (*fn)(dst, src, n);
    _.is_unreachable = false;
  }
  VEG_INLINE auto is_trivial() const noexcept -> bool
  {
    return fn == &mem::memmove;
  }
};
} // namespace mem
namespace concepts {
namespace alloc {
VEG_CONCEPT_EXPR((typename A),
                 (A),
                 dealloc,
                 mem::Alloc<A>::dealloc( //
                   VEG_DECLVAL(RefMut<A>),
                   VEG_DECLVAL(void*),
                   VEG_DECLVAL(mem::Layout)),
                 true);

VEG_CONCEPT_EXPR((typename A),
                 (A),
                 alloc,
                 mem::Alloc<A>::alloc(VEG_DECLVAL(RefMut<A>),
                                      VEG_DECLVAL(mem::Layout)),
                 VEG_CONCEPT(same<ExprType, mem::AllocBlock>));

VEG_CONCEPT_EXPR((typename A),
                 (A),
                 owns_alloc,
                 mem::Alloc<A>::owns(VEG_DECLVAL(Ref<A>),
                                     VEG_DECLVAL(mem::Layout)),
                 VEG_CONCEPT(same<ExprType, bool>));

VEG_CONCEPT_EXPR((typename A),
                 (A),
                 grow,
                 mem::Alloc<A>::grow(VEG_DECLVAL(RefMut<A>),
                                     VEG_DECLVAL(void*),
                                     VEG_DECLVAL(mem::Layout),
                                     VEG_DECLVAL(usize),
                                     VEG_DECLVAL(mem::RelocFn)),
                 VEG_CONCEPT(same<ExprType, mem::AllocBlock>));

VEG_CONCEPT_EXPR((typename A),
                 (A),
                 shrink,
                 mem::Alloc<A>::shrink(VEG_DECLVAL(RefMut<A>),
                                       VEG_DECLVAL(void*),
                                       VEG_DECLVAL(mem::Layout),
                                       VEG_DECLVAL(usize),
                                       VEG_DECLVAL(mem::RelocFn)),
                 VEG_CONCEPT(same<ExprType, mem::AllocBlock>));

VEG_CONCEPT_EXPR((typename C, typename T, typename A),
                 (C, T, A),
                 destroy,
                 mem::Cloner<C>::destroy( //
                   VEG_DECLVAL(RefMut<C>),
                   VEG_DECLVAL(T*),
                   VEG_DECLVAL(RefMut<A>)),
                 true);

VEG_CONCEPT_EXPR((typename C, typename T, typename A),
                 (C, T, A),
                 clone,
                 mem::Cloner<C>::clone( //
                   VEG_DECLVAL(RefMut<C>),
                   VEG_DECLVAL(Ref<T>),
                   VEG_DECLVAL(RefMut<A>)),
                 VEG_CONCEPT(same<ExprType, T>));

VEG_CONCEPT_EXPR((typename C, typename T, typename A),
                 (C, T, A),
                 clone_from,
                 mem::Cloner<C>::clone_from(VEG_DECLVAL(RefMut<C>),
                                            VEG_DECLVAL(RefMut<T>),
                                            VEG_DECLVAL(Ref<T>),
                                            VEG_DECLVAL(RefMut<A>)),
                 true);
} // namespace alloc
} // namespace concepts
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_ALLOC_HPP_QHYOV5XDS */
