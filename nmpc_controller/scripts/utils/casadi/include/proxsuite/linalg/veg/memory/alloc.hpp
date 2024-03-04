#ifndef VEG_ALLOC_HPP_TAWYRUICS
#define VEG_ALLOC_HPP_TAWYRUICS

#include "proxsuite/fwd.hpp"
#include "proxsuite/linalg/veg/ref.hpp"
#include "proxsuite/linalg/veg/type_traits/constructible.hpp"
#include "proxsuite/linalg/veg/type_traits/assignable.hpp"
#include "proxsuite/linalg/veg/internal/typedefs.hpp"
#include "proxsuite/linalg/veg/internal/macros.hpp"
#include "proxsuite/linalg/veg/memory/placement.hpp"
#include "proxsuite/linalg/veg/type_traits/alloc.hpp"

#include <cstddef> // std::max_align_t
#include <cstdlib> // std::{malloc, free, realloc}, ::{aligned_alloc, free}
#ifndef __APPLE__
#include <malloc.h> // ::malloc_usable_size
#else
#include <AvailabilityMacros.h>
#include <malloc/malloc.h>
#define malloc_usable_size malloc_size
#endif
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {

#ifdef __APPLE__
namespace alignment {

#if MAC_OS_X_VERSION_MIN_REQUIRED >= 101500 &&                                 \
  (defined(_LIBCPP_HAS_ALIGNED_ALLOC) || defined(_LIBCPP_HAS_C11_FEATURES)) && \
  defined(PROXSUITE_WITH_CPP_17)
VEG_INLINE void*
aligned_alloc(std::size_t alignment, std::size_t size)
{
  return std::aligned_alloc(alignment, size);
}
#elif MAC_OS_X_VERSION_MIN_REQUIRED >= 1090
VEG_INLINE void*
aligned_alloc(std::size_t alignment, std::size_t size)
{
  if (alignment < sizeof(void*)) {
    alignment = sizeof(void*);
  }
  void* p;
  if (::posix_memalign(&p, alignment, size) != 0) {
    p = 0;
  }
  return p;
}
#endif

} // namespace alignment
#endif

// Support aligned_alloc for c++14, code from boost/align.
namespace alignment {
namespace detail {
// Source: https://www.boost.org/doc/libs/1_65_0/boost/align/detail/min_size.hpp
template<std::size_t A, std::size_t B>
struct min_size : std::integral_constant<std::size_t, (A < B) ? A : B>
{};

template<class T>
struct offset_value
{
  char value;
  T object;
};

// Source:
// https://www.boost.org/doc/libs/1_65_0/boost/align/detail/alignment_of.hpp
template<class T>
struct alignment_of : min_size<sizeof(T), sizeof(offset_value<T>) - sizeof(T)>
{};

// Source:
// https://www.boost.org/doc/libs/1_65_0/boost/align/detail/is_alignment.hpp
constexpr inline bool
is_alignment(std::size_t value)
{
  return (value > 0) && ((value & (value - 1)) == 0);
}

// Source: https://www.boost.org/doc/libs/1_74_0/boost/align/detail/align.hpp
inline void*
align(std::size_t alignment, std::size_t size, void*& ptr, std::size_t& space)
{
  assert(is_alignment(alignment));
  char* p = reinterpret_cast<char*>(
    ~(alignment - 1) & (reinterpret_cast<std::size_t>(ptr) + alignment - 1));
  assert((p - static_cast<char*>(ptr)) >= 0);
  std::size_t n = std::size_t(p - static_cast<char*>(ptr));
  if (size + n <= space) {
    ptr = p;
    space -= n;
    return p;
  }
  return 0;
}

// Source:
// https://www.boost.org/doc/libs/1_65_0/boost/align/detail/aligned_alloc.hpp
inline void*
aligned_alloc(std::size_t alignment, std::size_t size)
{
  assert(is_alignment(alignment));
  enum
  {
    N = alignment_of<void*>::value
  };
  if (alignment < N) {
    alignment = N;
  }
  std::size_t n = size + alignment - N;
  void* p = std::malloc(sizeof(void*) + n);
  if (p) {
    void* r = static_cast<char*>(p) + sizeof(void*);
    (void)align(alignment, size, r, n);
    *(static_cast<void**>(r) - 1) = p;
    p = r;
  }
  return p;
}
} // namespace detail
} // namespace alignment

namespace mem {
enum struct CopyAvailable
{
  no,
  yes_maythrow,
  yes_nothrow,
};
enum struct DtorAvailable
{
  no,
  yes_maythrow,
  yes_nothrow,
};
template<typename T>
struct CopyAvailableFor
  : meta::constant<mem::CopyAvailable,
                   (VEG_CONCEPT(nothrow_copyable<T>) &&
                    VEG_CONCEPT(nothrow_copy_assignable<T>))
                     ? CopyAvailable::yes_nothrow
                   : (VEG_CONCEPT(copyable<T>) &&
                      VEG_CONCEPT(copy_assignable<T>))
                     ? CopyAvailable::yes_maythrow
                     : CopyAvailable::no>
{};
template<typename T>
struct DtorAvailableFor
  : meta::constant<mem::DtorAvailable,
                   VEG_CONCEPT(nothrow_destructible<T>)
                     ? DtorAvailable::yes_nothrow
                     : DtorAvailable::yes_maythrow>
{};

VEG_INLINE auto
aligned_alloc(usize align, usize size) noexcept -> void*
{
  usize const mask = align - 1;
#if defined(_WIN32)
  return _aligned_malloc((size + mask) & ~mask, align);
#elif defined(__APPLE__)
#ifdef PROXSUITE_WITH_CPP_17
  return alignment::aligned_alloc(align, (size + mask) & ~mask);
#else
  return alignment::detail::aligned_alloc(align, (size + mask) & ~mask);
#endif
#else
#ifdef PROXSUITE_WITH_CPP_17
  return std::aligned_alloc(align, (size + mask) & ~mask);
#else
  return alignment::detail::aligned_alloc(align, (size + mask) & ~mask);
#endif
#endif
}

VEG_INLINE void
aligned_free(usize /*align*/, void* ptr) noexcept
{
#ifndef _WIN32
  std::free(ptr);
#else
  _aligned_free(ptr);
#endif
}

struct SystemAlloc
{
  constexpr friend auto operator==(SystemAlloc /*unused*/,
                                   SystemAlloc /*unused*/) noexcept -> bool
  {
    return true;
  }
};
template<>
struct Alloc<SystemAlloc>
{
  static constexpr usize max_base_align = alignof(std::max_align_t);

  VEG_INLINE static void dealloc(RefMut<SystemAlloc> /*alloc*/,
                                 void* ptr,
                                 Layout layout) noexcept
  {
    (layout.align <= max_base_align) ? std::free(ptr)
                                     : mem::aligned_free(layout.align, ptr);
  }
  VEG_NODISCARD VEG_INLINE static auto alloc(RefMut<SystemAlloc> /*alloc*/,
                                             Layout layout) noexcept
    -> mem::AllocBlock
  {
    void* ptr = (layout.align <= max_base_align)
                  ? std::malloc(layout.byte_size)
                  : mem::aligned_alloc(layout.align, layout.byte_size);
    if (HEDLEY_UNLIKELY(ptr == nullptr)) {
      _detail::terminate();
    }
#ifndef _WIN32
    return { ptr, ::malloc_usable_size(ptr) };
#else
    return { ptr, _msize(ptr) };
#endif
  }
  VEG_NODISCARD VEG_NO_INLINE static auto realloc(RefMut<SystemAlloc> /*alloc*/,
                                                  void* ptr,
                                                  Layout layout,
                                                  usize new_size,
                                                  usize copy_size,
                                                  RelocFn reloc) noexcept
    -> mem::AllocBlock
  {
    void* new_ptr; // NOLINT
    bool typical_align = layout.align <= max_base_align;
    bool trivial_reloc = reloc.is_trivial();
    bool use_realloc = typical_align && trivial_reloc;

    if (use_realloc) {
      new_ptr = std::realloc(ptr, new_size);
    } else {
      new_ptr = mem::aligned_alloc(layout.align, new_size);
    }

    if (HEDLEY_UNLIKELY(new_ptr == nullptr)) {
      _detail::terminate();
    }

    if (!use_realloc) {
      reloc(new_ptr, ptr, copy_size);
      mem::aligned_free(layout.align, ptr);
    }
#ifndef _WIN32
    return { new_ptr, ::malloc_usable_size(new_ptr) };
#else
    return { new_ptr, _msize(new_ptr) };
#endif
  }
  VEG_NODISCARD VEG_INLINE auto try_grow_in_place(
    void* /*ptr*/,
    Layout /*layout*/,
    usize /*new_size*/) const noexcept -> bool
  {
    return false;
  }
  VEG_NODISCARD VEG_INLINE static auto grow(RefMut<SystemAlloc> alloc,
                                            void* ptr,
                                            Layout layout,
                                            usize new_size,
                                            RelocFn reloc) noexcept
    -> mem::AllocBlock
  {
    return realloc(
      VEG_FWD(alloc), ptr, layout, new_size, layout.byte_size, reloc);
  }
  VEG_NODISCARD VEG_INLINE static auto shrink(RefMut<SystemAlloc> alloc,
                                              void* ptr,
                                              Layout layout,
                                              usize new_size,
                                              RelocFn reloc) noexcept
    -> mem::AllocBlock
  {
    return realloc(VEG_FWD(alloc), ptr, layout, new_size, new_size, reloc);
  }
};

struct DefaultCloner
{};
template<>
struct Cloner<DefaultCloner>
{
  template<typename T>
  using trivial_clone = meta::bool_constant<VEG_CONCEPT(trivially_copyable<T>)>;

  template<typename T, typename Alloc>
  VEG_INLINE static void destroy(RefMut<DefaultCloner> /*cloner*/,
                                 T* ptr,
                                 RefMut<Alloc> /*alloc*/)
    VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_destructible<T>))
  {
    mem::destroy_at(ptr);
  }
  VEG_TEMPLATE((typename T, typename Alloc),
               requires(VEG_CONCEPT(copyable<T>)),
               VEG_NODISCARD VEG_INLINE static auto clone,
               (/*cloner*/, RefMut<DefaultCloner>),
               (rhs, Ref<T>),
               (/*alloc*/, RefMut<Alloc>))
  VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_copyable<T>))->T { return T(rhs.get()); }
  VEG_TEMPLATE((typename T, typename Alloc),
               requires(VEG_CONCEPT(copyable<T>)),
               VEG_INLINE static void clone_from,
               (/*cloner*/, RefMut<DefaultCloner>),
               (lhs, RefMut<T>),
               (rhs, Ref<T>),
               (/*alloc*/, RefMut<Alloc>))
  VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_copy_assignable<T>))
  {
    lhs.get() = rhs.get();
  }
};

VEG_INLINE_VAR(system_alloc, SystemAlloc);
VEG_INLINE_VAR(default_cloner, DefaultCloner);
} // namespace mem

namespace _detail {
namespace _mem {
template<typename A>
struct ManagedAlloc /* NOLINT */
{
  void* data;
  mem::Layout layout;
  RefMut<A> alloc;

  VEG_INLINE ~ManagedAlloc()
  {
    if (data != nullptr) {
      mem::Alloc<A>::dealloc(VEG_FWD(alloc), VEG_FWD(data), VEG_FWD(layout));
    }
  }
};
} // namespace _mem
} // namespace _detail
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_ALLOC_HPP_TAWYRUICS */
