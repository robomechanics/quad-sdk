#ifndef VEG_STACK_ALLOC_HPP_UTBYR4Y5S
#define VEG_STACK_ALLOC_HPP_UTBYR4Y5S

#include "proxsuite/linalg/veg/slice.hpp"
#include "proxsuite/linalg/veg/type_traits/alloc.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {

namespace _detail {
namespace _mem {

template<usize MaxAlign>
struct BumpAllocLayout
{

  mem::byte* current_ptr;
  mem::byte* start_ptr;
  mem::byte* end_ptr;

  static auto _align(usize byte_size) noexcept -> usize
  {
    usize const actual_max_align =
#if VEG_HAS_ASAN
      MaxAlign > 8 ? MaxAlign : 8
#else
      MaxAlign
#endif
      ;

    usize const mask = (actual_max_align - 1);
    return (byte_size + mask) & ~mask;
  }

  auto _is_last(void* ptr, usize byte_size) noexcept -> bool
  {
    return ptr == (current_ptr - _align(byte_size));
  }
  void _assert_last(void* ptr, usize byte_size) noexcept
  {
    VEG_ASSERT(ptr == (current_ptr - _align(byte_size)));
  }

  void _dealloc_last_unchecked(void* ptr, mem::Layout layout)
  {
    VEG_DEBUG_ASSERT(ptr == (current_ptr - _align(layout.byte_size)));
    (void)layout;
    current_ptr = static_cast<mem::byte*>(ptr);
  }

  void _dealloc_any(void* ptr, mem::Layout layout) noexcept
  {
    if (_is_last(ptr, layout.byte_size)) {
      _dealloc_last_unchecked(ptr, layout);
    }
  }

  void _dealloc_last(void* ptr, mem::Layout layout)
  {
    _assert_last(ptr, layout.byte_size);
    _dealloc_last_unchecked(ptr, layout);
  }

  auto _alloc(mem::Layout layout) noexcept -> mem::AllocBlock
  {
    auto given_bytes = _align(layout.byte_size);
    auto rem_bytes = usize(end_ptr - current_ptr);
    VEG_ASSERT_ALL_OF( //
      (layout.align <= MaxAlign),
      (given_bytes < rem_bytes));

    mem::AllocBlock blk = { current_ptr, given_bytes };
    current_ptr += given_bytes;
    return blk;
  }

  auto _grow_last_unchecked(void* ptr, usize new_byte_size) noexcept
    -> mem::AllocBlock
  {
    auto rem_bytes = usize(end_ptr - static_cast<mem::byte*>(ptr));
    auto given_bytes = _align(new_byte_size);
    (void)rem_bytes;
    VEG_DEBUG_ASSERT(given_bytes < rem_bytes);
    current_ptr = static_cast<mem::byte*>(ptr) + given_bytes;
    return { ptr, given_bytes };
  }

  auto _grow_last(void* ptr,
                  mem::Layout old_layout,
                  usize new_byte_size,
                  mem::RelocFn /*reloc*/) noexcept -> mem::AllocBlock
  {
    if (ptr == nullptr) {
      mem::AllocBlock blk =
        _alloc(mem::Layout{ new_byte_size, old_layout.align });
      return blk;
    }
    _assert_last(ptr, old_layout.byte_size);
    return _grow_last_unchecked(ptr, new_byte_size);
  }

  auto _grow_any(void* ptr,
                 mem::Layout old_layout,
                 usize new_byte_size,
                 mem::RelocFn reloc) noexcept -> mem::AllocBlock
  {
    if (_is_last(ptr, old_layout.byte_size)) {
      return _grow_last_unchecked(ptr, new_byte_size);
    } else {
      mem::AllocBlock blk =
        _alloc(mem::Layout{ new_byte_size, old_layout.align });
      reloc(blk.data, ptr, old_layout.byte_size);
      return blk;
    }
  }
};
} // namespace _mem
} // namespace _detail

namespace mem {
template<usize MaxAlign>
struct BumpAlloc : private _detail::_mem::BumpAllocLayout<MaxAlign>
{
#if VEG_HAS_ASAN
  static_assert(MaxAlign >= 8, ".");
#endif

  BumpAlloc(FromSliceMut /*tag*/, SliceMut<byte> s) noexcept
    : _detail::_mem::BumpAllocLayout<MaxAlign>{
      s.ptr_mut(),
      s.ptr_mut(),
      s.ptr_mut() + s.len(),
    }
  {

    VEG_ASSERT_ALL_OF(
      ((std::uintptr_t(s.ptr()) % MaxAlign) == std::uintptr_t(0)),
      ((usize(s.len()) % usize(MaxAlign)) == usize(0)));
  }
};
template<usize MaxAlign>
struct StackAlloc : private BumpAlloc<MaxAlign>
{
  using BumpAlloc<MaxAlign>::BumpAlloc;
};
template<usize MaxAlign>
struct MonotonicAlloc : private BumpAlloc<MaxAlign>
{
  using BumpAlloc<MaxAlign>::BumpAlloc;
};

template<usize MaxAlign>
struct Alloc<BumpAlloc<MaxAlign>>
{
  using ImplMut = _detail::_mem::BumpAllocLayout<MaxAlign>&;
  using RefMut = proxsuite::linalg::veg::RefMut<BumpAlloc<MaxAlign>>;

  VEG_INLINE static auto alloc(RefMut ref, mem::Layout layout) noexcept
    -> AllocBlock
  {
    return ImplMut(ref.get())._alloc(layout);
  }
  VEG_INLINE static void dealloc(RefMut ref,
                                 void* ptr,
                                 mem::Layout layout) noexcept
  {
    ImplMut(ref.get())._dealloc_any(ptr, layout);
  }
  VEG_INLINE static auto grow(RefMut ref,
                              void* ptr,
                              mem::Layout old_layout,
                              usize new_byte_size,
                              RelocFn reloc) noexcept -> mem::AllocBlock
  {
    return ImplMut(ref.get())._grow_any(ptr, old_layout, new_byte_size, reloc);
  }
};

template<usize MaxAlign>
struct Alloc<StackAlloc<MaxAlign>>
{
  using ImplMut = _detail::_mem::BumpAllocLayout<MaxAlign>&;
  using RefMut = proxsuite::linalg::veg::RefMut<StackAlloc<MaxAlign>>;

  VEG_INLINE static auto alloc(RefMut ref, mem::Layout layout) noexcept
    -> AllocBlock
  {
    return ImplMut(ref.get())._alloc(layout);
  }
  VEG_INLINE static void dealloc(RefMut ref,
                                 void* ptr,
                                 mem::Layout layout) noexcept
  {
    ImplMut(ref.get())._dealloc_last(ptr, layout);
  }
  VEG_INLINE static auto grow(RefMut ref,
                              void* ptr,
                              mem::Layout old_layout,
                              usize new_byte_size,
                              RelocFn reloc) noexcept -> mem::AllocBlock
  {
    return ImplMut(ref.get())._grow_last(ptr, old_layout, new_byte_size, reloc);
  }
};

template<usize MaxAlign>
struct Alloc<MonotonicAlloc<MaxAlign>>
{
  using ImplMut = _detail::_mem::BumpAllocLayout<MaxAlign>&;
  using RefMut = proxsuite::linalg::veg::RefMut<MonotonicAlloc<MaxAlign>>;

  VEG_INLINE static auto alloc(RefMut ref, mem::Layout layout) noexcept
    -> AllocBlock
  {
    return ImplMut(ref.get())._alloc(layout);
  }
  VEG_INLINE static void dealloc(RefMut /*ref*/,
                                 void* /*ptr*/,
                                 mem::Layout /*layout*/) noexcept
  {
  }
  VEG_INLINE static auto grow(RefMut ref,
                              void* ptr,
                              mem::Layout old_layout,
                              usize new_byte_size,
                              RelocFn reloc) noexcept -> mem::AllocBlock
  {
    return ImplMut(ref.get())._grow_last(ptr, old_layout, new_byte_size, reloc);
  }
};
} // namespace mem
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_STACK_ALLOC_HPP_UTBYR4Y5S */
