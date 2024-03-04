#ifndef VEG_DYNAMIC_STACK_DYNAMIC_STACK_HPP_UBOMZFTOS
#define VEG_DYNAMIC_STACK_DYNAMIC_STACK_HPP_UBOMZFTOS

#include "proxsuite/fwd.hpp"
#include "proxsuite/linalg/veg/util/assert.hpp"
#include "proxsuite/linalg/veg/internal/collection_algo.hpp"
#include "proxsuite/linalg/veg/memory/alloc.hpp"
#include "proxsuite/linalg/veg/memory/placement.hpp"
#include "proxsuite/linalg/veg/slice.hpp"
#include "proxsuite/linalg/veg/type_traits/constructible.hpp"
#include "proxsuite/linalg/veg/memory/placement.hpp"
#include "proxsuite/linalg/veg/memory/address.hpp"
#include "proxsuite/linalg/veg/internal/narrow.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {
namespace _detail {
namespace _dynstack {
constexpr auto
max2(isize a, isize b) noexcept -> isize
{
  return (a > b) ? a : b;
}
constexpr auto
round_up_pow2(isize a, isize b) noexcept -> isize
{
  return isize((usize(a) + ~(0 - usize(b))) & (0 - usize(b)));
}
} // namespace _dynstack
} // namespace _detail
namespace dynstack {
struct StackReq
{
  isize size_bytes;
  isize align;

  constexpr friend auto operator==(StackReq a, StackReq b) noexcept -> bool
  {
    return a.size_bytes == b.size_bytes && a.align == b.align;
  }

  constexpr friend auto operator&(StackReq a, StackReq b) noexcept -> StackReq
  {
    using namespace _detail::_dynstack;
    return {
      round_up_pow2(   //
        round_up_pow2( //
          a.size_bytes,
          b.align) +
          b.size_bytes,
        (max2)(a.align, b.align)),
      (max2)(a.align, b.align),
    };
  }
  constexpr friend auto operator|(StackReq a, StackReq b) noexcept -> StackReq
  {
    using namespace _detail::_dynstack;
    return {
      (max2)( //
        round_up_pow2(a.size_bytes, max2(a.align, b.align)),
        round_up_pow2(b.size_bytes, max2(a.align, b.align))),
      (max2)(a.align, b.align),
    };
  }

  constexpr auto alloc_req() const noexcept -> isize
  {
    return size_bytes + align - 1;
  }

  template<typename T>
  static constexpr auto with_len(proxsuite::linalg::veg::Tag<T> /*tag*/,
                                 isize len) noexcept -> StackReq
  {
    return {
      isize{ sizeof(T) } * len,
      isize{ alignof(T) },
    };
  }

  static VEG_CPP14(constexpr) auto and_(Slice<StackReq> reqs) noexcept
    -> StackReq
  {
    StackReq req{ 0, 1 };
    for (isize i = 0; i < reqs.len(); ++i) {
      req = req & reqs.ptr()[i];
    }
    return req;
  }

  static VEG_CPP14(constexpr) auto or_(Slice<StackReq> reqs) noexcept
    -> StackReq
  {
    StackReq req{ 0, 1 };
    for (isize i = 0; i < reqs.len(); ++i) {
      req = req | reqs.ptr()[i];
    }
    return req;
  }
};

template<typename T>
struct DynStackArray;
template<typename T>
struct DynStackAlloc;
} // namespace dynstack

namespace _detail {
// if possible:
// aligns the pointer
// then advances it by `size` bytes, and decreases `space` by `size`
// returns the previous aligned value
//
// otherwise, if there is not enough space for aligning or advancing the
// pointer, returns nullptr and the values are left unmodified
inline auto
align_next(isize alignment, isize size, void*& ptr, isize& space)
  VEG_ALWAYS_NOEXCEPT->void*
{
  static_assert(sizeof(std::uintptr_t) >= sizeof(void*),
                "std::uintptr_t can't hold a pointer value");

  using byte_ptr = unsigned char*;

  // assert alignment is power of two
  VEG_ASSERT_ALL_OF( //
    (alignment > isize{ 0 }),
    ((u64(alignment) & (u64(alignment) - 1)) == u64(0)));

  if (space < size) {
    return nullptr;
  }

  std::uintptr_t lo_mask = usize(alignment) - 1;
  std::uintptr_t hi_mask = ~lo_mask;

  auto const intptr = reinterpret_cast<std::uintptr_t>(ptr);
  auto* const byteptr = static_cast<byte_ptr>(ptr);

  auto offset = ((intptr + usize(alignment) - 1) & hi_mask) - intptr;

  if (usize(space) - usize(size) < offset) {
    return nullptr;
  }

  void* const rv = byteptr + offset;

  ptr = byteptr + (offset + usize(size));
  space = space - (isize(offset) + (size));

  return rv;
}
} // namespace _detail

namespace _detail {
namespace _dynstack {
template<typename T, bool = VEG_CONCEPT(trivially_destructible<T>)>
struct DynStackArrayDtor
{};

template<typename T>
struct DynStackArrayDtor<T, false>
{
  DynStackArrayDtor() = default;
  DynStackArrayDtor(DynStackArrayDtor const&) = default;
  DynStackArrayDtor(DynStackArrayDtor&&) = default;
  auto operator=(DynStackArrayDtor const&) -> DynStackArrayDtor& = default;
  auto operator=(DynStackArrayDtor&&) -> DynStackArrayDtor& = default;
  VEG_INLINE ~DynStackArrayDtor()
    VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_destructible<T>))
  {
    auto& self = static_cast<dynstack::DynStackArray<T>&>(*this);
    using Base = typename dynstack::DynStackAlloc<T>::Base const&;
    proxsuite::linalg::veg::_detail::_collections::backward_destroy(
      mut(mem::SystemAlloc{}),
      mut(mem::DefaultCloner{}),
      self.ptr_mut(),
      self.ptr_mut() + Base(self).len);
  }
};

struct cleanup;
struct DynAllocBase;

struct default_init_fn
{
  template<typename T>
  auto make(void* ptr, isize len) -> T*
  {
    return ::new (ptr) T[usize(len)];
  }
};

struct zero_init_fn
{
  template<typename T>
  auto make(void* ptr, isize len) -> T*
  {
    return ::new (ptr) T[usize(len)]{};
  }
};

struct no_init_fn
{
  template<typename T>
  auto make(void* ptr, isize len) -> T*
  {
    return proxsuite::linalg::veg::mem::launder(static_cast<T*>(
      static_cast<void*>(::new (ptr) unsigned char[usize(len) * sizeof(T)])));
  }
};

} // namespace _dynstack
} // namespace _detail

namespace dynstack {
struct DynStackMut
{
public:
  DynStackMut(FromSliceMut /*tag*/, SliceMut<unsigned char> s) VEG_NOEXCEPT
    : stack_data(s.ptr_mut())
    , stack_bytes(s.len())
  {
  }

  VEG_NODISCARD
  auto remaining_bytes() const VEG_NOEXCEPT->isize
  {
    return isize(stack_bytes);
  }
  VEG_NODISCARD
  auto ptr_mut() const VEG_NOEXCEPT->void* { return stack_data; }
  VEG_NODISCARD
  auto ptr() const VEG_NOEXCEPT->void const* { return stack_data; }

private:
  VEG_INLINE void assert_valid_len(PROXSUITE_MAYBE_UNUSED isize len)
    VEG_NOEXCEPT
  {
    VEG_INTERNAL_ASSERT_PRECONDITIONS(isize(len) >= 0);
  }

public:
  VEG_TEMPLATE((typename T),
               requires VEG_CONCEPT(constructible<T>),
               VEG_NODISCARD auto make_new,
               (/*unused*/, Tag<T>),
               (len, isize),
               (align = alignof(T), isize))
  VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_constructible<T>))->DynStackArray<T>
  {
    assert_valid_len(len);
    DynStackArray<T> get{
      *this, isize(len), align, _detail::_dynstack::zero_init_fn{}
    };
    VEG_ASSERT(get.ptr() != nullptr);
    return VEG_FWD(get);
  }

  VEG_TEMPLATE((typename T),
               requires VEG_CONCEPT(constructible<T>),
               VEG_NODISCARD auto make_new_for_overwrite,
               (/*unused*/, Tag<T>),
               (len, isize),
               (align = alignof(T), isize))

  VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_constructible<T>))->DynStackArray<T>
  {
    assert_valid_len(len);
    DynStackArray<T> get{
      *this, isize(len), align, _detail::_dynstack::default_init_fn{}
    };
    VEG_ASSERT(get.ptr() != nullptr);
    return VEG_FWD(get);
  }

  template<typename T>
  VEG_NODISCARD auto make_alloc(Tag<T> /*unused*/,
                                isize len,
                                isize align = alignof(T))
    VEG_NOEXCEPT->DynStackAlloc<T>
  {
    assert_valid_len(len);
    DynStackAlloc<T> get{
      *this, isize(len), align, _detail::_dynstack::no_init_fn{}
    };
    VEG_ASSERT(get.ptr() != nullptr);
    return VEG_FWD(get);
  }

private:
  void* stack_data;
  isize stack_bytes;

  template<typename T>
  friend struct DynStackAlloc;
  template<typename T>
  friend struct DynStackArray;
  friend struct _detail::_dynstack::cleanup;
  friend struct _detail::_dynstack::DynAllocBase;
};
} // namespace dynstack

namespace _detail {
namespace _dynstack {

struct cleanup
{
  bool const& success;
  proxsuite::linalg::veg::dynstack::DynStackMut& parent;
  void* old_data;
  isize old_rem_bytes;

  VEG_INLINE void operator()() const VEG_NOEXCEPT
  {
    if (!success) {
      parent.stack_data = old_data;
      parent.stack_bytes = old_rem_bytes;
    }
  }
};

struct DynAllocBase
{
  proxsuite::linalg::veg::dynstack::DynStackMut* parent;
  void* old_pos;
  void const volatile* data;
  isize len;

  void destroy(void const volatile* void_data_end) VEG_NOEXCEPT
  {
    if (data != nullptr) {
      // in case resource lifetimes are reodered by moving ownership
      PROXSUITE_MAYBE_UNUSED auto* parent_stack_data =
        static_cast<unsigned char*>(parent->stack_data);
      PROXSUITE_MAYBE_UNUSED auto* old_position =
        static_cast<unsigned char*>(old_pos);
      PROXSUITE_MAYBE_UNUSED auto* data_end =
        static_cast<unsigned char*>(const_cast<void*>(void_data_end));

      VEG_INTERNAL_ASSERT_PRECONDITIONS( //
        parent_stack_data == data_end,
        parent_stack_data >= old_position);

      parent->stack_bytes +=
        static_cast<isize>(static_cast<unsigned char*>(parent->stack_data) -
                           static_cast<unsigned char*>(old_pos));
      parent->stack_data = old_pos;
    }
  }
};
} // namespace _dynstack
} // namespace _detail

namespace dynstack {
template<typename T>
struct DynStackAlloc : _detail::_dynstack::DynAllocBase
{
private:
  using Base = _detail::_dynstack::DynAllocBase;

public:
  VEG_INLINE ~DynStackAlloc() VEG_NOEXCEPT
  {
    Base::destroy(ptr_mut() + Base::len);
  }

  DynStackAlloc(DynStackAlloc const&) = delete;
  DynStackAlloc(DynStackAlloc&& other) VEG_NOEXCEPT : Base{ Base(other) }
  {
    other.Base::len = 0;
    other.Base::data = nullptr;
  };

  auto operator=(DynStackAlloc const&) -> DynStackAlloc& = delete;
  auto operator=(DynStackAlloc&& rhs) VEG_NOEXCEPT->DynStackAlloc&
  {
    {
      auto cleanup = static_cast<decltype(rhs)>(*this);
    }
    static_cast<Base&>(*this) = rhs;
    static_cast<Base&>(rhs) = {};
    return *this;
  }

  VEG_NODISCARD auto as_mut() VEG_NOEXCEPT->SliceMut<T>
  {
    return {
      unsafe,
      FromRawParts{},
      ptr_mut(),
      len(),
    };
  }

  VEG_NODISCARD auto as_ref() const VEG_NOEXCEPT->Slice<T>
  {
    return {
      unsafe,
      FromRawParts{},
      ptr(),
      len(),
    };
  }

  VEG_NODISCARD auto ptr_mut() VEG_NOEXCEPT->T*
  {
    return /* NOLINT(clang-analyzer-linalg.uninitialized.UndefReturn) */
      static_cast<T*>(const_cast<void*>(Base::data));
  }
  VEG_NODISCARD auto ptr() const VEG_NOEXCEPT->T const*
  {
    return /* NOLINT(clang-analyzer-linalg.uninitialized.UndefReturn) */
      static_cast<T const*>(const_cast<void const*>(Base::data));
  }
  VEG_NODISCARD auto len() const VEG_NOEXCEPT->isize
  {
    return isize(Base::len);
  }

private:
  friend struct DynStackArray<T>;
  friend struct DynStackMut;
  friend struct _detail::_dynstack::DynStackArrayDtor<T>;

  template<typename Fn>
  DynStackAlloc(DynStackMut& parent_ref, isize alloc_size, isize align, Fn fn)
    VEG_NOEXCEPT_IF(VEG_CONCEPT(nothrow_constructible<T>))
    : Base{
      &parent_ref,
      parent_ref.stack_data,
      nullptr,
      0,
    }
  {

    void* const parent_data = parent_ref.stack_data;
    isize const parent_bytes = parent_ref.stack_bytes;

    void* const ptr = _detail::align_next(align,
                                          alloc_size * isize(sizeof(T)),
                                          parent_ref.stack_data,
                                          parent_ref.stack_bytes);

    if (ptr != nullptr) {
      bool success = false;
      auto&& cleanup = defer(_detail::_dynstack::cleanup{
        success, *parent, parent_data, parent_bytes });
      (void)cleanup;

      Base::len = alloc_size;
      Base::data = fn.template make<T>(ptr, alloc_size);

      success = true;
    }
  }
};

template<typename T>
struct DynStackArray
  : private // destruction order matters
    DynStackAlloc<T>
  , _detail::_dynstack::DynStackArrayDtor<T>
{
private:
  using Base = _detail::_dynstack::DynAllocBase;

public:
  using DynStackAlloc<T>::as_ref;
  using DynStackAlloc<T>::as_mut;
  using DynStackAlloc<T>::ptr;
  using DynStackAlloc<T>::ptr_mut;
  using DynStackAlloc<T>::len;

  ~DynStackArray() = default;
  DynStackArray(DynStackArray const&) = delete;
  DynStackArray(DynStackArray&&) VEG_NOEXCEPT = default;
  auto operator=(DynStackArray const&) -> DynStackArray& = delete;

  auto operator=(DynStackArray&& rhs) VEG_NOEXCEPT->DynStackArray&
  {
    {
      auto cleanup = static_cast<decltype(rhs)>(*this);
    }
    static_cast<Base&>(*this) = rhs;
    static_cast<Base&>(rhs) = {};
    return *this;
  }

private:
  using DynStackAlloc<T>::DynStackAlloc;
  friend struct DynStackMut;
  friend struct _detail::_dynstack::DynStackArrayDtor<T>;
};
} // namespace dynstack
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_DYNAMIC_STACK_DYNAMIC_STACK_HPP_UBOMZFTOS   \
        */
