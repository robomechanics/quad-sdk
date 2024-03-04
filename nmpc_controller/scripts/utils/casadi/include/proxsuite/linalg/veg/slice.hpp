#ifndef VEG_SLICE_HPP_GKSTE2JDS
#define VEG_SLICE_HPP_GKSTE2JDS

#include "proxsuite/linalg/veg/util/assert.hpp"
#include "proxsuite/linalg/veg/util/get.hpp"
#include "proxsuite/linalg/veg/internal/narrow.hpp"
#include "proxsuite/linalg/veg/tuple.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"
#include <initializer_list>

namespace proxsuite {
namespace linalg {
namespace veg {
template<typename T, usize N>
using CArray = T[N];

namespace _detail {
namespace _slice {
namespace adl {
struct AdlBase
{};
} // namespace adl
} // namespace _slice
} // namespace _detail

template<typename T>
struct Slice : _detail::_slice::adl::AdlBase
{
private:
  T const* data = nullptr;
  isize size = 0;

public:
  VEG_INLINE
  constexpr Slice() = default;

  VEG_INLINE
  constexpr Slice(Unsafe /*tag*/,
                  FromRawParts /*tag*/,
                  T const* data_,
                  isize count) VEG_NOEXCEPT
    : data{ data_ }
    , size{ count }
  {
  }

  VEG_NODISCARD
  VEG_INLINE
  constexpr auto ptr() const VEG_NOEXCEPT->T const* { return data; }
  VEG_NODISCARD
  VEG_INLINE
  constexpr auto len() const VEG_NOEXCEPT->isize { return size; }

  VEG_NODISCARD
  VEG_INLINE
  constexpr auto operator[](isize idx) const VEG_NOEXCEPT->T const&
  {
    return VEG_INTERNAL_ASSERT_PRECONDITION(usize(idx) < usize(len())),
           *(data + idx);
  }
  VEG_NODISCARD
  VEG_INLINE
  constexpr auto get_unchecked(Unsafe /*tag*/,
                               isize idx) const VEG_NOEXCEPT->Ref<T>
  {
    return ref(*(data + idx));
  }

  VEG_NODISCARD VEG_INLINE constexpr auto split_at(isize idx) const
    VEG_NOEXCEPT->Tuple<Slice<T>, Slice<T>>
  {
    return VEG_INTERNAL_ASSERT_PRECONDITION(usize(idx) <= usize(len())),
           Tuple<Slice<T>, Slice<T>>{
             tuplify,
             Slice<T>{
               unsafe,
               FromRawParts{},
               data,
               idx,
             },
             Slice<T>{
               unsafe,
               FromRawParts{},
               data + idx,
               size - idx,
             },
           };
  }

  VEG_NODISCARD VEG_INLINE auto as_bytes() const
    VEG_NOEXCEPT->Slice<unsigned char>
  {
    return {
      unsafe,
      from_raw_parts,
      reinterpret_cast<unsigned char const*>(data),
      isize(sizeof(T)) * size,
    };
  }
};

template<typename T>
struct SliceMut : private Slice<T>
{
  VEG_INLINE
  constexpr SliceMut() = default;

  VEG_INLINE
  constexpr SliceMut(Unsafe /*tag*/,
                     FromRawParts /*tag*/,
                     T const* data_,
                     isize count) VEG_NOEXCEPT
    : Slice<T>{
      unsafe,
      from_raw_parts,
      data_,
      count,
    }
  {
  }

  using Slice<T>::ptr;
  using Slice<T>::as_bytes;
  using Slice<T>::split_at;
  using Slice<T>::len;
  using Slice<T>::get_unchecked;

  VEG_NODISCARD VEG_INLINE constexpr auto as_const() const noexcept -> Slice<T>
  {
    return *this;
  }

  VEG_NODISCARD
  VEG_INLINE
  VEG_CPP14(constexpr) auto operator[](isize idx) VEG_NOEXCEPT->T&
  {
    return const_cast<T&>(static_cast<Slice<T> const&>(*this)[idx]);
  }
  VEG_NODISCARD
  VEG_INLINE
  VEG_CPP14(constexpr) auto ptr_mut() VEG_NOEXCEPT->T*
  {
    return const_cast<T*>(ptr());
  }
  VEG_NODISCARD
  VEG_INLINE
  VEG_CPP14(constexpr)
  auto get_mut_unchecked(Unsafe /*tag*/, isize idx) VEG_NOEXCEPT->RefMut<T>
  {
    return mut(const_cast<T&>(*(this->data + idx)));
  }
  VEG_NODISCARD VEG_INLINE auto as_mut_bytes()
    VEG_NOEXCEPT->SliceMut<unsigned char>
  {
    return {
      unsafe,
      from_raw_parts,
      reinterpret_cast<unsigned char*>(ptr_mut()),
      isize(sizeof(T)) * len(),
    };
  }

  VEG_NODISCARD VEG_INLINE VEG_CPP14(constexpr) auto split_at_mut(isize idx)
    VEG_NOEXCEPT->Tuple<SliceMut<T>, SliceMut<T>>
  {
    return VEG_INTERNAL_ASSERT_PRECONDITION(usize(idx) <= usize(len())),
           Tuple<SliceMut<T>, SliceMut<T>>{
             tuplify,
             SliceMut<T>{
               unsafe,
               from_raw_parts,
               ptr_mut(),
               idx,
             },
             SliceMut<T>{
               unsafe,
               from_raw_parts,
               ptr_mut() + idx,
               len() - idx,
             },
           };
  }
};

namespace array {
template<typename T, isize N>
struct Array
{
  static_assert(N > 0, ".");
  T _[static_cast<usize>(N)];

  constexpr auto as_ref() const -> Slice<T>
  {
    return {
      unsafe,
      from_raw_parts,
      static_cast<T const*>(_),
      N,
    };
  }
  VEG_CPP14(constexpr) auto as_mut() -> SliceMut<T>
  {
    return {
      unsafe,
      from_raw_parts,
      static_cast<T*>(_),
      N,
    };
  }
};
} // namespace array
using array::Array;

namespace nb {
struct init_list
{
  template<typename T>
  VEG_CPP14(constexpr)
  auto operator()(std::initializer_list<T> init_list) const noexcept -> Slice<T>
  {
    return {
      unsafe,
      from_raw_parts,
      init_list.begin(),
      isize(init_list.size()),
    };
  }
};
} // namespace nb
VEG_NIEBLOID(init_list);

template<typename T>
struct cpo::is_trivially_constructible<Slice<T>> : meta::bool_constant<true>
{};
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_SLICE_HPP_GKSTE2JDS */
