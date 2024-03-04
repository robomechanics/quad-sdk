/** \file */
//
// Copyright (c) 2022 INRIA
//
#ifndef PROXSUITE_LINALG_SPARSE_LDLT_CORE_HPP
#define PROXSUITE_LINALG_SPARSE_LDLT_CORE_HPP

#include <proxsuite/linalg/veg/slice.hpp>
#include <proxsuite/linalg/veg/memory/dynamic_stack.hpp>
#include <type_traits>
#include <Eigen/SparseCore>

#define SPARSE_LDLT_CONCEPT(...)                                               \
  VEG_CONCEPT_MACRO(::proxsuite::linalg::sparse::concepts, __VA_ARGS__)
#define SPARSE_LDLT_CHECK_CONCEPT(...)                                         \
  VEG_CONCEPT_MACRO(::proxsuite::linalg::sparse::concepts, __VA_ARGS__)

namespace proxsuite {
namespace linalg {
namespace sparse {
using proxsuite::linalg::veg::dynstack::DynStackMut;
using namespace proxsuite::linalg::veg::literals;

using proxsuite::linalg::veg::isize;
using proxsuite::linalg::veg::usize;

using proxsuite::linalg::veg::Slice;
using proxsuite::linalg::veg::SliceMut;

using proxsuite::linalg::veg::mut;
using proxsuite::linalg::veg::Ref;
using proxsuite::linalg::veg::ref;
using proxsuite::linalg::veg::RefMut;

inline namespace tags {
using proxsuite::linalg::veg::Unsafe;
using proxsuite::linalg::veg::unsafe;

using proxsuite::linalg::veg::from_raw_parts;
using proxsuite::linalg::veg::FromRawParts;
VEG_TAG(from_eigen, FromEigen);
} // namespace tags

namespace concepts {
} // namespace concepts

namespace _detail {
template<typename I, bool = sizeof(I) < sizeof(int)>
struct WrappingPlusType;

template<typename I>
struct WrappingPlusType<I, true>
{
  using Promoted = unsigned;
};
template<typename I>
struct WrappingPlusType<I, false>
{
  using Promoted = typename std::make_unsigned<I>::type;
};
} // namespace _detail
namespace util {
namespace nb {
struct wrapping_plus
{
  template<typename I>
  auto operator()(I a, I b) const noexcept -> I
  {
    using U = typename _detail::WrappingPlusType<I>::Promoted;
    return I(U(a) + U(b));
  }
};
struct checked_non_negative_plus
{
  template<typename I>
  auto operator()(I a, I b) const noexcept -> I
  {
    return (VEG_ASSERT(wrapping_plus{}(a, b) >= a), //
            wrapping_plus{}(a, b));
  }
};

struct wrapping_inc
{
  template<typename I>
  auto operator()(RefMut<I> a) const noexcept -> I
  {
    return a.get() = wrapping_plus{}(a.get(), I(1));
  }
};
struct wrapping_dec
{
  template<typename I>
  auto operator()(RefMut<I> a) const noexcept -> I
  {
    return a.get() = wrapping_plus{}(a.get(), I(-1));
  }
};
struct sign_extend
{
  template<typename I>
  auto operator()(I a) const noexcept -> usize
  {
    return usize(isize(typename std::make_signed<I>::type(a)));
  }
};
struct zero_extend
{
  template<typename I>
  auto operator()(I a) const noexcept -> usize
  {
    return usize(typename std::make_unsigned<I>::type(a));
  }
};
} // namespace nb
VEG_NIEBLOID(wrapping_plus);
VEG_NIEBLOID(checked_non_negative_plus);
VEG_NIEBLOID(wrapping_inc);
VEG_NIEBLOID(wrapping_dec);
VEG_NIEBLOID(sign_extend);
VEG_NIEBLOID(zero_extend);
} // namespace util

template<typename T>
struct DenseVecRef
{
  DenseVecRef() = default;
  DenseVecRef(FromRawParts /*from_raw_parts*/,
              T const* data,
              isize len) noexcept
    : _{ data, len }
  {
  }
  template<typename V>
  DenseVecRef(FromEigen /*from_eigen*/, V const& v) noexcept
    : _{ v.data(), v.rows() }
  {
    static_assert(V::InnerStrideAtCompileTime == 1, ".");
    static_assert(V::ColsAtCompileTime == 1, ".");
  }

  auto as_slice() const noexcept -> Slice<T>
  {
    return {
      unsafe,
      from_raw_parts,
      _.ptr,
      _.size,
    };
  }
  auto nrows() const noexcept -> isize { return _.size; }
  auto ncols() const noexcept -> isize { return 1; }

  auto to_eigen() const noexcept -> Eigen::Map<Eigen::Matrix<T, -1, 1> const>
  {
    return { _.ptr, _.size };
  }

private:
  struct
  {
    T const* ptr;
    isize size;
  } _ = {};
};

template<typename T>
struct DenseVecMut
{
  DenseVecMut() = default;
  DenseVecMut(FromRawParts /*from_raw_parts*/, T* data, isize len) noexcept
    : _{ data, len }
  {
  }
  template<typename V>
  DenseVecMut(FromEigen /*from_eigen*/, V&& v) noexcept
    : _{ v.data(), v.rows() }
  {
    static_assert(
      proxsuite::linalg::veg::uncvref_t<V>::InnerStrideAtCompileTime == 1, ".");
    static_assert(proxsuite::linalg::veg::uncvref_t<V>::ColsAtCompileTime == 1,
                  ".");
  }

  auto as_slice() const noexcept -> Slice<T>
  {
    return {
      unsafe,
      from_raw_parts,
      _.ptr,
      _.size,
    };
  }
  auto as_slice_mut() noexcept -> SliceMut<T>
  {
    return {
      unsafe,
      from_raw_parts,
      _.ptr,
      _.size,
    };
  }

  auto as_const() const noexcept -> DenseVecRef<T>
  {
    return { from_raw_parts, _.ptr, _.size };
  }
  auto nrows() const noexcept -> isize { return _.size; }
  auto ncols() const noexcept -> isize { return 1; }

  auto to_eigen() const noexcept -> Eigen::Map<Eigen::Matrix<T, -1, 1>>
  {
    return { _.ptr, _.size };
  }

private:
  struct
  {
    T* ptr;
    isize size;
  } _ = {};
};

template<typename T, typename I = isize>
struct VecRef
{
  VecRef( //
    FromRawParts /*from_raw_parts*/,
    isize nrows,
    isize nnz,
    I const* row_indices,
    T const* values)
    : _{ nrows, nnz, row_indices, values }
  {
  }

  auto nrows() const noexcept -> isize { return _.nrows; }
  auto ncols() const noexcept -> isize { return 1; }
  auto nnz() const noexcept -> isize { return _.nnz; }

  auto row_indices() const noexcept -> I const* { return _.row; }
  auto values() const noexcept -> T const* { return _.val; }

private:
  struct
  {
    isize nrows;
    isize nnz;
    I const* row;
    T const* val;
  } _;
};

namespace _detail {
template<typename D, typename I>
struct SymbolicMatRefInterface
{
private:
  template<typename U = D>
  auto _() const noexcept -> decltype((VEG_DECLVAL(U const&)._))
  {
    return static_cast<D const*>(this)->_;
  }

public:
  auto nrows() const noexcept -> isize { return _().nrows; }
  auto ncols() const noexcept -> isize { return _().ncols; }
  auto nnz() const noexcept -> isize { return _().nnz; }

  auto col_ptrs() const noexcept -> I const* { return _().col; }
  auto nnz_per_col() const noexcept -> I const* { return _().nnz_per_col; }
  auto is_compressed() const noexcept -> bool
  {
    return nnz_per_col() == nullptr;
  }

  auto row_indices() const noexcept -> I const* { return _().row; }

  auto col_start(usize j) const noexcept -> usize
  {
    return VEG_ASSERT(j < usize(ncols())), util::zero_extend(_().col[j]);
  }
  auto col_start_unchecked(Unsafe /*unsafe*/, usize j) const noexcept -> usize
  {
    return VEG_DEBUG_ASSERT(j < usize(ncols())), util::zero_extend(_().col[j]);
  }
  auto col_end(usize j) const noexcept -> usize
  {
    return VEG_ASSERT(j < usize(ncols())), col_end_unchecked(unsafe, j);
  }
  auto col_end_unchecked(Unsafe /*unsafe*/, usize j) const noexcept -> usize
  {
    return VEG_DEBUG_ASSERT(j < usize(ncols())),
           util::zero_extend(is_compressed()
                               ? _().col[j + 1]
                               : I(_().col[j] + _().nnz_per_col[j]));
  }
};
template<typename D, typename I>
struct SymbolicMatMutInterface : SymbolicMatRefInterface<D, I>
{
private:
  template<typename U = D>
  auto _() noexcept -> decltype((VEG_DECLVAL(U&)._))
  {
    return static_cast<D*>(this)->_;
  }

public:
  auto col_ptrs_mut() noexcept -> I* { return _().col; }
  auto nnz_per_col_mut() noexcept -> I* { return _().nnz_per_col; }
  auto row_indices_mut() noexcept -> I* { return _().row; }
};
} // namespace _detail

template<typename I = isize>
struct SymbolicMatRef : _detail::SymbolicMatRefInterface<SymbolicMatRef<I>, I>
{
  friend struct _detail::SymbolicMatRefInterface<SymbolicMatRef, I>;
  SymbolicMatRef(FromRawParts /*from_raw_parts*/,
                 isize nrows,
                 isize ncols,
                 isize nnz,
                 I const* col_ptrs,
                 I const* nnz_per_col,
                 I const* row_indices)
    : _{
      nrows, ncols, nnz, col_ptrs, nnz_per_col, row_indices,
    }
  {
  }

private:
  struct
  {
    isize nrows;
    isize ncols;
    isize nnz;
    I const* col;
    I const* nnz_per_col;
    I const* row;
  } _;
};
template<typename I = isize>
struct SymbolicMatMut : _detail::SymbolicMatMutInterface<SymbolicMatMut<I>, I>
{
  friend struct _detail::SymbolicMatRefInterface<SymbolicMatMut, I>;
  friend struct _detail::SymbolicMatMutInterface<SymbolicMatMut, I>;
  SymbolicMatMut(FromRawParts /*from_raw_parts*/,
                 isize nrows,
                 isize ncols,
                 isize nnz,
                 I* col_ptrs,
                 I* nnz_per_col,
                 I* row_indices)
    : _{
      nrows, ncols, nnz, col_ptrs, nnz_per_col, row_indices,
    }
  {
  }

  auto as_const() const noexcept -> SymbolicMatRef<I>
  {
    return {
      from_raw_parts,   this->nrows(),       this->ncols(),       this->nnz(),
      this->col_ptrs(), this->nnz_per_col(), this->row_indices(),
    };
  }

private:
  struct
  {
    isize nrows;
    isize ncols;
    isize nnz;
    I* col;
    I* nnz_per_col;
    I* row;
  } _;
};

template<typename T, typename I = isize>
struct MatRef : _detail::SymbolicMatRefInterface<MatRef<T, I>, I>
{
  friend struct _detail::SymbolicMatRefInterface<MatRef, I>;
  MatRef(FromRawParts /*from_raw_parts*/,
         isize nrows,
         isize ncols,
         isize nnz,
         I const* col_ptrs,
         I const* nnz_per_col,
         I const* row_indices,
         T const* values)
    : _{
      nrows, ncols, nnz, col_ptrs, nnz_per_col, row_indices, values,
    }
  {
  }

  template<typename M>
  MatRef(FromEigen /*from_eigen*/, M const& m)
    : _{
      m.rows(),
      m.cols(),
      m.nonZeros(),
      m.outerIndexPtr(),
      m.innerNonZeroPtr(),
      m.innerIndexPtr(),
      m.valuePtr(),
    }
  {
    static_assert(!bool(M::IsRowMajor), ".");
  }

  auto values() const noexcept -> T const* { return _.val; }
  auto symbolic() const noexcept -> SymbolicMatRef<I>
  {
    return {
      from_raw_parts,   this->nrows(),       this->ncols(),       this->nnz(),
      this->col_ptrs(), this->nnz_per_col(), this->row_indices(),
    };
  }

  auto to_eigen() const noexcept
    -> Eigen::Map<Eigen::SparseMatrix<T, Eigen::ColMajor, I> const>
  {
    return { _.nrows, _.ncols, _.nnz, _.col, _.row, _.val, _.nnz_per_col };
  }

private:
  struct
  {
    isize nrows;
    isize ncols;
    isize nnz;
    I const* col;
    I const* nnz_per_col;
    I const* row;
    T const* val;
  } _;
};

template<typename T, typename I = isize>
struct MatMut : _detail::SymbolicMatMutInterface<MatMut<T, I>, I>
{
  friend struct _detail::SymbolicMatRefInterface<MatMut, I>;
  friend struct _detail::SymbolicMatMutInterface<MatMut, I>;
  MatMut(FromRawParts /*from_raw_parts*/,
         isize nrows,
         isize ncols,
         isize nnz,
         I* col_ptrs,
         I* nnz_per_col,
         I* row_indices,
         T* values)
    : _{
      nrows, ncols, nnz, col_ptrs, nnz_per_col, row_indices, values,
    }
  {
  }

  template<typename M>
  MatMut(FromEigen /*from_eigen*/, M&& m)
    : _{
      m.rows(),
      m.cols(),
      m.nonZeros(),
      m.outerIndexPtr(),
      m.innerNonZeroPtr(),
      m.innerIndexPtr(),
      m.valuePtr(),
    }
  {
    static_assert(!bool(proxsuite::linalg::veg::uncvref_t<M>::IsRowMajor), ".");
  }

  auto values() const noexcept -> T const* { return _.val; }
  auto values_mut() const noexcept -> T* { return _.val; }
  auto is_compressed() const noexcept -> bool
  {
    return _.nnz_per_col == nullptr;
  }

  auto as_const() const noexcept -> MatRef<T, I>
  {
    return {
      from_raw_parts,      this->nrows(),    this->ncols(),
      this->nnz(),         this->col_ptrs(), this->nnz_per_col(),
      this->row_indices(), this->values(),
    };
  }
  auto symbolic() const noexcept -> SymbolicMatRef<I>
  {
    return {
      from_raw_parts,   this->nrows(),       this->ncols(),       this->nnz(),
      this->col_ptrs(), this->nnz_per_col(), this->row_indices(),
    };
  }
  auto symbolic_mut() const noexcept -> SymbolicMatRef<I>
  {
    return {
      from_raw_parts,          this->nrows(),
      this->ncols(),           this->nnz(),
      this->col_ptrs_mut(),    this->nnz_per_col_mut(),
      this->row_indices_mut(),
    };
  }
  auto to_eigen() const noexcept
    -> Eigen::Map<Eigen::SparseMatrix<T, Eigen::ColMajor, I>>
  {
    return { _.nrows, _.ncols, _.nnz, _.col, _.row, _.val, _.nnz_per_col };
  }
  void _set_nnz(isize new_nnz) noexcept { _.nnz = new_nnz; }

private:
  struct
  {
    isize nrows;
    isize ncols;
    isize nnz;
    I* col;
    I* nnz_per_col;
    I* row;
    T* val;
  } _;
};
} // namespace sparse
} // namespace linalg
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_LINALG_SPARSE_LDLT_CORE_HPP */
