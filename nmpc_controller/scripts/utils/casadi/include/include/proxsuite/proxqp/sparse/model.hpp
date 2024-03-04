//
// Copyright (c) 2022 INRIA
//
/** \file */
#ifndef PROXSUITE_PROXQP_SPARSE_MODEL_HPP
#define PROXSUITE_PROXQP_SPARSE_MODEL_HPP

#include <Eigen/Sparse>
#include "proxsuite/linalg/sparse/core.hpp"
#include "proxsuite/proxqp/sparse/fwd.hpp"

namespace proxsuite {
namespace proxqp {
namespace sparse {
///
/// @brief This class stores the model of the QP problem.
///
/*!
 * Model class of the sparse solver storing the QP problem structure.
 */
template<typename T, typename I>
struct Model
{
  typedef T Scalar;
  using VectorType = Eigen::Matrix<T, Eigen::Dynamic, 1>;

  isize dim;
  isize n_eq;
  isize n_in;

  isize H_nnz;
  isize A_nnz;
  isize C_nnz;

  proxsuite::linalg::veg::Vec<I> kkt_col_ptrs;
  proxsuite::linalg::veg::Vec<I> kkt_row_indices;
  proxsuite::linalg::veg::Vec<T> kkt_values;

  proxsuite::linalg::veg::Vec<I> kkt_col_ptrs_unscaled;
  proxsuite::linalg::veg::Vec<I> kkt_row_indices_unscaled;
  proxsuite::linalg::veg::Vec<T> kkt_values_unscaled;

  VectorType g;
  VectorType b;
  VectorType l;
  VectorType u;

  /*!
   * Default constructor.
   * @param dim primal variable dimension.
   * @param n_eq number of equality constraints.
   * @param n_in number of inequality constraints.
   */
  Model(isize dim, isize n_eq, isize n_in)
    : dim(dim)
    , n_eq(n_eq)
    , n_in(n_in)
  {
    PROXSUITE_THROW_PRETTY(dim == 0,
                           std::invalid_argument,
                           "wrong argument size: the dimension wrt primal "
                           "variable x should be strictly positive.");

    const T infinite_bound_value = helpers::infinite_bound<T>::value();

    g.setZero();
    b.setZero();
    u.setZero();
    u.fill(+infinite_bound_value); // in case it appears u is nullopt (i.e., the
                                   // problem is only lower bounded)
    l.fill(-infinite_bound_value); // in case it appears l is nullopt (i.e., the
                                   // problem is only upper bounded)
  }
  /*!
   * Returns the current (scaled) KKT matrix of the problem.
   */
  auto kkt() const -> proxsuite::linalg::sparse::MatRef<T, I>
  {
    auto n_tot = kkt_col_ptrs.len() - 1;
    auto nnz =
      isize(proxsuite::linalg::sparse::util::zero_extend(kkt_col_ptrs[n_tot]));
    return {
      proxsuite::linalg::sparse::from_raw_parts,
      n_tot,
      n_tot,
      nnz,
      kkt_col_ptrs.ptr(),
      nullptr,
      kkt_row_indices.ptr(),
      kkt_values.ptr(),
    };
  }
  /*!
   * Returns the current (scaled) KKT matrix of the problem (mutable form).
   */
  auto kkt_mut() -> proxsuite::linalg::sparse::MatMut<T, I>
  {
    auto n_tot = kkt_col_ptrs.len() - 1;
    auto nnz =
      isize(proxsuite::linalg::sparse::util::zero_extend(kkt_col_ptrs[n_tot]));
    return {
      proxsuite::linalg::sparse::from_raw_parts,
      n_tot,
      n_tot,
      nnz,
      kkt_col_ptrs.ptr_mut(),
      nullptr,
      kkt_row_indices.ptr_mut(),
      kkt_values.ptr_mut(),
    };
  }
  /*!
   * Returns the original (unscaled) KKT matrix of the problem.
   */
  auto kkt_unscaled() const -> proxsuite::linalg::sparse::MatRef<T, I>
  {
    auto n_tot = kkt_col_ptrs_unscaled.len() - 1;
    auto nnz = isize(proxsuite::linalg::sparse::util::zero_extend(
      kkt_col_ptrs_unscaled[n_tot]));
    return {
      proxsuite::linalg::sparse::from_raw_parts,
      n_tot,
      n_tot,
      nnz,
      kkt_col_ptrs_unscaled.ptr(),
      nullptr,
      kkt_row_indices_unscaled.ptr(),
      kkt_values_unscaled.ptr(),
    };
  }
  /*!
   * Returns the original (unscaled) KKT matrix of the problem (mutable form).
   */
  auto kkt_mut_unscaled() -> proxsuite::linalg::sparse::MatMut<T, I>
  {
    auto n_tot = kkt_col_ptrs_unscaled.len() - 1;
    auto nnz = isize(proxsuite::linalg::sparse::util::zero_extend(
      kkt_col_ptrs_unscaled[n_tot]));
    return {
      proxsuite::linalg::sparse::from_raw_parts,
      n_tot,
      n_tot,
      nnz,
      kkt_col_ptrs_unscaled.ptr_mut(),
      nullptr,
      kkt_row_indices_unscaled.ptr_mut(),
      kkt_values_unscaled.ptr_mut(),
    };
  }
};

template<typename _Scalar>
struct SparseModel
{
  typedef _Scalar Scalar;
  enum
  {
    layout = Eigen::RowMajor
  };

  using VectorType = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  Eigen::SparseMatrix<Scalar, 1> H;
  VectorType g;
  Eigen::SparseMatrix<Scalar, 1> A;
  VectorType b;
  Eigen::SparseMatrix<Scalar, 1> C;
  VectorType u;
  VectorType l;

  template<typename Vector_g,
           typename Vector_b,
           typename Vector_u,
           typename Vector_l>
  SparseModel(const Eigen::SparseMatrix<Scalar, 1>& H,
              const Eigen::MatrixBase<Vector_g>& g,
              const Eigen::SparseMatrix<Scalar, 1>& A,
              const Eigen::MatrixBase<Vector_b>& b,
              const Eigen::SparseMatrix<Scalar, 1>& C,
              const Eigen::MatrixBase<Vector_u>& u,
              const Eigen::MatrixBase<Vector_l>& l) noexcept
    : H(H)
    , g(g)
    , A(A)
    , b(b)
    , C(C)
    , u(u)
    , l(l)
  {
  }

  auto as_view() -> proxqp::dense::QpView<Scalar>
  {
    return {
      { proxqp::from_eigen, H },
      { proxqp::from_eigen, g },
      { proxqp::from_eigen, A },
      { proxqp::from_eigen, b },
      { proxqp::from_ptr_rows_cols_stride,
        nullptr,
        0,
        proxqp::isize(H.rows()),
        0 },
      { proxqp::from_ptr_size, nullptr, 0 },
    };
  }
  auto as_mut() -> proxqp::dense::QpViewMut<Scalar>
  {
    return {
      { proxqp::from_eigen, H },
      { proxqp::from_eigen, g },
      { proxqp::from_eigen, A },
      { proxqp::from_eigen, b },
      { proxqp::from_ptr_rows_cols_stride,
        nullptr,
        0,
        proxqp::isize(H.rows()),
        0 },
      { proxqp::from_ptr_size, nullptr, 0 },
    };
  }
};

} // namespace sparse
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_SPARSE_MODEL_HPP */
