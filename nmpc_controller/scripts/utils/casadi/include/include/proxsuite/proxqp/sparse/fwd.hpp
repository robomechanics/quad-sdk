//
// Copyright (c) 2022-2023 INRIA
//
/** \file */
#ifndef PROXSUITE_PROXQP_SPARSE_FWD_HPP
#define PROXSUITE_PROXQP_SPARSE_FWD_HPP

#include <Eigen/Sparse>
#include "proxsuite/linalg/veg/vec.hpp"
#include "proxsuite/proxqp/dense/views.hpp"
#include "proxsuite/helpers/common.hpp"

namespace proxsuite {
namespace proxqp {
namespace sparse {

using dense::infty_norm;
using proxsuite::linalg::veg::i64;
using proxsuite::linalg::veg::isize;
using proxsuite::linalg::veg::usize;

template<typename T>
using DMat = Eigen::Matrix<T, -1, -1>;

static constexpr auto DYN = Eigen::Dynamic;
enum
{
  layout = Eigen::RowMajor
};
template<typename T, typename I>
using SparseMat = Eigen::SparseMatrix<T, Eigen::ColMajor, I>;
// using SparseMat = Eigen::SparseMatrix<T, Eigen::RowMajor, I>;
template<typename T>
using VecRef = Eigen::Ref<Eigen::Matrix<T, DYN, 1> const>;
template<typename T>
using MatRef = Eigen::Ref<Eigen::Matrix<T, DYN, DYN> const>;
template<typename T>
using Vec = Eigen::Matrix<T, DYN, 1>;

template<typename T, typename I>
using Mat = Eigen::SparseMatrix<T, Eigen::ColMajor, I>;
// using Mat = Eigen::SparseMatrix<T, Eigen::RowMajor, I>;
using VecBool = Eigen::Matrix<bool, DYN, 1>;

///
/// @brief This class defines the workspace of the sparse solver.
///
/*!
 * Workspace class of the sparse solver.
 */
template<typename T, typename I>
struct Workspace;

} // namespace sparse
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_SPARSE_FWD_HPP */
