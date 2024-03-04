//
// Copyright (c) 2022 INRIA
//
/** \file */

#ifndef PROXSUITE_PROXQP_SPARSE_VIEWS_HPP
#define PROXSUITE_PROXQP_SPARSE_VIEWS_HPP

#include <proxsuite/linalg/dense/core.hpp>
#include <proxsuite/linalg/sparse/core.hpp>
#include <proxsuite/linalg/sparse/factorize.hpp>
#include <proxsuite/linalg/sparse/update.hpp>
#include <proxsuite/linalg/sparse/rowmod.hpp>
#include <proxsuite/proxqp/dense/views.hpp>
#include <proxsuite/proxqp/settings.hpp>
#include <proxsuite/linalg/veg/vec.hpp>
#include "proxsuite/proxqp/sparse/model.hpp"
#include "proxsuite/proxqp/results.hpp"

#include <Eigen/IterativeLinearSolvers>
#include <unsupported/Eigen/IterativeSolvers>

namespace proxsuite {
namespace proxqp {
namespace sparse {

template<typename T, typename I>
struct QpView
{
  proxsuite::linalg::sparse::MatRef<T, I> H;
  proxsuite::linalg::sparse::DenseVecRef<T> g;
  proxsuite::linalg::sparse::MatRef<T, I> AT;
  proxsuite::linalg::sparse::DenseVecRef<T> b;
  proxsuite::linalg::sparse::MatRef<T, I> CT;
  proxsuite::linalg::sparse::DenseVecRef<T> l;
  proxsuite::linalg::sparse::DenseVecRef<T> u;
};

template<typename T, typename I>
struct QpViewMut
{
  proxsuite::linalg::sparse::MatMut<T, I> H;
  proxsuite::linalg::sparse::DenseVecMut<T> g;
  proxsuite::linalg::sparse::MatMut<T, I> AT;
  proxsuite::linalg::sparse::DenseVecMut<T> b;
  proxsuite::linalg::sparse::MatMut<T, I> CT;
  proxsuite::linalg::sparse::DenseVecMut<T> l;
  proxsuite::linalg::sparse::DenseVecMut<T> u;

  auto as_const() noexcept -> QpView<T, I>
  {
    return {
      H.as_const(),  g.as_const(), AT.as_const(), b.as_const(),
      CT.as_const(), l.as_const(), u.as_const(),
    };
  }
};

} // namespace sparse
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_SPARSE_VIEWS_HPP */
