//
// Copyright (c) 2022 INRIA
//
/** \file */

#ifndef PROXSUITE_PROXQP_SPARSE_PRECOND_IDENTITY_HPP
#define PROXSUITE_PROXQP_SPARSE_PRECOND_IDENTITY_HPP

namespace proxsuite {
namespace proxqp {
namespace sparse {
namespace preconditioner {

template<typename T, typename I>
struct Identity
{

  static auto scale_qp_in_place_req(proxsuite::linalg::veg::Tag<T> /*tag*/,
                                    isize /*n*/,
                                    isize /*n_eq*/,
                                    isize /*n_in*/)
    -> proxsuite::linalg::veg::dynstack::StackReq
  {
    return { 0, 1 };
  }

  void scale_qp_in_place(
    QpViewMut<T, I> /*qp*/,
    proxsuite::linalg::veg::dynstack::DynStackMut /*stack*/)
  {
  }

  // modifies variables in place
  void scale_primal_in_place(VectorViewMut<T> /*primal*/) {}
  void scale_dual_in_place(VectorViewMut<T> /*dual*/) {}

  void scale_dual_in_place_eq(VectorViewMut<T> /*dual*/) {}
  void scale_dual_in_place_in(VectorViewMut<T> /*dual*/) {}

  void unscale_primal_in_place(VectorViewMut<T> /*primal*/) {}
  void unscale_dual_in_place(VectorViewMut<T> /*dual*/) {}

  void unscale_dual_in_place_eq(VectorViewMut<T> /*dual*/) {}

  void unscale_dual_in_place_in(VectorViewMut<T> /*dual*/) {}
  // modifies residuals in place
  void scale_primal_residual_in_place(VectorViewMut<T> /*primal*/) {}

  void scale_primal_residual_in_place_eq(VectorViewMut<T> /*primal_eq*/) {}
  void scale_primal_residual_in_place_in(VectorViewMut<T> /*primal_in*/) {}
  void scale_dual_residual_in_place(VectorViewMut<T> /*dual*/) {}
  void unscale_primal_residual_in_place(VectorViewMut<T> /*primal*/) {}
  void unscale_primal_residual_in_place_eq(VectorViewMut<T> /*primal_eq*/) {}
  void unscale_primal_residual_in_place_in(VectorViewMut<T> /*primal_in*/) {}
  void unscale_dual_residual_in_place(VectorViewMut<T> /*dual*/) {}
};

} // namespace preconditioner

} // namespace sparse
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_SPARSE_PRECOND_IDENTITY_HPP */
