//
// Copyright (c) 2022 INRIA
//
/**
 * @file identity.hpp
 */
#ifndef PROXSUITE_PROXQP_DENSE_PRECOND_IDENTITY_HPP
#define PROXSUITE_PROXQP_DENSE_PRECOND_IDENTITY_HPP

#include "proxsuite/proxqp/dense/views.hpp"

namespace proxsuite {
namespace proxqp {
namespace dense {
namespace preconditioner {
struct IdentityPrecond
{

  /*!
   * Scales the qp performing using identity equilibrator (i.e., does nothing).
   */
  template<typename T>
  void scale_qp_in_place(QpViewBoxMut<T> /*qp*/) const noexcept
  {
  }
  /*!
   * Scales a primal variable in place.
   */
  template<typename T>
  void scale_primal_in_place(VectorViewMut<T> /*x*/) const noexcept
  {
  }
  /*!
   * Scales a dual inequality constrained variable in place.
   */
  template<typename T>
  void scale_dual_in_place_in(VectorViewMut<T> /*y*/) const noexcept
  {
  }
  /*!
   * Scales a dual equality constrained variable in place.
   */
  template<typename T>
  void scale_dual_in_place_eq(VectorViewMut<T> /*y*/) const noexcept
  {
  }
  /*!
   * Scales a primal residual in place.
   */
  template<typename T>
  void scale_primal_residual_in_place(VectorViewMut<T> /*x*/) const noexcept
  {
  }
  /*!
   * Scales a dual residual in place.
   */
  template<typename T>
  void scale_dual_residual_in_place(VectorViewMut<T> /*y*/) const noexcept
  {
  }
  /*!
   * Unscales a primal variable in place.
   */
  template<typename T>
  void unscale_primal_in_place(VectorViewMut<T> /*x*/) const noexcept
  {
  }

  /*!
   * Unscales a dual variable in place.
   */
  template<typename T>
  void unscale_dual_in_place_in(VectorViewMut<T> /*y*/) const noexcept
  {
  }
  /*!
   * Unscales a dual equality variable in place.
   */
  template<typename T>
  void unscale_dual_in_place_eq(VectorViewMut<T> /*y*/) const noexcept
  {
  }
  /*!
   * Unscales a primal inequality residual in place.
   */
  template<typename T>
  void unscale_primal_residual_in_place_in(
    VectorViewMut<T> /*x*/) const noexcept
  {
  }
  /*!
   * Unscales a primal equality residual in place.
   */
  template<typename T>
  void unscale_primal_residual_in_place_eq(
    VectorViewMut<T> /*x*/) const noexcept
  {
  }
  /*!
   * Unscales a dual residual in place.
   */
  template<typename T>
  void unscale_dual_residual_in_place(VectorViewMut<T> /*y*/) const noexcept
  {
  }
};
} // namespace preconditioner
} // namespace dense
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_DENSE_PRECOND_IDENTITY_HPP     \
        */
