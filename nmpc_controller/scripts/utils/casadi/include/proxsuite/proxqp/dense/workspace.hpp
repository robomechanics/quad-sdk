//
// Copyright (c) 2022 INRIA
//
/**
 * @file workspace.hpp
 */
#ifndef PROXSUITE_PROXQP_DENSE_WORKSPACE_HPP
#define PROXSUITE_PROXQP_DENSE_WORKSPACE_HPP

#include <Eigen/Core>
#include <proxsuite/linalg/dense/ldlt.hpp>
#include <proxsuite/proxqp/timings.hpp>
#include <proxsuite/linalg/veg/vec.hpp>
// #include <proxsuite/proxqp/dense/preconditioner/ruiz.hpp>

namespace proxsuite {
namespace proxqp {
namespace dense {
///
/// @brief This class defines the workspace of the dense solver.
///
/*!
 * Workspace class of the dense solver.
 */
template<typename T>
struct Workspace
{

  ///// Cholesky Factorization
  proxsuite::linalg::dense::Ldlt<T> ldl{};
  proxsuite::linalg::veg::Vec<unsigned char> ldl_stack;
  Timer<T> timer;

  ///// QP STORAGE
  Mat<T> H_scaled;
  Vec<T> g_scaled;
  Mat<T> A_scaled;
  Mat<T> C_scaled;
  Vec<T> b_scaled;
  Vec<T> u_scaled;
  Vec<T> l_scaled;

  ///// Initial variable loading

  Vec<T> x_prev;
  Vec<T> y_prev;
  Vec<T> z_prev;

  ///// KKT system storage
  Mat<T> kkt;

  //// Active set & permutation vector
  VecISize current_bijection_map;
  VecISize new_bijection_map;

  VecBool active_set_up;
  VecBool active_set_low;
  VecBool active_inequalities;

  //// First order residuals for line search

  Vec<T> Hdx;
  Vec<T> Cdx;
  Vec<T> Adx;

  Vec<T> active_part_z;
  proxsuite::linalg::veg::Vec<T> alphas;

  ///// Newton variables
  Vec<T> dw_aug;
  Vec<T> rhs;
  Vec<T> err;

  //// Relative residuals constants

  T dual_feasibility_rhs_2;
  T correction_guess_rhs_g;
  T correction_guess_rhs_b;
  T alpha;

  Vec<T> dual_residual_scaled;
  Vec<T> primal_residual_eq_scaled;
  Vec<T> primal_residual_in_scaled_up;
  Vec<T> primal_residual_in_scaled_low;

  Vec<T> primal_residual_in_scaled_up_plus_alphaCdx;
  Vec<T> primal_residual_in_scaled_low_plus_alphaCdx;
  Vec<T> CTz;

  bool constraints_changed;
  bool dirty;
  bool refactorize;
  bool proximal_parameter_update;
  bool is_initialized;

  sparse::isize n_c; // final number of active inequalities
  /*!
   * Default constructor.
   * @param dim primal variable dimension.
   * @param n_eq number of equality constraints.
   * @param n_in number of inequality constraints.
   */
  Workspace(isize dim = 0, isize n_eq = 0, isize n_in = 0)
    : //
      // ruiz(preconditioner::RuizEquilibration<T>{dim, n_eq + n_in}),
    ldl{}
    , // old version with alloc
    H_scaled(dim, dim)
    , g_scaled(dim)
    , A_scaled(n_eq, dim)
    , C_scaled(n_in, dim)
    , b_scaled(n_eq)
    , u_scaled(n_in)
    , l_scaled(n_in)
    , x_prev(dim)
    , y_prev(n_eq)
    , z_prev(n_in)
    , kkt(dim + n_eq, dim + n_eq)
    , current_bijection_map(n_in)
    , new_bijection_map(n_in)
    , active_set_up(n_in)
    , active_set_low(n_in)
    , active_inequalities(n_in)
    , Hdx(dim)
    , Cdx(n_in)
    , Adx(n_eq)
    , active_part_z(n_in)
    , dw_aug(dim + n_eq + n_in)
    , rhs(dim + n_eq + n_in)
    , err(dim + n_eq + n_in)
    ,

    dual_residual_scaled(dim)
    , primal_residual_eq_scaled(n_eq)
    , primal_residual_in_scaled_up(n_in)
    , primal_residual_in_scaled_low(n_in)
    ,

    primal_residual_in_scaled_up_plus_alphaCdx(n_in)
    , primal_residual_in_scaled_low_plus_alphaCdx(n_in)
    , CTz(dim)
    , constraints_changed(false)
    , dirty(false)
    , refactorize(false)
    , proximal_parameter_update(false)
    , is_initialized(false)

  {
    ldl.reserve_uninit(dim + n_eq + n_in);
    ldl_stack.resize_for_overwrite(
      proxsuite::linalg::veg::dynstack::StackReq(

        proxsuite::linalg::dense::Ldlt<T>::factorize_req(dim + n_eq + n_in) |

        (proxsuite::linalg::dense::temp_vec_req(
           proxsuite::linalg::veg::Tag<T>{}, n_eq + n_in) &
         proxsuite::linalg::veg::dynstack::StackReq{
           isize{ sizeof(isize) } * (n_eq + n_in), alignof(isize) } &
         proxsuite::linalg::dense::Ldlt<T>::diagonal_update_req(
           dim + n_eq + n_in, n_eq + n_in)) |

        (proxsuite::linalg::dense::temp_mat_req(
           proxsuite::linalg::veg::Tag<T>{}, dim + n_eq + n_in, n_in) &
         proxsuite::linalg::dense::Ldlt<T>::insert_block_at_req(
           dim + n_eq + n_in, n_in)) |

        proxsuite::linalg::dense::Ldlt<T>::solve_in_place_req(dim + n_eq +
                                                              n_in))

        .alloc_req());

    alphas.reserve(2 * n_in);
    H_scaled.setZero();
    g_scaled.setZero();
    A_scaled.setZero();
    C_scaled.setZero();
    b_scaled.setZero();
    u_scaled.setZero();
    l_scaled.setZero();
    x_prev.setZero();
    y_prev.setZero();
    z_prev.setZero();
    kkt.setZero();
    for (isize i = 0; i < n_in; i++) {
      current_bijection_map(i) = i;
      new_bijection_map(i) = i;
    }
    Hdx.setZero();
    Cdx.setZero();
    Adx.setZero();
    active_part_z.setZero();
    dw_aug.setZero();
    rhs.setZero();
    err.setZero();

    dual_feasibility_rhs_2 = 0;
    correction_guess_rhs_g = 0;
    correction_guess_rhs_b = 0;
    alpha = 1.;

    dual_residual_scaled.setZero();
    primal_residual_eq_scaled.setZero();
    primal_residual_in_scaled_up.setZero();
    primal_residual_in_scaled_low.setZero();

    primal_residual_in_scaled_up_plus_alphaCdx.setZero();
    primal_residual_in_scaled_low_plus_alphaCdx.setZero();
    CTz.setZero();
    n_c = 0;
  }
  /*!
   * Clean-ups solver's workspace.
   */
  void cleanup()
  {
    isize n_in = C_scaled.rows();
    H_scaled.setZero();
    g_scaled.setZero();
    A_scaled.setZero();
    C_scaled.setZero();
    b_scaled.setZero();
    u_scaled.setZero();
    l_scaled.setZero();
    Hdx.setZero();
    Cdx.setZero();
    Adx.setZero();
    active_part_z.setZero();
    dw_aug.setZero();
    rhs.setZero();
    err.setZero();

    alpha = 1.;

    dual_residual_scaled.setZero();
    primal_residual_eq_scaled.setZero();
    primal_residual_in_scaled_up.setZero();
    primal_residual_in_scaled_low.setZero();

    primal_residual_in_scaled_up_plus_alphaCdx.setZero();
    primal_residual_in_scaled_low_plus_alphaCdx.setZero();
    CTz.setZero();

    x_prev.setZero();
    y_prev.setZero();
    z_prev.setZero();

    for (isize i = 0; i < n_in; i++) {
      current_bijection_map(i) = i;
      new_bijection_map(i) = i;
      active_inequalities(i) = false;
    }
    constraints_changed = false;
    dirty = false;
    refactorize = false;
    proximal_parameter_update = false;
    is_initialized = false;
    n_c = 0;
  }
};
} // namespace dense
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_DENSE_WORKSPACE_HPP */
