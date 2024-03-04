//
// Copyright (c) 2022 INRIA
//
/** \file */

#ifndef PROXSUITE_PROXQP_SPARSE_SOLVER_HPP
#define PROXSUITE_PROXQP_SPARSE_SOLVER_HPP

#include <chrono>
#include <cmath>

#include <proxsuite/linalg/dense/core.hpp>
#include <proxsuite/linalg/sparse/core.hpp>
#include <proxsuite/linalg/sparse/factorize.hpp>
#include <proxsuite/linalg/sparse/update.hpp>
#include <proxsuite/linalg/sparse/rowmod.hpp>
#include <proxsuite/proxqp/dense/views.hpp>
#include <proxsuite/proxqp/settings.hpp>
#include <proxsuite/linalg/veg/vec.hpp>
#include "proxsuite/proxqp/results.hpp"
#include "proxsuite/proxqp/sparse/fwd.hpp"
#include "proxsuite/proxqp/sparse/views.hpp"
#include "proxsuite/proxqp/sparse/model.hpp"
#include "proxsuite/proxqp/sparse/workspace.hpp"
#include "proxsuite/proxqp/sparse/utils.hpp"
#include "proxsuite/proxqp/sparse/preconditioner/ruiz.hpp"
#include "proxsuite/proxqp/sparse/preconditioner/identity.hpp"

#include <iostream>
#include <iomanip>
#include <Eigen/IterativeLinearSolvers>
#include <unsupported/Eigen/IterativeSolvers>

namespace proxsuite {
namespace proxqp {
namespace sparse {

template<typename T, typename I>
void
ldl_solve(VectorViewMut<T> sol,
          VectorView<T> rhs,
          isize n_tot,
          proxsuite::linalg::sparse::MatMut<T, I> ldl,
          Eigen::MINRES<detail::AugmentedKkt<T, I>,
                        Eigen::Upper | Eigen::Lower,
                        Eigen::IdentityPreconditioner>& iterative_solver,
          bool do_ldlt,
          proxsuite::linalg::veg::dynstack::DynStackMut stack,
          T* ldl_values,
          I* perm,
          I* ldl_col_ptrs,
          I const* perm_inv)
{
  LDLT_TEMP_VEC_UNINIT(T, work_, n_tot, stack);
  auto rhs_e = rhs.to_eigen();
  auto sol_e = sol.to_eigen();
  auto zx = proxsuite::linalg::sparse::util::zero_extend;

  if (do_ldlt) {

    for (isize i = 0; i < n_tot; ++i) {
      work_[i] = rhs_e[isize(zx(perm[i]))];
    }

    proxsuite::linalg::sparse::dense_lsolve<T, I>( //
      { proxsuite::linalg::sparse::from_eigen, work_ },
      ldl.as_const());

    for (isize i = 0; i < n_tot; ++i) {
      work_[i] /= ldl_values[isize(zx(ldl_col_ptrs[i]))];
    }

    proxsuite::linalg::sparse::dense_ltsolve<T, I>( //
      { proxsuite::linalg::sparse::from_eigen, work_ },
      ldl.as_const());

    for (isize i = 0; i < n_tot; ++i) {
      sol_e[i] = work_[isize(zx(perm_inv[i]))];
    }
  } else {
    work_ = iterative_solver.solve(rhs_e);
    sol_e = work_;
  }
}

template<typename T, typename I>
void
ldl_iter_solve_noalias(
  VectorViewMut<T> sol,
  VectorView<T> rhs,
  VectorView<T> init_guess,
  Results<T> const& results,
  Model<T, I> const& data,
  isize n_tot,
  proxsuite::linalg::sparse::MatMut<T, I> ldl,
  Eigen::MINRES<detail::AugmentedKkt<T, I>,
                Eigen::Upper | Eigen::Lower,
                Eigen::IdentityPreconditioner>& iterative_solver,
  bool do_ldlt,
  proxsuite::linalg::veg::dynstack::DynStackMut stack,
  T* ldl_values,
  I* perm,
  I* ldl_col_ptrs,
  I const* perm_inv,
  Settings<T> const& settings,
  proxsuite::linalg::sparse::MatMut<T, I> kkt_active,
  proxsuite::linalg::veg::SliceMut<bool> active_constraints)
{
  auto rhs_e = rhs.to_eigen();
  auto sol_e = sol.to_eigen();

  if (init_guess.dim == sol.dim) {
    sol_e = init_guess.to_eigen();
  } else {
    sol_e.setZero();
  }

  LDLT_TEMP_VEC_UNINIT(T, err, n_tot, stack);

  T prev_err_norm = std::numeric_limits<T>::infinity();

  for (isize solve_iter = 0; solve_iter < settings.nb_iterative_refinement;
       ++solve_iter) {

    auto err_x = err.head(data.dim);
    auto err_y = err.segment(data.dim, data.n_eq);
    auto err_z = err.tail(data.n_in);

    auto sol_x = sol_e.head(data.dim);
    auto sol_y = sol_e.segment(data.dim, data.n_eq);
    auto sol_z = sol_e.tail(data.n_in); // removed active set condition

    err = -rhs_e;

    if (solve_iter > 0) {
      T mu_eq_neg = -results.info.mu_eq;
      T mu_in_neg = -results.info.mu_in;
      detail::noalias_symhiv_add(err, kkt_active.to_eigen(), sol_e);
      err_x += results.info.rho * sol_x;
      err_y += mu_eq_neg * sol_y;
      for (isize i = 0; i < data.n_in; ++i) {
        err_z[i] += (active_constraints[i] ? mu_in_neg : T(1)) * sol_z[i];
      }
    }

    T err_norm = infty_norm(err);
    if (err_norm > prev_err_norm / T(2)) {
      break;
    }
    prev_err_norm = err_norm;

    ldl_solve({ proxqp::from_eigen, err },
              { proxqp::from_eigen, err },
              n_tot,
              ldl,
              iterative_solver,
              do_ldlt,
              stack,
              ldl_values,
              perm,
              ldl_col_ptrs,
              perm_inv);

    sol_e -= err;
  }
}
/*!
 * Solves in place a linear system.
 *
 * @param rhs right hand side vector of the linear system to solver.
 * @param init_guess initial guess for solving the linear system
 * @param ldl current ldlt.
 * @param do_ldlt boolean variable for doing the ldlt (rather than MinRes
 * algorithm).
 * @param perm_inv pointer to the inverse of the permutation.
 * @param results solver results.
 * @param data model of the QP.
 * @param n_tot dimension of the KKT matrix
 * @param kkt_active active part of the KKT matrix.
 * @param active_constraints vector boolean precising whether the constraints
 * are active or not.
 * @param iterative_solver iterative solver matrix free.
 * @param stack memory stack.
 * @param ldl_values pointor to ldl values.
 * @param perm pointor to the ldl permutation.
 * @param ldl_col_ptrs pointor to the column of the ldl.
 * @param perm_inv pointor the inverse permutation.
 * @param settings solver's settings.
 * @param kkt_active active part of the kkt.
 */
template<typename T, typename I>
void
ldl_solve_in_place(
  VectorViewMut<T> rhs,
  VectorView<T> init_guess,
  Results<T> const& results,
  Model<T, I> const& data,
  isize n_tot,
  proxsuite::linalg::sparse::MatMut<T, I> ldl,
  Eigen::MINRES<detail::AugmentedKkt<T, I>,
                Eigen::Upper | Eigen::Lower,
                Eigen::IdentityPreconditioner>& iterative_solver,
  bool do_ldlt,
  proxsuite::linalg::veg::dynstack::DynStackMut stack,
  T* ldl_values,
  I* perm,
  I* ldl_col_ptrs,
  I const* perm_inv,
  Settings<T> const& settings,
  proxsuite::linalg::sparse::MatMut<T, I> kkt_active,
  proxsuite::linalg::veg::SliceMut<bool> active_constraints)
{
  LDLT_TEMP_VEC_UNINIT(T, tmp, n_tot, stack);
  ldl_iter_solve_noalias({ proxqp::from_eigen, tmp },
                         rhs.as_const(),
                         init_guess,
                         results,
                         data,
                         n_tot,
                         ldl,
                         iterative_solver,
                         do_ldlt,
                         stack,
                         ldl_values,
                         perm,
                         ldl_col_ptrs,
                         perm_inv,
                         settings,
                         kkt_active,
                         active_constraints);
  rhs.to_eigen() = tmp;
}
/*!
 * Reconstructs manually the permutted matrix.
 *
 * @param ldl current ldlt.
 * @param do_ldlt boolean variable for doing the ldlt (rather than MinRes
 * algorithm).
 */
template<typename T, typename I>
auto
inner_reconstructed_matrix(proxsuite::linalg::sparse::MatMut<T, I> ldl)
  -> DMat<T>
{
  auto ldl_dense = ldl.to_eigen().toDense();
  auto l = DMat<T>(ldl_dense.template triangularView<Eigen::UnitLower>());
  auto lt = l.transpose();
  auto d = ldl_dense.diagonal().asDiagonal();
  auto mat = DMat<T>(l * d * lt);
  return mat;
}
/*!
 * Reconstructs manually the value of the KKT matrix.
 *
 * @param ldl current ldlt.
 * @param do_ldlt boolean variable for doing the ldlt (rather than MinRes
 * algorithm).
 * @param perm_inv pointer to the inverse of the permutation.
 * @param n_tot dimension of the KKT matrix
 */
template<typename T, typename I>
auto
reconstructed_matrix(proxsuite::linalg::sparse::MatMut<T, I> ldl,
                     I const* perm_inv,
                     isize n_tot) -> DMat<T>
{
  auto mat = inner_reconstructed_matrix(ldl);
  auto mat_backup = mat;
  for (isize i = 0; i < n_tot; ++i) {
    for (isize j = 0; j < n_tot; ++j) {
      mat(i, j) = mat_backup(perm_inv[i], perm_inv[j]);
    }
  }
  return mat;
}
/*!
 * Derives the norm of the difference between current KKT and the one it should
 * be (derived manually).
 *
 * @param ldl current ldlt.
 * @param perm_inv pointer to the inverse of the permutation.
 * @param results solver results.
 * @param data model of the QP.
 * @param n_tot dimension of the KKT matrix
 * @param kkt_active active part of the KKT matrix.
 * @param active_constraints vector boolean precising whether the constraints
 * are active or not.
 */
template<typename T, typename I>
auto
reconstruction_error(proxsuite::linalg::sparse::MatMut<T, I> ldl,
                     I const* perm_inv,
                     Results<T> const& results,
                     Model<T, I> const& data,
                     isize n_tot,
                     proxsuite::linalg::sparse::MatMut<T, I> kkt_active,
                     proxsuite::linalg::veg::SliceMut<bool> active_constraints)
  -> DMat<T>
{
  T mu_eq_neg = -results.info.mu_eq;
  T mu_in_neg = -results.info.mu_in;
  auto diff = DMat<T>(
    reconstructed_matrix(ldl, perm_inv, n_tot) -
    DMat<T>(
      DMat<T>(kkt_active.to_eigen()).template selfadjointView<Eigen::Upper>()));
  diff.diagonal().head(data.dim).array() -= results.info.rho;
  diff.diagonal().segment(data.dim, data.n_eq).array() -= mu_eq_neg;
  for (isize i = 0; i < data.n_in; ++i) {
    diff.diagonal()[data.dim + data.n_eq + i] -=
      active_constraints[i] ? mu_in_neg : T(1);
  }
  return diff;
}

template<typename T>
struct PrimalDualGradResult
{
  T a;
  T b;
  T grad;
  VEG_REFLECT(PrimalDualGradResult, a, b, grad);
};

/*!
 * Executes the PROXQP algorithm.
 *
 * @param results solver results.
 * @param data QP problem model as defined by the user (without any scaling
 * performed).
 * @param settings solver settings.
 * @param work solver workspace.
 * @param precond preconditioner.
 */
template<typename T, typename I, typename P>
void
qp_solve(Results<T>& results,
         Model<T, I>& data,
         const Settings<T>& settings,
         Workspace<T, I>& work,
         P& precond)
{
  if (settings.compute_timings) {
    work.timer.stop();
    work.timer.start();
  }

  if (work.internal
        .dirty) // the following is used when a solve has already been executed
                // (and without any intermediary model update)
  {
    proxsuite::linalg::sparse::MatMut<T, I> kkt_unscaled =
      data.kkt_mut_unscaled();

    auto kkt_top_n_rows = detail::top_rows_mut_unchecked(
      proxsuite::linalg::veg::unsafe, kkt_unscaled, data.dim);

    proxsuite::linalg::sparse::MatMut<T, I> H_unscaled =
      detail::middle_cols_mut(kkt_top_n_rows, 0, data.dim, data.H_nnz);

    proxsuite::linalg::sparse::MatMut<T, I> AT_unscaled =
      detail::middle_cols_mut(kkt_top_n_rows, data.dim, data.n_eq, data.A_nnz);

    proxsuite::linalg::sparse::MatMut<T, I> CT_unscaled =
      detail::middle_cols_mut(
        kkt_top_n_rows, data.dim + data.n_eq, data.n_in, data.C_nnz);

    SparseMat<T, I> H_triu =
      H_unscaled.to_eigen().template triangularView<Eigen::Upper>();
    sparse::QpView<T, I> qp = {
      { proxsuite::linalg::sparse::from_eigen, H_triu },
      { proxsuite::linalg::sparse::from_eigen, data.g },
      { proxsuite::linalg::sparse::from_eigen, AT_unscaled.to_eigen() },
      { proxsuite::linalg::sparse::from_eigen, data.b },
      { proxsuite::linalg::sparse::from_eigen, CT_unscaled.to_eigen() },
      { proxsuite::linalg::sparse::from_eigen, data.l },
      { proxsuite::linalg::sparse::from_eigen, data.u }
    };

    switch (settings.initial_guess) { // the following is used when one solve
                                      // has already been executed
      case InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS: {
        results.cleanup(settings);
        break;
      }
      case InitialGuessStatus::COLD_START_WITH_PREVIOUS_RESULT: {
        // keep solutions but restart workspace and results
        results.cold_start(settings);
        precond.scale_primal_in_place(
          { proxsuite::proxqp::from_eigen, results.x });
        precond.scale_dual_in_place_eq(
          { proxsuite::proxqp::from_eigen, results.y });
        precond.scale_dual_in_place_in(
          { proxsuite::proxqp::from_eigen, results.z });
        break;
      }
      case InitialGuessStatus::NO_INITIAL_GUESS: {
        results.cleanup(settings);
        break;
      }
      case InitialGuessStatus::WARM_START: {
        results.cold_start(settings); // because there was already a solve,
                                      // precond was already computed if set so
        precond.scale_primal_in_place(
          { proxsuite::proxqp::from_eigen,
            results.x }); // it contains the value given in entry for warm start
        precond.scale_dual_in_place_eq(
          { proxsuite::proxqp::from_eigen, results.y });
        precond.scale_dual_in_place_in(
          { proxsuite::proxqp::from_eigen, results.z });
        break;
      }
      case InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT: {
        // keep workspace and results solutions except statistics
        results.cleanup_statistics();
        precond.scale_primal_in_place(
          { proxsuite::proxqp::from_eigen, results.x });
        precond.scale_dual_in_place_eq(
          { proxsuite::proxqp::from_eigen, results.y });
        precond.scale_dual_in_place_in(
          { proxsuite::proxqp::from_eigen, results.z });
        break;
      }
    }
    work.setup_impl(
      qp,
      data,
      settings,
      false,
      precond,
      P::scale_qp_in_place_req(
        proxsuite::linalg::veg::Tag<T>{}, data.dim, data.n_eq, data.n_in));

  } else {
    // the following is used for a first solve after initializing or updating
    // the Qp object
    switch (settings.initial_guess) {
      case InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS: {
        break;
      }
      case InitialGuessStatus::COLD_START_WITH_PREVIOUS_RESULT: {
        precond.scale_primal_in_place(
          { proxsuite::proxqp::from_eigen,
            results.x }); // meaningful for when there is an upate of the model
                          // and one wants to warm start with previous result
        precond.scale_dual_in_place_eq(
          { proxsuite::proxqp::from_eigen, results.y });
        precond.scale_dual_in_place_in(
          { proxsuite::proxqp::from_eigen, results.z });
        break;
      }
      case InitialGuessStatus::NO_INITIAL_GUESS: {
        break;
      }
      case InitialGuessStatus::WARM_START: {
        precond.scale_primal_in_place(
          { proxsuite::proxqp::from_eigen, results.x });
        precond.scale_dual_in_place_eq(
          { proxsuite::proxqp::from_eigen, results.y });
        precond.scale_dual_in_place_in(
          { proxsuite::proxqp::from_eigen, results.z });
        break;
      }
      case InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT: {
        precond.scale_primal_in_place(
          { proxsuite::proxqp::from_eigen,
            results.x }); // meaningful for when there is an upate of the model
                          // and one wants to warm start with previous result
        precond.scale_dual_in_place_eq(
          { proxsuite::proxqp::from_eigen, results.y });
        precond.scale_dual_in_place_in(
          { proxsuite::proxqp::from_eigen, results.z });
        break;
      }
    }
  }

  if (settings.verbose) {
    sparse::print_setup_header(settings, results, data);
  }
  using namespace proxsuite::linalg::veg::literals;
  namespace util = proxsuite::linalg::sparse::util;
  auto zx = util::zero_extend;

  proxsuite::linalg::veg::dynstack::DynStackMut stack = work.stack_mut();

  isize n = data.dim;
  isize n_eq = data.n_eq;
  isize n_in = data.n_in;
  isize n_tot = n + n_eq + n_in;

  VectorViewMut<T> x{ proxqp::from_eigen, results.x };
  VectorViewMut<T> y{ proxqp::from_eigen, results.y };
  VectorViewMut<T> z{ proxqp::from_eigen, results.z };

  proxsuite::linalg::sparse::MatMut<T, I> kkt = data.kkt_mut();

  auto kkt_top_n_rows =
    detail::top_rows_mut_unchecked(proxsuite::linalg::veg::unsafe, kkt, n);

  proxsuite::linalg::sparse::MatMut<T, I> H_scaled =
    detail::middle_cols_mut(kkt_top_n_rows, 0, n, data.H_nnz);

  proxsuite::linalg::sparse::MatMut<T, I> AT_scaled =
    detail::middle_cols_mut(kkt_top_n_rows, n, n_eq, data.A_nnz);

  proxsuite::linalg::sparse::MatMut<T, I> CT_scaled =
    detail::middle_cols_mut(kkt_top_n_rows, n + n_eq, n_in, data.C_nnz);

  auto& g_scaled_e = work.internal.g_scaled;
  auto& b_scaled_e = work.internal.b_scaled;
  auto& l_scaled_e = work.internal.l_scaled;
  auto& u_scaled_e = work.internal.u_scaled;

  QpViewMut<T, I> qp_scaled = {
    H_scaled,
    { proxsuite::linalg::sparse::from_eigen, g_scaled_e },
    AT_scaled,
    { proxsuite::linalg::sparse::from_eigen, b_scaled_e },
    CT_scaled,
    { proxsuite::linalg::sparse::from_eigen, l_scaled_e },
    { proxsuite::linalg::sparse::from_eigen, u_scaled_e },
  };

  T const dual_feasibility_rhs_2 = infty_norm(data.g);

  // auto ldl_col_ptrs = work.ldl_col_ptrs_mut();
  auto ldl_col_ptrs = work.internal.ldl.col_ptrs.ptr_mut();
  proxsuite::linalg::veg::Tag<I> itag;
  proxsuite::linalg::veg::Tag<T> xtag;

  bool do_ldlt = work.internal.do_ldlt;

  isize ldlt_ntot = do_ldlt ? n_tot : 0;

  auto _perm = stack.make_new_for_overwrite(itag, ldlt_ntot);

  I* perm_inv = work.internal.ldl.perm_inv.ptr_mut();
  I* perm = _perm.ptr_mut();

  if (do_ldlt) {
    // compute perm from perm_inv
    for (isize i = 0; i < n_tot; ++i) {
      perm[isize(zx(perm_inv[i]))] = I(i);
    }
  }

  I* kkt_nnz_counts = work.internal.kkt_nnz_counts.ptr_mut();

  auto& iterative_solver = *work.internal.matrix_free_solver.get();
  isize C_active_nnz = 0;
  switch (settings.initial_guess) {
    case InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS: {
      // H and A are always active
      for (usize j = 0; j < usize(n + n_eq); ++j) {
        kkt_nnz_counts[isize(j)] = I(kkt.col_end(j) - kkt.col_start(j));
      }
      // ineq constraints initially inactive
      for (isize j = 0; j < n_in; ++j) {
        kkt_nnz_counts[n + n_eq + j] = 0;
        work.active_inequalities[j] = false;
      }
      break;
    }
    case InitialGuessStatus::COLD_START_WITH_PREVIOUS_RESULT: {
      // keep solutions + restart workspace and results except rho and mu : done
      // in setup

      // H and A are always active
      for (usize j = 0; j < usize(n + n_eq); ++j) {
        kkt_nnz_counts[isize(j)] = I(kkt.col_end(j) - kkt.col_start(j));
      }
      // keep constraints inactive from previous solution
      for (isize j = 0; j < n_in; ++j) {
        if (results.z(j) != 0) {
          kkt_nnz_counts[n + n_eq + j] = I(kkt.col_end(usize(n + n_eq + j)) -
                                           kkt.col_start(usize(n + n_eq + j)));
          work.active_inequalities[j] = true;
          C_active_nnz += kkt_nnz_counts[n + n_eq + j];
        } else {
          kkt_nnz_counts[n + n_eq + j] = 0;
          work.active_inequalities[j] = false;
        }
      }
      break;
    }
    case InitialGuessStatus::NO_INITIAL_GUESS: {
      // already set to zero in the setup
      // H and A are always active
      for (usize j = 0; j < usize(n + n_eq); ++j) {
        kkt_nnz_counts[isize(j)] = I(kkt.col_end(j) - kkt.col_start(j));
      }
      // ineq constraints initially inactive
      for (isize j = 0; j < n_in; ++j) {
        kkt_nnz_counts[n + n_eq + j] = 0;
        work.active_inequalities[j] = false;
      }
      break;
    }
    case InitialGuessStatus::WARM_START: {
      // keep previous solution

      // H and A are always active
      for (usize j = 0; j < usize(n + n_eq); ++j) {
        kkt_nnz_counts[isize(j)] = I(kkt.col_end(j) - kkt.col_start(j));
      }
      // keep constraints inactive from previous solution
      for (isize j = 0; j < n_in; ++j) {
        if (results.z(j) != 0) {
          kkt_nnz_counts[n + n_eq + j] = I(kkt.col_end(usize(n + n_eq + j)) -
                                           kkt.col_start(usize(n + n_eq + j)));
          work.active_inequalities[j] = true;
          C_active_nnz += kkt_nnz_counts[n + n_eq + j];

        } else {
          kkt_nnz_counts[n + n_eq + j] = 0;
          work.active_inequalities[j] = false;
        }
      }
      break;
    }
    case InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT: {
      // keep workspace and results solutions except statistics
      // H and A are always active
      for (usize j = 0; j < usize(n + n_eq); ++j) {
        kkt_nnz_counts[isize(j)] = I(kkt.col_end(j) - kkt.col_start(j));
      }
      // keep constraints inactive from previous solution
      for (isize j = 0; j < n_in; ++j) {
        if (results.z(j) != 0) {
          kkt_nnz_counts[n + n_eq + j] = I(kkt.col_end(usize(n + n_eq + j)) -
                                           kkt.col_start(usize(n + n_eq + j)));
          work.active_inequalities[j] = true;
          C_active_nnz += kkt_nnz_counts[n + n_eq + j];
        } else {
          kkt_nnz_counts[n + n_eq + j] = 0;
          work.active_inequalities[j] = false;
        }
      }
      break;
    }
  }

  proxsuite::linalg::sparse::MatMut<T, I> kkt_active = {
    proxsuite::linalg::sparse::from_raw_parts,
    n_tot,
    n_tot,
    data.H_nnz + data.A_nnz + C_active_nnz,
    kkt.col_ptrs_mut(),
    kkt_nnz_counts,
    kkt.row_indices_mut(),
    kkt.values_mut(),
  };

  I* etree = work.internal.ldl.etree.ptr_mut();
  I* ldl_nnz_counts = work.internal.ldl.nnz_counts.ptr_mut();
  I* ldl_row_indices = work.internal.ldl.row_indices.ptr_mut();
  T* ldl_values = work.internal.ldl.values.ptr_mut();
  proxsuite::linalg::veg::SliceMut<bool> active_constraints =
    work.active_inequalities.as_mut();

  proxsuite::linalg::sparse::MatMut<T, I> ldl = {
    proxsuite::linalg::sparse::from_raw_parts,
    n_tot,
    n_tot,
    0,
    ldl_col_ptrs,
    do_ldlt ? ldl_nnz_counts : nullptr, // si do_ldlt est vrai do ldl_nnz_counts
    ldl_row_indices,
    ldl_values,
  };

  T bcl_eta_ext_init = pow(T(0.1), settings.alpha_bcl);
  T bcl_eta_ext = bcl_eta_ext_init;
  T bcl_eta_in(1);
  T eps_in_min = std::min(settings.eps_abs, T(1e-9));

  auto x_e = x.to_eigen();
  auto y_e = y.to_eigen();
  auto z_e = z.to_eigen();
  sparse::refactorize<T, I>(
    work, results, kkt_active, active_constraints, data, stack, xtag);
  switch (settings.initial_guess) {
    case InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS: {
      LDLT_TEMP_VEC_UNINIT(T, rhs, n_tot, stack);
      LDLT_TEMP_VEC_UNINIT(T, no_guess, 0, stack);

      rhs.head(n) = -g_scaled_e;
      rhs.segment(n, n_eq) = b_scaled_e;
      rhs.segment(n + n_eq, n_in).setZero();

      ldl_solve_in_place({ proxqp::from_eigen, rhs },
                         { proxqp::from_eigen, no_guess },
                         results,
                         data,
                         n_tot,
                         ldl,
                         iterative_solver,
                         do_ldlt,
                         stack,
                         ldl_values,
                         perm,
                         ldl_col_ptrs,
                         perm_inv,
                         settings,
                         kkt_active,
                         active_constraints);
      x_e = rhs.head(n);
      y_e = rhs.segment(n, n_eq);
      z_e = rhs.segment(n + n_eq, n_in);
      break;
    }
    case InitialGuessStatus::COLD_START_WITH_PREVIOUS_RESULT: {
      // keep solutions but restart workspace and results
      break;
    }
    case InitialGuessStatus::NO_INITIAL_GUESS: {
      // already set to zero in the setup
      break;
    }
    case InitialGuessStatus::WARM_START: {
      // keep previous solution
      break;
    }
    case InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT: {
      // keep workspace and results solutions except statistics
      break;
    }
  }
  T rhs_duality_gap(0);

  for (isize iter = 0; iter < settings.max_iter; ++iter) {

    results.info.iter_ext += 1;
    if (iter == settings.max_iter) {
      break;
    }
    T new_bcl_mu_eq = results.info.mu_eq;
    T new_bcl_mu_in = results.info.mu_in;
    T new_bcl_mu_eq_inv = results.info.mu_eq_inv;
    T new_bcl_mu_in_inv = results.info.mu_in_inv;

    {
      T primal_feasibility_eq_rhs_0;
      T primal_feasibility_in_rhs_0;

      T dual_feasibility_rhs_0(0);
      T dual_feasibility_rhs_1(0);
      T dual_feasibility_rhs_3(0);

      LDLT_TEMP_VEC_UNINIT(T, primal_residual_eq_scaled, n_eq, stack);
      LDLT_TEMP_VEC_UNINIT(T, primal_residual_in_scaled_lo, n_in, stack);
      LDLT_TEMP_VEC_UNINIT(T, primal_residual_in_scaled_up, n_in, stack);

      LDLT_TEMP_VEC_UNINIT(T, dual_residual_scaled, n, stack);

      // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
      auto is_primal_feasible = [&](T primal_feasibility_lhs) -> bool {
        T rhs_pri = settings.eps_abs;
        if (settings.eps_rel != 0) {
          rhs_pri +=
            settings.eps_rel * std::max({ primal_feasibility_eq_rhs_0,
                                          primal_feasibility_in_rhs_0 });
        }
        return primal_feasibility_lhs <= rhs_pri;
      };
      auto is_dual_feasible = [&](T dual_feasibility_lhs) -> bool {
        T rhs_dua = settings.eps_abs;
        if (settings.eps_rel != 0) {
          rhs_dua += settings.eps_rel * std::max({
                                          dual_feasibility_rhs_0,
                                          dual_feasibility_rhs_1,
                                          dual_feasibility_rhs_2,
                                          dual_feasibility_rhs_3,
                                        });
        }

        return dual_feasibility_lhs <= rhs_dua;
      };
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

      VEG_BIND( // ?
        auto,
        (primal_feasibility_lhs, dual_feasibility_lhs),
        detail::unscaled_primal_dual_residual(work,
                                              results,
                                              primal_residual_eq_scaled,
                                              primal_residual_in_scaled_lo,
                                              primal_residual_in_scaled_up,
                                              dual_residual_scaled,
                                              primal_feasibility_eq_rhs_0,
                                              primal_feasibility_in_rhs_0,
                                              dual_feasibility_rhs_0,
                                              dual_feasibility_rhs_1,
                                              dual_feasibility_rhs_3,
                                              rhs_duality_gap,
                                              precond,
                                              data,
                                              qp_scaled.as_const(),
                                              detail::vec_mut(x_e),
                                              detail::vec_mut(y_e),
                                              detail::vec_mut(z_e),
                                              stack));
      /*put in debug mode
      if (settings.verbose) {
              std::cout << "-------- outer iteration: " << iter << " primal
      residual "
                                                      << primal_feasibility_lhs
      << " dual residual "
                                                      << dual_feasibility_lhs <<
      " mu_in " << results.info.mu_in
                                                      << " bcl_eta_ext " <<
      bcl_eta_ext << " bcl_eta_in "
                                                      << bcl_eta_in <<
      std::endl;
      }
      */
      if (settings.verbose) {
        LDLT_TEMP_VEC_UNINIT(T, tmp, n, stack);
        tmp.setZero();
        detail::noalias_symhiv_add(tmp, qp_scaled.H.to_eigen(), x_e);
        precond.unscale_dual_residual_in_place({ proxqp::from_eigen, tmp });

        precond.unscale_primal_in_place({ proxqp::from_eigen, x_e });
        precond.unscale_dual_in_place_eq({ proxqp::from_eigen, y_e });
        precond.unscale_dual_in_place_in({ proxqp::from_eigen, z_e });
        tmp *= 0.5;
        tmp += data.g;
        results.info.objValue = (tmp).dot(x_e);
        std::cout << "\033[1;32m[outer iteration " << iter + 1 << "]\033[0m"
                  << std::endl;
        std::cout << std::scientific << std::setw(2) << std::setprecision(2)
                  << "| primal residual=" << primal_feasibility_lhs
                  << " | dual residual=" << dual_feasibility_lhs
                  << " | duality gap=" << results.info.duality_gap
                  << " | mu_in=" << results.info.mu_in
                  << " | rho=" << results.info.rho << std::endl;
        results.info.pri_res = primal_feasibility_lhs;
        results.info.dua_res = dual_feasibility_lhs;
        precond.scale_primal_in_place(VectorViewMut<T>{ from_eigen, x_e });
        precond.scale_dual_in_place_eq(VectorViewMut<T>{ from_eigen, y_e });
        precond.scale_dual_in_place_in(VectorViewMut<T>{ from_eigen, z_e });
      }
      if (is_primal_feasible(primal_feasibility_lhs) &&
          is_dual_feasible(dual_feasibility_lhs)) {
        if (settings.check_duality_gap) {
          if (std::fabs(results.info.duality_gap) <=
              settings.eps_duality_gap_abs +
                settings.eps_duality_gap_rel * rhs_duality_gap) {
            results.info.pri_res = primal_feasibility_lhs;
            results.info.dua_res = dual_feasibility_lhs;
            results.info.status = QPSolverOutput::PROXQP_SOLVED;
            break;
          }
        } else {
          results.info.pri_res = primal_feasibility_lhs;
          results.info.dua_res = dual_feasibility_lhs;
          results.info.status = QPSolverOutput::PROXQP_SOLVED;
          break;
        }
      }

      LDLT_TEMP_VEC_UNINIT(T, x_prev_e, n, stack);
      LDLT_TEMP_VEC_UNINIT(T, y_prev_e, n_eq, stack);
      LDLT_TEMP_VEC_UNINIT(T, z_prev_e, n_in, stack);
      LDLT_TEMP_VEC(T, dw_prev, n_tot, stack);

      x_prev_e = x_e;
      y_prev_e = y_e;
      z_prev_e = z_e;

      // Cx + 1/mu_in * z_prev
      primal_residual_in_scaled_up += results.info.mu_in * z_prev_e;
      primal_residual_in_scaled_lo = primal_residual_in_scaled_up;

      // Cx - l + 1/mu_in * z_prev
      primal_residual_in_scaled_lo -= l_scaled_e;

      // Cx - u + 1/mu_in * z_prev
      primal_residual_in_scaled_up -= u_scaled_e;

      // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
      auto primal_dual_newton_semi_smooth = [&]() -> void {
        for (isize iter_inner = 0; iter_inner < settings.max_iter_in;
             ++iter_inner) {
          LDLT_TEMP_VEC_UNINIT(T, dw, n_tot, stack);

          if (iter_inner == settings.max_iter_in - 1) {
            results.info.iter += settings.max_iter_in;
            break;
          }

          // primal_dual_semi_smooth_newton_step
          {
            LDLT_TEMP_VEC_UNINIT(bool, new_active_constraints, n_in, stack);
            auto rhs = dw;

            work.active_set_low.array() =
              primal_residual_in_scaled_lo.array() <= 0;
            work.active_set_up.array() =
              primal_residual_in_scaled_up.array() >= 0;
            new_active_constraints = work.active_set_low || work.active_set_up;

            // active set change
            if (n_in > 0) {
              bool removed = false;
              bool added = false;

              for (isize i = 0; i < n_in; ++i) {
                bool was_active = active_constraints[i];
                bool is_active = new_active_constraints[i];

                isize idx = n + n_eq + i;

                usize col_nnz =
                  zx(kkt.col_end(usize(idx))) - zx(kkt.col_start(usize(idx)));

                if (is_active && !was_active) {
                  added = true;

                  kkt_active.nnz_per_col_mut()[idx] = I(col_nnz);
                  kkt_active._set_nnz(kkt_active.nnz() + isize(col_nnz));

                  if (do_ldlt) {
                    proxsuite::linalg::sparse::VecRef<T, I> new_col{
                      proxsuite::linalg::sparse::from_raw_parts,
                      n_tot,
                      isize(col_nnz),
                      kkt.row_indices() + zx(kkt.col_start(usize(idx))),
                      kkt.values() + zx(kkt.col_start(usize(idx))),
                    };

                    ldl =
                      proxsuite::linalg::sparse::add_row(ldl,
                                                         etree,
                                                         perm_inv,
                                                         idx,
                                                         new_col,
                                                         -results.info.mu_in,
                                                         stack);
                  }
                  active_constraints[i] = new_active_constraints[i];

                } else if (!is_active && was_active) {
                  removed = true;
                  kkt_active.nnz_per_col_mut()[idx] = 0;
                  kkt_active._set_nnz(kkt_active.nnz() - isize(col_nnz));
                  if (do_ldlt) {
                    ldl = proxsuite::linalg::sparse::delete_row(
                      ldl, etree, perm_inv, idx, stack);
                  }
                  active_constraints[i] = new_active_constraints[i];
                }
              }

              if (!do_ldlt) {
                if (removed || added) {
                  refactorize(work,
                              results,
                              kkt_active,
                              active_constraints,
                              data,
                              stack,
                              xtag);
                }
              }
            }

            rhs.head(n) = -dual_residual_scaled;
            rhs.segment(n, n_eq) = -primal_residual_eq_scaled;

            for (isize i = 0; i < n_in; ++i) {
              if (work.active_set_up(i)) {
                rhs(n + n_eq + i) =
                  results.info.mu_in * z_e(i) - primal_residual_in_scaled_up(i);
              } else if (work.active_set_low(i)) {
                rhs(n + n_eq + i) =
                  results.info.mu_in * z_e(i) - primal_residual_in_scaled_lo(i);
              } else {
                rhs(n + n_eq + i) = -z_e(i);
                rhs.head(n) += z_e(i) * CT_scaled.to_eigen().col(i);
              }
            }

            ldl_solve_in_place(
              { proxqp::from_eigen, rhs },
              { proxqp::from_eigen,
                dw_prev }, // todo: MAJ dw_prev avec dw pour avoir meilleur
                           // guess sur les solve in place
              results,
              data,
              n_tot,
              ldl,
              iterative_solver,
              do_ldlt,
              stack,
              ldl_values,
              perm,
              ldl_col_ptrs,
              perm_inv,
              settings,
              kkt_active,
              active_constraints);
          }
          auto dx = dw.head(n);
          auto dy = dw.segment(n, n_eq);
          auto dz = dw.segment(n + n_eq, n_in);

          LDLT_TEMP_VEC(T, Hdx, n, stack);
          LDLT_TEMP_VEC(T, Adx, n_eq, stack);
          LDLT_TEMP_VEC(T, Cdx, n_in, stack);

          LDLT_TEMP_VEC(T, ATdy, n, stack);
          LDLT_TEMP_VEC(T, CTdz, n, stack);

          detail::noalias_symhiv_add(Hdx, H_scaled.to_eigen(), dx);
          detail::noalias_gevmmv_add(Adx, ATdy, AT_scaled.to_eigen(), dx, dy);
          detail::noalias_gevmmv_add(Cdx, CTdz, CT_scaled.to_eigen(), dx, dz);

          T alpha = 1;
          // primal dual line search
          if (n_in > 0) {
            auto primal_dual_gradient_norm =
              [&](T alpha_cur) -> PrimalDualGradResult<T> {
              LDLT_TEMP_VEC_UNINIT(T, Cdx_active, n_in, stack);
              LDLT_TEMP_VEC_UNINIT(T, active_part_z, n_in, stack);
              {
                LDLT_TEMP_VEC_UNINIT(T, tmp_lo, n_in, stack);
                LDLT_TEMP_VEC_UNINIT(T, tmp_up, n_in, stack);

                auto zero = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(n_in);

                tmp_lo = primal_residual_in_scaled_lo + alpha_cur * Cdx;
                tmp_up = primal_residual_in_scaled_up + alpha_cur * Cdx;
                Cdx_active =
                  (tmp_lo.array() < 0 || tmp_up.array() > 0).select(Cdx, zero);
                active_part_z = (tmp_lo.array() < 0)
                                  .select(primal_residual_in_scaled_lo, zero) +
                                (tmp_up.array() > 0)
                                  .select(primal_residual_in_scaled_up, zero);
              }

              T a = dx.dot(Hdx) +                         //
                    results.info.rho * dx.squaredNorm() + //
                    results.info.mu_eq_inv * Adx.squaredNorm() +
                    +results.info.mu_in_inv * Cdx_active.squaredNorm() +
                    results.info.nu * results.info.mu_eq *
                      (results.info.mu_eq_inv * Adx - dy).squaredNorm() +
                    results.info.nu * results.info.mu_in *
                      (results.info.mu_in_inv * Cdx_active - dz).squaredNorm();

              T b =
                x_e.dot(Hdx) +                                               //
                (results.info.rho * (x_e - x_prev_e) + g_scaled_e).dot(dx) + //
                Adx.dot(results.info.mu_eq_inv * primal_residual_eq_scaled +
                        y_e) +                                           //
                results.info.mu_in_inv * Cdx_active.dot(active_part_z) + //
                results.info.nu * primal_residual_eq_scaled.dot(
                                    results.info.mu_eq_inv * Adx - dy) + //
                results.info.nu *
                  (active_part_z - results.info.mu_in * z_e)
                    .dot(results.info.mu_in_inv * Cdx_active - dz);

              return {
                a,
                b,
                a * alpha_cur + b,
              };
            };

            LDLT_TEMP_VEC_UNINIT(T, alphas, 2 * n_in, stack);
            isize alphas_count = 0;

            for (isize i = 0; i < n_in; ++i) {
              T alpha_candidates[2] = {
                -primal_residual_in_scaled_lo(i) / (Cdx(i)),
                -primal_residual_in_scaled_up(i) / (Cdx(i)),
              };

              for (auto alpha_candidate : alpha_candidates) {
                if (alpha_candidate > 0) {
                  alphas[alphas_count] = alpha_candidate;
                  ++alphas_count;
                }
              }
            }
            std::sort(alphas.data(), alphas.data() + alphas_count);
            alphas_count =
              std::unique(alphas.data(), alphas.data() + alphas_count) -
              alphas.data();

            if (alphas_count > 0 && alphas[0] <= 1) {
              auto infty = std::numeric_limits<T>::infinity();

              T last_neg_grad = 0;
              T alpha_last_neg = 0;
              T first_pos_grad = 0;
              T alpha_first_pos = infty;

              {
                for (isize i = 0; i < alphas_count; ++i) {
                  T alpha_cur = alphas[i];
                  T gr = primal_dual_gradient_norm(alpha_cur).grad;

                  if (gr < 0) {
                    alpha_last_neg = alpha_cur;
                    last_neg_grad = gr;
                  } else {
                    first_pos_grad = gr;
                    alpha_first_pos = alpha_cur;
                    break;
                  }
                }

                if (alpha_last_neg == 0) {
                  last_neg_grad =
                    primal_dual_gradient_norm(alpha_last_neg).grad;
                }

                if (alpha_first_pos == infty) {
                  auto res = primal_dual_gradient_norm(2 * alpha_last_neg + 1);
                  alpha = -res.b / res.a;
                } else {
                  alpha = alpha_last_neg -
                          last_neg_grad * (alpha_first_pos - alpha_last_neg) /
                            (first_pos_grad - last_neg_grad);
                }
              }
            } else {
              auto res = primal_dual_gradient_norm(T(0));
              alpha = -res.b / res.a;
            }
          }
          if (alpha * infty_norm(dw) < T(1e-11) && iter_inner > 0) {
            results.info.iter += iter_inner + 1;
            return;
          }

          x_e += alpha * dx;
          y_e += alpha * dy;
          z_e += alpha * dz;

          dual_residual_scaled +=
            alpha * (Hdx + ATdy + CTdz + results.info.rho * dx);
          primal_residual_eq_scaled += alpha * (Adx - results.info.mu_eq * dy);
          primal_residual_in_scaled_lo += alpha * Cdx;
          primal_residual_in_scaled_up += alpha * Cdx;

          T err_in = std::max({
            (infty_norm(helpers::negative_part(primal_residual_in_scaled_lo) +
                        helpers::positive_part(primal_residual_in_scaled_up) -
                        results.info.mu_in * z_e)),
            (infty_norm(primal_residual_eq_scaled)),
            (infty_norm(dual_residual_scaled)),
          });
          /* put in debug mode
          if (settings.verbose) {
                  std::cout << "--inner iter " << iter_inner << " iner error "
                                                          << err_in << " alpha "
          << alpha << " infty_norm(dw) "
                                                          << infty_norm(dw) <<
          std::endl;
          }
          */
          if (settings.verbose) {
            std::cout << "\033[1;34m[inner iteration " << iter_inner + 1
                      << "]\033[0m" << std::endl;
            std::cout << std::scientific << std::setw(2) << std::setprecision(2)
                      << "| inner residual=" << err_in << " | alpha=" << alpha
                      << std::endl;
          }
          if (err_in <= bcl_eta_in) {
            results.info.iter += iter_inner + 1;
            return;
          }

          // compute primal and dual infeasibility criteria
          bool is_primal_infeasible = proxsuite::proxqp::sparse::detail::
            global_primal_residual_infeasibility(
              VectorViewMut<T>{ from_eigen, ATdy },
              VectorViewMut<T>{ from_eigen, CTdz },
              VectorViewMut<T>{ from_eigen, dy },
              VectorViewMut<T>{ from_eigen, dz },
              qp_scaled.as_const(),
              settings,
              precond);
          bool is_dual_infeasible = proxsuite::proxqp::sparse::detail::
            global_dual_residual_infeasibility(
              VectorViewMut<T>{ from_eigen, Adx },
              VectorViewMut<T>{ from_eigen, Cdx },
              VectorViewMut<T>{ from_eigen, Hdx },
              VectorViewMut<T>{ from_eigen, dx },
              qp_scaled.as_const(),
              settings,
              data,
              precond);
          if (is_primal_infeasible) {
            results.info.status = QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE;
            dw_prev = dw;
            break;
          } else if (is_dual_infeasible) {
            results.info.status = QPSolverOutput::PROXQP_DUAL_INFEASIBLE;
            dw_prev = dw;
            break;
          }
        }
      };
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

      primal_dual_newton_semi_smooth();
      if (results.info.status == QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE ||
          results.info.status == QPSolverOutput::PROXQP_DUAL_INFEASIBLE) {
        // certificate of infeasibility
        results.x = dw_prev.head(data.dim);
        results.y = dw_prev.segment(data.dim, data.n_eq);
        results.z = dw_prev.tail(data.n_in);
        break;
      }
      // VEG bind : met le résultat tuple de unscaled_primal_dual_residual dans
      // (primal_feasibility_lhs_new, dual_feasibility_lhs_new) en guessant leur
      // type via auto
      VEG_BIND(
        auto,
        (primal_feasibility_lhs_new, dual_feasibility_lhs_new),
        detail::unscaled_primal_dual_residual(work,
                                              results,
                                              primal_residual_eq_scaled,
                                              primal_residual_in_scaled_lo,
                                              primal_residual_in_scaled_up,
                                              dual_residual_scaled,
                                              primal_feasibility_eq_rhs_0,
                                              primal_feasibility_in_rhs_0,
                                              dual_feasibility_rhs_0,
                                              dual_feasibility_rhs_1,
                                              dual_feasibility_rhs_3,
                                              rhs_duality_gap,
                                              precond,
                                              data,
                                              qp_scaled.as_const(),
                                              detail::vec_mut(x_e),
                                              detail::vec_mut(y_e),
                                              detail::vec_mut(z_e),
                                              stack));

      if (is_primal_feasible(primal_feasibility_lhs_new) &&
          is_dual_feasible(dual_feasibility_lhs_new)) {
        if (settings.check_duality_gap) {
          if (std::fabs(results.info.duality_gap) <=
              settings.eps_duality_gap_abs +
                settings.eps_duality_gap_rel * rhs_duality_gap) {
            results.info.pri_res = primal_feasibility_lhs_new;
            results.info.dua_res = dual_feasibility_lhs_new;
            results.info.status = QPSolverOutput::PROXQP_SOLVED;
            break;
          }
        } else {
          results.info.pri_res = primal_feasibility_lhs_new;
          results.info.dua_res = dual_feasibility_lhs_new;
          results.info.status = QPSolverOutput::PROXQP_SOLVED;
          break;
        }
      }

      // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
      auto bcl_update = [&]() -> void {
        if (primal_feasibility_lhs_new <= bcl_eta_ext ||
            iter > settings.safe_guard) {
          bcl_eta_ext *= pow(results.info.mu_in, settings.beta_bcl);
          bcl_eta_in = std::max(bcl_eta_in * results.info.mu_in, eps_in_min);

        } else {
          y_e = y_prev_e;
          z_e = z_prev_e;
          new_bcl_mu_in = std::max(
            results.info.mu_in * settings.mu_update_factor, settings.mu_min_in);
          new_bcl_mu_eq = std::max(
            results.info.mu_eq * settings.mu_update_factor, settings.mu_min_eq);

          new_bcl_mu_in_inv =
            std::min(results.info.mu_in_inv * settings.mu_update_inv_factor,
                     settings.mu_max_in_inv);
          new_bcl_mu_eq_inv =
            std::min(results.info.mu_eq_inv * settings.mu_update_inv_factor,
                     settings.mu_max_eq_inv);
          bcl_eta_ext =
            bcl_eta_ext_init * pow(new_bcl_mu_in, settings.alpha_bcl);
          bcl_eta_in = std::max(new_bcl_mu_in, eps_in_min);
        }
      };
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      bcl_update();

      VEG_BIND(
        auto,
        (_, dual_feasibility_lhs_new_2),
        detail::unscaled_primal_dual_residual(work,
                                              results,
                                              primal_residual_eq_scaled,
                                              primal_residual_in_scaled_lo,
                                              primal_residual_in_scaled_up,
                                              dual_residual_scaled,
                                              primal_feasibility_eq_rhs_0,
                                              primal_feasibility_in_rhs_0,
                                              dual_feasibility_rhs_0,
                                              dual_feasibility_rhs_1,
                                              dual_feasibility_rhs_3,
                                              rhs_duality_gap,
                                              precond,
                                              data,
                                              qp_scaled.as_const(),
                                              detail::vec_mut(x_e),
                                              detail::vec_mut(y_e),
                                              detail::vec_mut(z_e),
                                              stack));
      proxsuite::linalg::veg::unused(_);

      if (primal_feasibility_lhs_new >= primal_feasibility_lhs && //
          dual_feasibility_lhs_new_2 >= primal_feasibility_lhs && //
          results.info.mu_in <= T(1.E-5)) {
        new_bcl_mu_in = settings.cold_reset_mu_in;
        new_bcl_mu_eq = settings.cold_reset_mu_eq;
        new_bcl_mu_in_inv = settings.cold_reset_mu_in_inv;
        new_bcl_mu_eq_inv = settings.cold_reset_mu_eq_inv;
      }
    }
    if (results.info.mu_in != new_bcl_mu_in ||
        results.info.mu_eq != new_bcl_mu_eq) {
      {
        ++results.info.mu_updates;
      }
      /*
      refactorize(
                      work,
                      results,
                      kkt_active,
                      active_constraints,
                      data,
                      stack,
                      xtag);
      */
      if (work.internal.do_ldlt) {
        isize w_values = 1; // un seul elt non nul
        T alpha = 0;
        for (isize j = 0; j < n_eq + n_in; ++j) {
          I row_index = I(j + n);
          if (j < n_eq) {
            alpha = results.info.mu_eq - new_bcl_mu_eq;

          } else {
            if (!work.active_inequalities[j - n_eq]) {
              continue;
            }
            alpha = results.info.mu_in - new_bcl_mu_in;
          }
          T value = 1;
          proxsuite::linalg::sparse::VecRef<T, I> w{
            proxsuite::linalg::veg::from_raw_parts,
            n + n_eq + n_in,
            w_values,
            &row_index, // &: adresse de row index
            &value,
          };
          ldl = rank1_update(ldl, etree, perm_inv, w, alpha, stack);
        }
      } else {
        refactorize(
          work, results, kkt_active, active_constraints, data, stack, xtag);
      }
    }

    results.info.mu_eq = new_bcl_mu_eq;
    results.info.mu_in = new_bcl_mu_in;
    results.info.mu_eq_inv = new_bcl_mu_eq_inv;
    results.info.mu_in_inv = new_bcl_mu_in_inv;
  }
  LDLT_TEMP_VEC_UNINIT(T, tmp, n, stack);
  tmp.setZero();
  detail::noalias_symhiv_add(tmp, qp_scaled.H.to_eigen(), x_e);
  precond.unscale_dual_residual_in_place({ proxqp::from_eigen, tmp });

  precond.unscale_primal_in_place({ proxqp::from_eigen, x_e });
  precond.unscale_dual_in_place_eq({ proxqp::from_eigen, y_e });
  precond.unscale_dual_in_place_in({ proxqp::from_eigen, z_e });
  tmp *= 0.5;
  tmp += data.g;
  results.info.objValue = (tmp).dot(x_e);

  if (settings.compute_timings) {
    results.info.solve_time = work.timer.elapsed().user; // in nanoseconds
    results.info.run_time = results.info.solve_time + results.info.setup_time;
  }
  if (settings.verbose) {
    std::cout << "-------------------SOLVER STATISTICS-------------------"
              << std::endl;
    std::cout << "outer iter:   " << results.info.iter_ext << std::endl;
    std::cout << "total iter:   " << results.info.iter << std::endl;
    std::cout << "mu updates:   " << results.info.mu_updates << std::endl;
    std::cout << "rho updates:  " << results.info.rho_updates << std::endl;
    std::cout << "objective:    " << results.info.objValue << std::endl;
    switch (results.info.status) {
      case QPSolverOutput::PROXQP_SOLVED: {
        std::cout << "status:       "
                  << "Solved" << std::endl;
        break;
      }
      case QPSolverOutput::PROXQP_MAX_ITER_REACHED: {
        std::cout << "status:       "
                  << "Maximum number of iterations reached" << std::endl;
        break;
      }
      case QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE: {
        std::cout << "status:       "
                  << "Primal infeasible" << std::endl;
        break;
      }
      case QPSolverOutput::PROXQP_DUAL_INFEASIBLE: {
        std::cout << "status:       "
                  << "Dual infeasible" << std::endl;
        break;
      }
      default: {
        assert(false && "Should never happened");
        break;
      }
    }
    if (settings.compute_timings)
      std::cout << "run time:     " << results.info.solve_time << std::endl;
    std::cout << "--------------------------------------------------------"
              << std::endl;
  }

  assert(!std::isnan(results.info.pri_res));
  assert(!std::isnan(results.info.dua_res));
  assert(!std::isnan(results.info.duality_gap));

  work.set_dirty();
}
} // namespace sparse
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_SPARSE_SOLVER_HPP */
