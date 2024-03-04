//
// Copyright (c) 2022 INRIA
//
/**
 * @file utils.hpp
 */
#ifndef PROXSUITE_PROXQP_DENSE_UTILS_HPP
#define PROXSUITE_PROXQP_DENSE_UTILS_HPP

#include <iostream>
#include <fstream>
#include <cmath>
#include <type_traits>

#include "proxsuite/helpers/common.hpp"
#include "proxsuite/proxqp/dense/views.hpp"
#include "proxsuite/proxqp/dense/workspace.hpp"
#include <proxsuite/proxqp/dense/model.hpp>
#include <proxsuite/proxqp/results.hpp>
#include <proxsuite/proxqp/utils/prints.hpp>
#include <proxsuite/proxqp/settings.hpp>
#include <proxsuite/proxqp/dense/preconditioner/ruiz.hpp>

// #include <fmt/format.h>
// #include <fmt/ostream.h>

namespace proxsuite {
namespace proxqp {
namespace dense {

template<typename T>
void
print_setup_header(const Settings<T>& settings,
                   const Results<T>& results,
                   const Model<T>& model)
{

  proxsuite::proxqp::print_preambule();

  // Print variables and constraints
  std::cout << "problem:  " << std::noshowpos << std::endl;
  std::cout << "          variables n = " << model.dim
            << ", equality constraints n_eq = " << model.n_eq << ",\n"
            << "          inequality constraints n_in = " << model.n_in
            << std::endl;

  // Print Settings
  std::cout << "settings: " << std::endl;
  std::cout << "          backend = dense," << std::endl;
  std::cout << "          eps_abs = " << settings.eps_abs
            << " eps_rel = " << settings.eps_rel << std::endl;
  std::cout << "          eps_prim_inf = " << settings.eps_primal_inf
            << ", eps_dual_inf = " << settings.eps_dual_inf << "," << std::endl;

  std::cout << "          rho = " << results.info.rho
            << ", mu_eq = " << results.info.mu_eq
            << ", mu_in = " << results.info.mu_in << "," << std::endl;
  std::cout << "          max_iter = " << settings.max_iter
            << ", max_iter_in = " << settings.max_iter_in << "," << std::endl;

  if (settings.compute_preconditioner) {
    std::cout << "          scaling: on, " << std::endl;
  } else {
    std::cout << "          scaling: off, " << std::endl;
  }
  if (settings.compute_timings) {
    std::cout << "          timings: on, " << std::endl;
  } else {
    std::cout << "          timings: off, " << std::endl;
  }
  switch (settings.initial_guess) {
    case InitialGuessStatus::WARM_START:
      std::cout << "          initial guess: warm start. \n" << std::endl;
      break;
    case InitialGuessStatus::NO_INITIAL_GUESS:
      std::cout << "          initial guess: initial guess. \n" << std::endl;
      break;
    case InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT:
      std::cout
        << "          initial guess: warm start with previous result. \n"
        << std::endl;
      break;
    case InitialGuessStatus::COLD_START_WITH_PREVIOUS_RESULT:
      std::cout
        << "          initial guess: cold start with previous result. \n"
        << std::endl;
      break;
    case InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS:
      std::cout
        << "          initial guess: equality constrained initial guess. \n"
        << std::endl;
  }
}

/*!
 * Save a matrix into a CSV format. Used for debug purposes.
 *
 * @param filename filename name for the CSV.
 * @param mat matrix to save into CSV format.
 */
template<typename Derived>
void
save_data(const std::string& filename, const ::Eigen::MatrixBase<Derived>& mat)
{
  // https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
  const static Eigen::IOFormat CSVFormat(
    Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

  std::ofstream file(filename);
  if (file.is_open()) {
    file << mat.format(CSVFormat);
    file.close();
  }
}

/*!
 * Derives the global primal residual of the QP problem.
 *
 * @param qpwork solver workspace.
 * @param qpmodel QP problem model as defined by the user (without any scaling
 * performed).
 * @param qpresults solver results.
 * @param ruiz ruiz preconditioner.
 * @param primal_feasibility_lhs primal infeasibility.
 * @param primal_feasibility_eq_rhs_0 scalar variable used when using a relative
 * stopping criterion.
 * @param primal_feasibility_in_rhs_0 scalar variable used when using a relative
 * stopping criterion.
 * @param primal_feasibility_eq_lhs scalar variable used when using a relative
 * stopping criterion.
 * @param primal_feasibility_in_lhs scalar variable used when using a relative
 * stopping criterion.
 */
template<typename T>
void
global_primal_residual(const Model<T>& qpmodel,
                       const Results<T>& qpresults,
                       Workspace<T>& qpwork,
                       const preconditioner::RuizEquilibration<T>& ruiz,
                       T& primal_feasibility_lhs,
                       T& primal_feasibility_eq_rhs_0,
                       T& primal_feasibility_in_rhs_0,
                       T& primal_feasibility_eq_lhs,
                       T& primal_feasibility_in_lhs)
{
  // COMPUTES:
  // primal_residual_eq_scaled = scaled(Ax - b)
  //
  // primal_feasibility_lhs = max(norm(unscaled(Ax - b)),
  //                              norm(unscaled([Cx - u]+ + [Cx - l]-)))
  // primal_feasibility_eq_rhs_0 = norm(unscaled(Ax))
  // primal_feasibility_in_rhs_0 = norm(unscaled(Cx))
  //
  // MAY_ALIAS[primal_residual_in_scaled_u, primal_residual_in_scaled_l]
  //
  // INDETERMINATE:
  // primal_residual_in_scaled_u = unscaled(Cx)
  // primal_residual_in_scaled_l = unscaled([Cx - u]+ + [Cx - l]-)
  qpwork.primal_residual_eq_scaled.noalias() = qpwork.A_scaled * qpresults.x;
  qpwork.primal_residual_in_scaled_up.noalias() = qpwork.C_scaled * qpresults.x;

  ruiz.unscale_primal_residual_in_place_eq(
    VectorViewMut<T>{ from_eigen, qpwork.primal_residual_eq_scaled });
  primal_feasibility_eq_rhs_0 = infty_norm(qpwork.primal_residual_eq_scaled);
  ruiz.unscale_primal_residual_in_place_in(
    VectorViewMut<T>{ from_eigen, qpwork.primal_residual_in_scaled_up });
  primal_feasibility_in_rhs_0 = infty_norm(qpwork.primal_residual_in_scaled_up);

  qpwork.primal_residual_in_scaled_low =
    helpers::positive_part(qpwork.primal_residual_in_scaled_up - qpmodel.u) +
    helpers::negative_part(qpwork.primal_residual_in_scaled_up - qpmodel.l);
  qpwork.primal_residual_eq_scaled -= qpmodel.b;

  primal_feasibility_in_lhs = infty_norm(qpwork.primal_residual_in_scaled_low);
  primal_feasibility_eq_lhs = infty_norm(qpwork.primal_residual_eq_scaled);
  primal_feasibility_lhs =
    std::max(primal_feasibility_eq_lhs, primal_feasibility_in_lhs);

  ruiz.scale_primal_residual_in_place_eq(
    VectorViewMut<T>{ from_eigen, qpwork.primal_residual_eq_scaled });
}

/*!
 * Check whether the global primal infeasibility criterion is satisfied.
 *
 * @param qpwork solver workspace.
 * @param qpsettings solver settings.
 * @param ruiz ruiz preconditioner.
 * @param ATdy variable used for testing global primal infeasibility criterion
 * is satisfied.
 * @param CTdz variable used for testing global primal infeasibility criterion
 * is satisfied.
 * @param dy variable used for testing global primal infeasibility criterion is
 * satisfied.
 * @param dz variable used for testing global primal infeasibility criterion is
 * satisfied.
 */
template<typename T>
bool
global_primal_residual_infeasibility(
  VectorViewMut<T> ATdy,
  VectorViewMut<T> CTdz,
  VectorViewMut<T> dy,
  VectorViewMut<T> dz,
  Workspace<T>& qpwork,
  const Settings<T>& qpsettings,
  const preconditioner::RuizEquilibration<T>& ruiz)
{

  // The problem is primal infeasible if the following four conditions hold:
  //
  // ||unscaled(A^Tdy)|| <= eps_p_inf ||unscaled(dy)||
  // b^T dy <= -eps_p_inf ||unscaled(dy)||
  // ||unscaled(C^Tdz)|| <= eps_p_inf ||unscaled(dz)||
  // u^T [dz]_+ - l^T[-dz]_+ <= -eps_p_inf ||unscaled(dz)||
  //
  // the variables in entry are changed in place

  bool res = infty_norm(dy.to_eigen()) != 0 && infty_norm(dz.to_eigen()) != 0;
  if (!res) {
    return res;
  }
  ruiz.unscale_dual_residual_in_place(ATdy);
  ruiz.unscale_dual_residual_in_place(CTdz);
  T eq_inf = dy.to_eigen().dot(qpwork.b_scaled);
  T in_inf = helpers::positive_part(dz.to_eigen()).dot(qpwork.u_scaled) -
             helpers::negative_part(dz.to_eigen()).dot(qpwork.l_scaled);
  ruiz.unscale_dual_in_place_eq(dy);
  ruiz.unscale_dual_in_place_in(dz);

  T bound_y = qpsettings.eps_primal_inf * infty_norm(dy.to_eigen());
  T bound_z = qpsettings.eps_primal_inf * infty_norm(dz.to_eigen());

  res = infty_norm(ATdy.to_eigen()) <= bound_y && eq_inf <= -bound_y &&
        infty_norm(CTdz.to_eigen()) <= bound_z && in_inf <= -bound_z;
  return res;
}

/*!
 * Check whether the global dual infeasibility criterion is satisfied.
 *
 * @param qpwork solver workspace.
 * @param qpsettings solver settings.
 * @param qpmodel QP problem model as defined by the user (without any scaling
 * performed).
 * @param ruiz ruiz preconditioner.
 * @param Adx variable used for testing global dual infeasibility criterion is
 * satisfied.
 * @param Cdx variable used for testing global dual infeasibility criterion is
 * satisfied.
 * @param Hdx variable used for testing global dual infeasibility criterion is
 * satisfied.
 * @param dx variable used for testing global dual infeasibility criterion is
 * satisfied.
 */
template<typename T>
bool
global_dual_residual_infeasibility(
  VectorViewMut<T> Adx,
  VectorViewMut<T> Cdx,
  VectorViewMut<T> Hdx,
  VectorViewMut<T> dx,
  Workspace<T>& qpwork,
  const Settings<T>& qpsettings,
  const Model<T>& qpmodel,
  const preconditioner::RuizEquilibration<T>& ruiz)
{

  // The problem is dual infeasible the two following conditions hold:
  //
  // FIRST
  // ||unscaled(Adx)|| <= eps_d_inf ||unscaled(dx)||
  // unscaled(Cdx)_i \in [-eps_d_inf,eps_d_inf] ||unscaled(dx)|| if u_i and l_i
  // are finite 					or >=
  // -eps_d_inf||unscaled(dx)|| if u_i = +inf or <= eps_d_inf||unscaled(dx)|| if
  // l_i = -inf
  //
  // SECOND
  //
  // ||unscaled(Hdx)|| <= c eps_d_inf * ||unscaled(dx)||  and  q^Tdx <= -c
  // eps_d_inf  ||unscaled(dx)|| the variables in
  // entry are changed in place
  ruiz.unscale_dual_residual_in_place(Hdx);
  ruiz.unscale_primal_residual_in_place_eq(Adx);
  ruiz.unscale_primal_residual_in_place_in(Cdx);
  T gdx = (dx.to_eigen()).dot(qpwork.g_scaled);
  ruiz.unscale_primal_in_place(dx);

  T bound = infty_norm(dx.to_eigen()) * qpsettings.eps_dual_inf;
  T bound_neg = -bound;

  bool first_cond = infty_norm(Adx.to_eigen()) <= bound;

  for (i64 iter = 0; iter < qpmodel.n_in; ++iter) {
    T Cdx_i = Cdx.to_eigen()[iter];
    if (qpwork.u_scaled[iter] <= 1.E20 && qpwork.l_scaled[iter] >= -1.E20) {
      first_cond = first_cond && Cdx_i <= bound && Cdx_i >= bound_neg;
    } else if (qpwork.u_scaled[iter] > 1.E20) {
      first_cond = first_cond && Cdx_i >= bound_neg;
    } else if (qpwork.l_scaled[iter] < -1.E20) {
      first_cond = first_cond && Cdx_i <= bound;
    }
  }

  bound *= ruiz.c;
  bound_neg *= ruiz.c;
  bool second_cond_alt1 =
    infty_norm(Hdx.to_eigen()) <= bound && gdx <= bound_neg;
  bound_neg *= qpsettings.eps_dual_inf;

  bool res = first_cond && second_cond_alt1 && infty_norm(dx.to_eigen()) != 0;
  return res;
}

/*!
 * Derives the global dual residual of the QP problem.
 *
 * @param qpwork solver workspace.
 * @param qpresults solver results.
 * @param ruiz ruiz preconditioner.
 * @param dual_feasibility_lhs primal infeasibility.
 * @param primal_feasibility_eq_rhs_0 scalar variable used when using a relative
 * stopping criterion.
 * @param dual_feasibility_rhs_0 scalar variable used when using a relative
 * stopping criterion.
 * @param dual_feasibility_rhs_1 scalar variable used when using a relative
 * stopping criterion.
 * @param dual_feasibility_rhs_3 scalar variable used when using a relative
 * stopping criterion.
 */
template<typename T>
void
global_dual_residual(Results<T>& qpresults,
                     Workspace<T>& qpwork,
                     const Model<T>& qpmodel,
                     const preconditioner::RuizEquilibration<T>& ruiz,
                     T& dual_feasibility_lhs,
                     T& dual_feasibility_rhs_0,
                     T& dual_feasibility_rhs_1,
                     T& dual_feasibility_rhs_3,
                     T& rhs_duality_gap,
                     T& duality_gap)
{
  // dual_feasibility_lhs = norm(dual_residual_scaled)
  // dual_feasibility_rhs_0 = norm(unscaled(Hx))
  // dual_feasibility_rhs_1 = norm(unscaled(ATy))
  // dual_feasibility_rhs_3 = norm(unscaled(CTz))
  //
  // dual_residual_scaled = scaled(Hx + g + ATy + CTz)

  qpwork.dual_residual_scaled = qpwork.g_scaled;
  qpwork.CTz.noalias() =
    qpwork.H_scaled.template selfadjointView<Eigen::Lower>() * qpresults.x;
  qpwork.dual_residual_scaled += qpwork.CTz;
  ruiz.unscale_dual_residual_in_place(
    VectorViewMut<T>{ from_eigen, qpwork.CTz }); // contains unscaled Hx
  dual_feasibility_rhs_0 = infty_norm(qpwork.CTz);

  ruiz.unscale_primal_in_place(VectorViewMut<T>{ from_eigen, qpresults.x });
  duality_gap = (qpmodel.g).dot(qpresults.x);
  rhs_duality_gap = std::fabs(duality_gap);
  const T xHx = (qpwork.CTz).dot(qpresults.x);
  duality_gap += xHx; // contains now xHx+g.Tx
  rhs_duality_gap = std::max(rhs_duality_gap, std::abs(xHx));

  ruiz.scale_primal_in_place(VectorViewMut<T>{ from_eigen, qpresults.x });

  qpwork.CTz.noalias() = qpwork.A_scaled.transpose() * qpresults.y;
  qpwork.dual_residual_scaled += qpwork.CTz;
  ruiz.unscale_dual_residual_in_place(
    VectorViewMut<T>{ from_eigen, qpwork.CTz });
  dual_feasibility_rhs_1 = infty_norm(qpwork.CTz);

  qpwork.CTz.noalias() = qpwork.C_scaled.transpose() * qpresults.z;
  qpwork.dual_residual_scaled += qpwork.CTz;
  ruiz.unscale_dual_residual_in_place(
    VectorViewMut<T>{ from_eigen, qpwork.CTz });
  dual_feasibility_rhs_3 = infty_norm(qpwork.CTz);

  ruiz.unscale_dual_residual_in_place(
    VectorViewMut<T>{ from_eigen, qpwork.dual_residual_scaled });

  dual_feasibility_lhs = infty_norm(qpwork.dual_residual_scaled);

  ruiz.scale_dual_residual_in_place(
    VectorViewMut<T>{ from_eigen, qpwork.dual_residual_scaled });

  ruiz.unscale_dual_in_place_eq(VectorViewMut<T>{ from_eigen, qpresults.y });
  const T by = (qpmodel.b).dot(qpresults.y);
  rhs_duality_gap = std::max(rhs_duality_gap, std::abs(by));
  duality_gap += by;
  ruiz.scale_dual_in_place_eq(VectorViewMut<T>{ from_eigen, qpresults.y });

  ruiz.unscale_dual_in_place_in(VectorViewMut<T>{ from_eigen, qpresults.z });

  const T zu =
    helpers::select(qpwork.active_set_up, qpresults.z, 0)
      .dot(helpers::at_most(qpmodel.u, helpers::infinite_bound<T>::value()));
  rhs_duality_gap = std::max(rhs_duality_gap, std::abs(zu));
  duality_gap += zu;

  const T zl =
    helpers::select(qpwork.active_set_low, qpresults.z, 0)
      .dot(helpers::at_least(qpmodel.l, -helpers::infinite_bound<T>::value()));
  rhs_duality_gap = std::max(rhs_duality_gap, std::abs(zl));
  duality_gap += zl;

  ruiz.scale_dual_in_place_in(VectorViewMut<T>{ from_eigen, qpresults.z });
}

} // namespace dense
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_DENSE_UTILS_HPP */
