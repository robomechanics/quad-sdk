//
// Copyright (c) 2022-2023 INRIA
//
/** \file */

#ifndef PROXSUITE_PROXQP_SPARSE_UTILS_HPP
#define PROXSUITE_PROXQP_SPARSE_UTILS_HPP

#include <iostream>
#include <Eigen/IterativeLinearSolvers>
#include <unsupported/Eigen/IterativeSolvers>

#include "proxsuite/helpers/common.hpp"
#include <proxsuite/linalg/dense/core.hpp>
#include <proxsuite/linalg/sparse/core.hpp>
#include "proxsuite/proxqp/sparse/workspace.hpp"
#include <proxsuite/linalg/sparse/factorize.hpp>
#include <proxsuite/linalg/sparse/update.hpp>
#include <proxsuite/linalg/sparse/rowmod.hpp>
#include <proxsuite/proxqp/dense/views.hpp>
#include <proxsuite/proxqp/settings.hpp>
#include <proxsuite/linalg/veg/vec.hpp>
#include "proxsuite/proxqp/results.hpp"
#include "proxsuite/proxqp/utils/prints.hpp"
#include "proxsuite/proxqp/sparse/views.hpp"
#include "proxsuite/proxqp/sparse/model.hpp"
#include "proxsuite/proxqp/sparse/preconditioner/ruiz.hpp"
#include "proxsuite/proxqp/sparse/preconditioner/identity.hpp"

namespace proxsuite {
namespace proxqp {
namespace sparse {

template<typename T, typename I>
void
print_setup_header(const Settings<T>& settings,
                   Results<T>& results,
                   const Model<T, I>& model)
{

  proxsuite::proxqp::print_preambule();

  // Print variables and constraints
  std::cout << "problem:  " << std::noshowpos << std::endl;
  std::cout << "          variables n = " << model.dim
            << ", equality constraints n_eq = " << model.n_eq << ",\n"
            << "          inequality constraints n_in = " << model.n_in
            << ", nnz = " << model.H_nnz + model.A_nnz + model.C_nnz << ",\n"
            << std::endl;

  // Print Settings
  std::cout << "settings: " << std::endl;
  std::cout << "          backend = sparse," << std::endl;
  std::cout << "          sparse_backend = " << settings.sparse_backend;
  if (settings.sparse_backend == SparseBackend::Automatic) {
    std::cout << " -> " << results.info.sparse_backend;
  }
  std::cout << "," << std::endl;
  std::cout << "          eps_abs = " << settings.eps_abs
            << ", eps_rel = " << settings.eps_rel << std::endl;
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

namespace detail {

template<typename T, typename I>
VEG_NO_INLINE void
noalias_gevmmv_add_impl( //
  VectorViewMut<T> out_l,
  VectorViewMut<T> out_r,
  proxsuite::linalg::sparse::MatRef<T, I> a,
  VectorView<T> in_l,
  VectorView<T> in_r)
{
  VEG_ASSERT_ALL_OF /* NOLINT */ (a.nrows() == out_r.dim,
                                  a.ncols() == in_r.dim,
                                  a.ncols() == out_l.dim,
                                  a.nrows() == in_l.dim);
  // equivalent to
  // out_r.to_eigen().noalias() += a.to_eigen() * in_r.to_eigen();
  // out_l.to_eigen().noalias() += a.to_eigen().transpose() * in_l.to_eigen();

  auto* ai = a.row_indices();
  auto* ax = a.values();
  auto n = a.ncols();

  for (usize j = 0; j < usize(n); ++j) {
    usize col_start = a.col_start(j);
    usize col_end = a.col_end(j);

    T acc0 = 0;
    T acc1 = 0;
    T acc2 = 0;
    T acc3 = 0;

    T in_rj = in_r(isize(j));

    usize pcount = col_end - col_start;

    usize p = col_start;

    auto zx = proxsuite::linalg::sparse::util::zero_extend;

    for (; p < col_start + pcount / 4 * 4; p += 4) {
      auto i0 = isize(zx(ai[p + 0]));
      auto i1 = isize(zx(ai[p + 1]));
      auto i2 = isize(zx(ai[p + 2]));
      auto i3 = isize(zx(ai[p + 3]));

      T ai0j = ax[p + 0];
      T ai1j = ax[p + 1];
      T ai2j = ax[p + 2];
      T ai3j = ax[p + 3];

      out_r(i0) += ai0j * in_rj;
      out_r(i1) += ai1j * in_rj;
      out_r(i2) += ai2j * in_rj;
      out_r(i3) += ai3j * in_rj;

      acc0 += ai0j * in_l(i0);
      acc1 += ai1j * in_l(i1);
      acc2 += ai2j * in_l(i2);
      acc3 += ai3j * in_l(i3);
    }

    for (; p < col_end; ++p) {
      auto i = isize(zx(ai[p]));

      T aij = ax[p];
      out_r(i) += aij * in_rj;
      acc0 += aij * in_l(i);
    }

    acc0 = ((acc0 + acc1) + (acc2 + acc3));
    out_l(isize(j)) += acc0;
  }
}

template<typename T, typename I>
VEG_NO_INLINE void
noalias_symhiv_add_impl( //
  VectorViewMut<T> out,
  proxsuite::linalg::sparse::MatRef<T, I> a,
  VectorView<T> in)
{
  VEG_ASSERT_ALL_OF /* NOLINT */ ( //
    a.nrows() == a.ncols(),
    a.nrows() == out.dim,
    a.ncols() == in.dim);
  // equivalent to
  // out.to_eigen().noalias() +=
  // 		a.to_eigen().template selfadjointView<Eigen::Upper>() *
  // in.to_eigen();

  auto* ai = a.row_indices();
  auto* ax = a.values();
  auto n = a.ncols();

  for (usize j = 0; j < usize(n); ++j) {
    usize col_start = a.col_start(j);
    usize col_end = a.col_end(j);

    if (col_start == col_end) {
      continue;
    }

    T acc0 = 0;
    T acc1 = 0;
    T acc2 = 0;
    T acc3 = 0;

    T in_j = in(isize(j));

    usize pcount = col_end - col_start;

    auto zx = proxsuite::linalg::sparse::util::zero_extend;

    if (zx(ai[col_end - 1]) == j) {
      T ajj = ax[col_end - 1];
      out(isize(j)) += ajj * in_j;
      pcount -= 1;
    }

    usize p = col_start;

    for (; p < col_start + pcount / 4 * 4; p += 4) {
      auto i0 = isize(zx(ai[p + 0]));
      auto i1 = isize(zx(ai[p + 1]));
      auto i2 = isize(zx(ai[p + 2]));
      auto i3 = isize(zx(ai[p + 3]));

      T ai0j = ax[p + 0];
      T ai1j = ax[p + 1];
      T ai2j = ax[p + 2];
      T ai3j = ax[p + 3];

      out(i0) += ai0j * in_j;
      out(i1) += ai1j * in_j;
      out(i2) += ai2j * in_j;
      out(i3) += ai3j * in_j;

      acc0 += ai0j * in(i0);
      acc1 += ai1j * in(i1);
      acc2 += ai2j * in(i2);
      acc3 += ai3j * in(i3);
    }
    for (; p < col_start + pcount; ++p) {
      auto i = isize(zx(ai[p]));

      T aij = ax[p];
      out(i) += aij * in_j;
      acc0 += aij * in(i);
    }
    acc0 = ((acc0 + acc1) + (acc2 + acc3));
    out(isize(j)) += acc0;
  }
}

template<typename OutL, typename OutR, typename A, typename InL, typename InR>
void
noalias_gevmmv_add(OutL&& out_l,
                   OutR&& out_r,
                   A const& a,
                   InL const& in_l,
                   InR const& in_r)
{
  // noalias general vector matrix matrix vector add
  noalias_gevmmv_add_impl<typename A::Scalar, typename A::StorageIndex>(
    { proxqp::from_eigen, out_l },
    { proxqp::from_eigen, out_r },
    { proxsuite::linalg::sparse::from_eigen, a },
    { proxqp::from_eigen, in_l },
    { proxqp::from_eigen, in_r });
}

template<typename Out, typename A, typename In>
void
noalias_symhiv_add(Out&& out, A const& a, In const& in)
{
  // noalias symmetric (hi) matrix vector add
  noalias_symhiv_add_impl<typename A::Scalar, typename A::StorageIndex>(
    { proxqp::from_eigen, out },
    { proxsuite::linalg::sparse::from_eigen, a },
    { proxqp::from_eigen, in });
}

template<typename T, typename I>
struct AugmentedKkt : Eigen::EigenBase<AugmentedKkt<T, I>>
{
  struct Raw /* NOLINT */
  {
    proxsuite::linalg::sparse::MatRef<T, I> kkt_active;
    proxsuite::linalg::veg::Slice<bool> active_constraints;
    isize n;
    isize n_eq;
    isize n_in;
    T rho;
    T mu_eq;
    T mu_in;
  } _;

  AugmentedKkt /* NOLINT */ (Raw raw) noexcept
    : _{ raw }
  {
  }

  using Scalar = T;
  using RealScalar = T;
  using StorageIndex = I;
  enum
  {
    ColsAtCompileTime = Eigen::Dynamic,
    MaxColsAtCompileTime = Eigen::Dynamic,
    IsRowMajor = false,
  };

  auto rows() const noexcept -> isize { return _.n + _.n_eq + _.n_in; }
  auto cols() const noexcept -> isize { return rows(); }
  template<typename Rhs>
  auto operator*(Eigen::MatrixBase<Rhs> const& x) const
    -> Eigen::Product<AugmentedKkt, Rhs, Eigen::AliasFreeProduct>
  {
    return Eigen::Product< //
      AugmentedKkt,
      Rhs,
      Eigen::AliasFreeProduct>{
      *this,
      x.derived(),
    };
  }
};

template<typename T>
using VecMapMut = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>,
                             Eigen::Unaligned,
                             Eigen::Stride<Eigen::Dynamic, 1>>;
template<typename T>
using VecMap = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1> const,
                          Eigen::Unaligned,
                          Eigen::Stride<Eigen::Dynamic, 1>>;

template<typename V>
auto
vec(V const& v) -> VecMap<typename V::Scalar>
{
  static_assert(V::InnerStrideAtCompileTime == 1, ".");
  return {
    v.data(),
    v.rows(),
    v.cols(),
    Eigen::Stride<Eigen::Dynamic, 1>{
      v.outerStride(),
      v.innerStride(),
    },
  };
}

template<typename V>
auto
vec_mut(V&& v)
  -> VecMapMut<typename proxsuite::linalg::veg::uncvref_t<V>::Scalar>
{
  static_assert(
    proxsuite::linalg::veg::uncvref_t<V>::InnerStrideAtCompileTime == 1, ".");
  return {
    v.data(),
    v.rows(),
    v.cols(),
    Eigen::Stride<Eigen::Dynamic, 1>{
      v.outerStride(),
      v.innerStride(),
    },
  };
}

template<typename T, typename I>
auto
middle_cols(proxsuite::linalg::sparse::MatRef<T, I> mat,
            isize start,
            isize ncols,
            isize nnz) -> proxsuite::linalg::sparse::MatRef<T, I>
{
  VEG_ASSERT(start <= mat.ncols());
  VEG_ASSERT(ncols <= mat.ncols() - start);

  return {
    proxsuite::linalg::sparse::from_raw_parts,
    mat.nrows(),
    ncols,
    nnz,
    mat.col_ptrs() + start,
    mat.is_compressed() ? nullptr : (mat.nnz_per_col() + start),
    mat.row_indices(),
    mat.values(),
  };
}

template<typename T, typename I>
auto
middle_cols_mut(proxsuite::linalg::sparse::MatMut<T, I> mat,
                isize start,
                isize ncols,
                isize nnz) -> proxsuite::linalg::sparse::MatMut<T, I>
{
  VEG_ASSERT(start <= mat.ncols());
  VEG_ASSERT(ncols <= mat.ncols() - start);
  return {
    proxsuite::linalg::sparse::from_raw_parts,
    mat.nrows(),
    ncols,
    nnz,
    mat.col_ptrs_mut() + start,
    mat.is_compressed() ? nullptr : (mat.nnz_per_col_mut() + start),
    mat.row_indices_mut(),
    mat.values_mut(),
  };
}

template<typename T, typename I>
auto
top_rows_unchecked(proxsuite::linalg::veg::Unsafe /*unsafe*/,
                   proxsuite::linalg::sparse::MatRef<T, I> mat,
                   isize nrows) -> proxsuite::linalg::sparse::MatRef<T, I>
{
  VEG_ASSERT(nrows <= mat.nrows());
  return {
    proxsuite::linalg::sparse::from_raw_parts,
    nrows,
    mat.ncols(),
    mat.nnz(),
    mat.col_ptrs(),
    mat.nnz_per_col(),
    mat.row_indices(),
    mat.values(),
  };
}

template<typename T, typename I>
auto
top_rows_mut_unchecked(proxsuite::linalg::veg::Unsafe /*unsafe*/,
                       proxsuite::linalg::sparse::MatMut<T, I> mat,
                       isize nrows) -> proxsuite::linalg::sparse::MatMut<T, I>
{
  VEG_ASSERT(nrows <= mat.nrows());
  return {
    proxsuite::linalg::sparse::from_raw_parts,
    nrows,
    mat.ncols(),
    mat.nnz(),
    mat.col_ptrs_mut(),
    mat.nnz_per_col_mut(),
    mat.row_indices_mut(),
    mat.values_mut(),
  };
}
/*!
 * Check whether the global primal infeasibility criterion is satisfied.
 *
 * @param qp_scaled view on the scaled version of the qp problem.
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
template<typename T, typename I, typename P>
bool
global_primal_residual_infeasibility(VectorViewMut<T> ATdy,
                                     VectorViewMut<T> CTdz,
                                     VectorViewMut<T> dy,
                                     VectorViewMut<T> dz,
                                     const QpView<T, I> qp_scaled,
                                     const Settings<T>& qpsettings,
                                     const P& ruiz)
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
  T eq_inf = dy.to_eigen().dot(qp_scaled.b.to_eigen());
  T in_inf = helpers::positive_part(dz.to_eigen()).dot(qp_scaled.u.to_eigen()) -
             helpers::negative_part(dz.to_eigen()).dot(qp_scaled.l.to_eigen());
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

* @param qp_scaled view on the scaled version of the qp problem.
* @param qpsettings solver settings.
* @param qpmodel QP problem model as defined by the user (without any scaling
performed).
* @param ruiz ruiz preconditioner.
* @param Adx variable used for testing global dual infeasibility criterion is
satisfied.
* @param Cdx variable used for testing global dual infeasibility criterion is
satisfied.
* @param Hdx variable used for testing global dual infeasibility criterion is
satisfied.
* @param dx variable used for testing global dual infeasibility criterion is
satisfied.
*/
template<typename T, typename I, typename P>
bool
global_dual_residual_infeasibility(VectorViewMut<T> Adx,
                                   VectorViewMut<T> Cdx,
                                   VectorViewMut<T> Hdx,
                                   VectorViewMut<T> dx,
                                   const QpView<T, I> qp_scaled,
                                   const Settings<T>& qpsettings,
                                   const Model<T, I>& qpmodel,
                                   const P& ruiz)
{

  // The problem is dual infeasible if one of the conditions hold:
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
  // eps_d_inf  ||unscaled(dx)||
  // the variables in entry are changed in place
  ruiz.unscale_dual_residual_in_place(Hdx);
  ruiz.unscale_primal_residual_in_place_eq(Adx);
  ruiz.unscale_primal_residual_in_place_in(Cdx);
  T gdx = (dx.to_eigen()).dot(qp_scaled.g.to_eigen());
  ruiz.unscale_primal_in_place(dx);

  T bound = infty_norm(dx.to_eigen()) * qpsettings.eps_dual_inf;
  T bound_neg = -bound;

  bool first_cond = infty_norm(Adx.to_eigen()) <= bound;

  for (i64 iter = 0; iter < qpmodel.n_in; ++iter) {
    T Cdx_i = Cdx.to_eigen()[iter];
    if (qp_scaled.u.to_eigen()[iter] <= 1.E20 &&
        qp_scaled.l.to_eigen()[iter] >= -1.E20) {
      first_cond = first_cond && Cdx_i <= bound && Cdx_i >= bound_neg;
    } else if (qp_scaled.u.to_eigen()[iter] > 1.E20) {
      first_cond = first_cond && Cdx_i >= bound_neg;
    } else if (qp_scaled.l.to_eigen()[iter] < -1.E20) {
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
 * Derives the global primal and dual residual of the QP problem for determining
 * whether the solution is reached (at the desired accuracy).
 *
 * @param primal_residual_eq_scaled vector storing the primal equality residual.
 * @param primal_residual_in_scaled_lo vector storing the primal lower bound
 * inequality residual.
 * @param primal_residual_in_scaled_up vector storing the primal uppder bound
 * inequality residual.
 * @param dual_residual_scaled vector storing the dual residual.
 * @param primal_feasibility_eq_rhs_0 scalar variable used when using a relative
 * stopping criterion.
 * @param primal_feasibility_in_rhs_0 scalar variable used when using a relative
 * stopping criterion.
 * @param dual_feasibility_rhs_0 scalar variable used when using a relative
 * stopping criterion.
 * @param dual_feasibility_rhs_1 scalar variable used when using a relative
 * stopping criterion.
 * @param dual_feasibility_rhs_3 scalar variable used when using a relative
 * stopping criterion.
 * @param precond preconditioner.
 * @param data model of the problem.
 * @param qp_scaled view on the scaled version of the qp problem.
 * @param x_e current estimate of primal variable x.
 * @param y_e current estimate of equality constrained lagrange multiplier.
 * @param z_e current estimate of inequality constrained lagrange multiplier.
 * @param stak stack.
 */
template<typename T, typename I, typename P>
auto
unscaled_primal_dual_residual(
  const Workspace<T, I>& work,
  Results<T>& results,
  VecMapMut<T> primal_residual_eq_scaled,
  VecMapMut<T> primal_residual_in_scaled_lo,
  VecMapMut<T> primal_residual_in_scaled_up,
  VecMapMut<T> dual_residual_scaled,
  T& primal_feasibility_eq_rhs_0,
  T& primal_feasibility_in_rhs_0,
  T& dual_feasibility_rhs_0,
  T& dual_feasibility_rhs_1,
  T& dual_feasibility_rhs_3,
  T& rhs_duality_gap,
  const P& precond,
  Model<T, I> const& data,
  const QpView<T, I> qp_scaled,
  VecMapMut<T> x_e,
  VecMapMut<T> y_e,
  VecMapMut<T> z_e,
  proxsuite::linalg::veg::dynstack::DynStackMut stack)
  -> proxsuite::linalg::veg::Tuple<T, T>
{
  isize n = x_e.rows();

  LDLT_TEMP_VEC_UNINIT(T, tmp, n, stack);
  dual_residual_scaled = qp_scaled.g.to_eigen();
  {
    tmp.setZero();
    noalias_symhiv_add(tmp, qp_scaled.H.to_eigen(), x_e);
    dual_residual_scaled += tmp;

    precond.unscale_dual_residual_in_place(
      { proxqp::from_eigen, tmp }); // contains unscaled Hx
    dual_feasibility_rhs_0 = infty_norm(tmp);
    precond.unscale_primal_in_place({ proxqp::from_eigen, x_e });
    results.info.duality_gap = x_e.dot(data.g); // contains gTx
    rhs_duality_gap = std::fabs(results.info.duality_gap);

    const T xHx = (tmp).dot(x_e);
    results.info.duality_gap += xHx;
    rhs_duality_gap = std::max(rhs_duality_gap, std::abs(xHx));
    tmp += data.g; // contains now Hx+g
    precond.scale_primal_in_place({ proxqp::from_eigen, x_e });

    precond.unscale_dual_in_place_eq({ proxsuite::proxqp::from_eigen, y_e });
    const T by = (data.b).dot(y_e);
    results.info.duality_gap += by;
    rhs_duality_gap = std::max(rhs_duality_gap, std::abs(by));
    precond.scale_dual_in_place_eq({ proxsuite::proxqp::from_eigen, y_e });

    precond.unscale_dual_in_place_in({ proxsuite::proxqp::from_eigen, z_e });

    const T zl =
      helpers::select(work.active_set_low, results.z, 0)
        .dot(helpers::at_least(data.l, -helpers::infinite_bound<T>::value()));
    results.info.duality_gap += zl;
    rhs_duality_gap = std::max(rhs_duality_gap, std::abs(zl));

    const T zu =
      helpers::select(work.active_set_up, results.z, 0)
        .dot(helpers::at_most(data.u, helpers::infinite_bound<T>::value()));
    results.info.duality_gap += zu;
    rhs_duality_gap = std::max(rhs_duality_gap, std::abs(zu));

    precond.scale_dual_in_place_in({ proxsuite::proxqp::from_eigen, z_e });
  }

  {
    auto ATy = tmp;
    ATy.setZero();
    primal_residual_eq_scaled.setZero();

    detail::noalias_gevmmv_add(
      primal_residual_eq_scaled, ATy, qp_scaled.AT.to_eigen(), x_e, y_e);

    dual_residual_scaled += ATy;

    precond.unscale_dual_residual_in_place({ proxqp::from_eigen, ATy });
    dual_feasibility_rhs_1 = infty_norm(ATy);
  }

  {
    auto CTz = tmp;
    CTz.setZero();
    primal_residual_in_scaled_up.setZero();

    detail::noalias_gevmmv_add(
      primal_residual_in_scaled_up, CTz, qp_scaled.CT.to_eigen(), x_e, z_e);

    dual_residual_scaled += CTz;

    precond.unscale_dual_residual_in_place({ proxqp::from_eigen, CTz });
    dual_feasibility_rhs_3 = infty_norm(CTz);
  }
  precond.unscale_primal_residual_in_place_eq(
    { proxqp::from_eigen, primal_residual_eq_scaled });
  primal_feasibility_eq_rhs_0 = infty_norm(primal_residual_eq_scaled);

  precond.unscale_primal_residual_in_place_in(
    { proxqp::from_eigen, primal_residual_in_scaled_up });
  primal_feasibility_in_rhs_0 = infty_norm(primal_residual_in_scaled_up);

  auto b = data.b;
  auto l = data.l;
  auto u = data.u;
  primal_residual_in_scaled_lo =
    helpers::positive_part(primal_residual_in_scaled_up - u) +
    helpers::negative_part(primal_residual_in_scaled_up - l);

  primal_residual_eq_scaled -= b;
  T primal_feasibility_eq_lhs = infty_norm(primal_residual_eq_scaled);
  T primal_feasibility_in_lhs = infty_norm(primal_residual_in_scaled_lo);
  T primal_feasibility_lhs =
    std::max(primal_feasibility_eq_lhs, primal_feasibility_in_lhs);

  // scaled Ax - b
  precond.scale_primal_residual_in_place_eq(
    { proxqp::from_eigen, primal_residual_eq_scaled });
  // scaled Cx
  precond.scale_primal_residual_in_place_in(
    { proxqp::from_eigen, primal_residual_in_scaled_up });

  precond.unscale_dual_residual_in_place(
    { proxqp::from_eigen, dual_residual_scaled });
  T dual_feasibility_lhs = infty_norm(dual_residual_scaled);
  precond.scale_dual_residual_in_place(
    { proxqp::from_eigen, dual_residual_scaled });

  return proxsuite::linalg::veg::tuplify(primal_feasibility_lhs,
                                         dual_feasibility_lhs);
}

} // namespace detail
} // namespace sparse
} // namespace proxqp
} // namespace proxsuite

namespace Eigen {
namespace internal {
template<typename T, typename I>
struct traits<proxsuite::proxqp::sparse::detail::AugmentedKkt<T, I>>
  : Eigen::internal::traits<Eigen::SparseMatrix<T, Eigen::ColMajor, I>>
{};

template<typename Rhs, typename T, typename I>
struct generic_product_impl<
  proxsuite::proxqp::sparse::detail::AugmentedKkt<T, I>,
  Rhs,
  SparseShape,
  DenseShape,
  GemvProduct>
  : generic_product_impl_base<
      proxsuite::proxqp::sparse::detail::AugmentedKkt<T, I>,
      Rhs,
      generic_product_impl<
        proxsuite::proxqp::sparse::detail::AugmentedKkt<T, I>,
        Rhs>>
{
  using Mat_ = proxsuite::proxqp::sparse::detail::AugmentedKkt<T, I>;

  using Scalar = typename Product<Mat_, Rhs>::Scalar;

  template<typename Dst>
  static void scaleAndAddTo(Dst& dst,
                            Mat_ const& lhs,
                            Rhs const& rhs,
                            PROXSUITE_MAYBE_UNUSED Scalar const& alpha)
  {
    using proxsuite::linalg::veg::isize;

    VEG_ASSERT(alpha == Scalar(1));
    proxsuite::proxqp::sparse::detail::noalias_symhiv_add(
      dst, lhs._.kkt_active.to_eigen(), rhs);

    {
      isize n = lhs._.n;
      isize n_eq = lhs._.n_eq;
      isize n_in = lhs._.n_in;

      auto dst_x = dst.head(n);
      auto dst_y = dst.segment(n, n_eq);
      auto dst_z = dst.tail(n_in);

      auto rhs_x = rhs.head(n);
      auto rhs_y = rhs.segment(n, n_eq);
      auto rhs_z = rhs.tail(n_in);

      dst_x += lhs._.rho * rhs_x;
      dst_y += (-1 / lhs._.mu_eq) * rhs_y;
      for (isize i = 0; i < n_in; ++i) {
        dst_z[i] +=
          (lhs._.active_constraints[i] ? -1 / lhs._.mu_in : T(1)) * rhs_z[i];
      }
    }
  }
};
} // namespace internal
} // namespace Eigen

#endif /* end of include guard PROXSUITE_PROXQP_SPARSE_UTILS_HPP */
