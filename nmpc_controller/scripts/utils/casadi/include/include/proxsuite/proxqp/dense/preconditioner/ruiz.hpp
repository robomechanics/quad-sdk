//
// Copyright (c) 2022 INRIA
//
/**
 * @file ruiz.hpp
 */
#ifndef PROXSUITE_PROXQP_DENSE_PRECOND_RUIZ_HPP
#define PROXSUITE_PROXQP_DENSE_PRECOND_RUIZ_HPP

#include "proxsuite/proxqp/dense/views.hpp"
#include "proxsuite/proxqp/dense/fwd.hpp"
#include <proxsuite/linalg/dense/core.hpp>
#include <ostream>

#include <Eigen/Core>

namespace proxsuite {
namespace proxqp {
enum struct Symmetry
{
  general,
  lower,
  upper,
};
namespace dense {
namespace detail {

template<typename T>
auto
ruiz_scale_qp_in_place( //
  VectorViewMut<T> delta_,
  std::ostream* logger_ptr,
  QpViewBoxMut<T> qp,
  T epsilon,
  isize max_iter,
  Symmetry sym,
  proxsuite::linalg::veg::dynstack::DynStackMut stack) -> T
{

  T c(1);
  auto S = delta_.to_eigen();

  auto H = qp.H.to_eigen();
  auto g = qp.g.to_eigen();
  auto A = qp.A.to_eigen();
  auto b = qp.b.to_eigen();
  auto C = qp.C.to_eigen();
  auto u = qp.u.to_eigen();
  auto l = qp.l.to_eigen();

  static constexpr T machine_eps = std::numeric_limits<T>::epsilon();
  /*
   * compute equilibration parameters and scale in place the qp following
   * algorithm
   *
   * modified: removed g in gamma computation
   */

  isize n = qp.H.rows;
  isize n_eq = qp.A.rows;
  isize n_in = qp.C.rows;

  T gamma = T(1);

  LDLT_TEMP_VEC(T, delta, n + n_eq + n_in, stack);

  i64 iter = 1;

  while (infty_norm((1 - delta.array()).matrix()) > epsilon) {
    if (logger_ptr != nullptr) {
      *logger_ptr                                   //
        << "j : "                                   //
        << iter                                     //
        << " ; error : "                            //
        << infty_norm((1 - delta.array()).matrix()) //
        << "\n\n";
    }
    if (iter == max_iter) {
      break;
    } else {
      ++iter;
    }

    // normalization vector
    {
      for (isize k = 0; k < n; ++k) {
        switch (sym) {
          case Symmetry::upper: { // upper triangular part
            T aux = sqrt(std::max({
              infty_norm(H.col(k).head(k)),
              infty_norm(H.row(k).tail(n - k)),
              n_eq > 0 ? infty_norm(A.col(k)) : T(0),
              n_in > 0 ? infty_norm(C.col(k)) : T(0),
            }));
            if (aux == T(0)) {
              delta(k) = T(1);
            } else {
              delta(k) = T(1) / (aux + machine_eps);
            }
            break;
          }
          case Symmetry::lower: { // lower triangular part

            T aux = sqrt(std::max({
              infty_norm(H.col(k).head(k)),
              infty_norm(H.col(k).tail(n - k)),
              n_eq > 0 ? infty_norm(A.col(k)) : T(0),
              n_in > 0 ? infty_norm(C.col(k)) : T(0),
            }));
            if (aux == T(0)) {
              delta(k) = T(1);
            } else {
              delta(k) = T(1) / (aux + machine_eps);
            }
            break;
          }
          case Symmetry::general: {

            T aux = sqrt(std::max({
              infty_norm(H.col(k)),
              n_eq > 0 ? infty_norm(A.col(k)) : T(0),
              n_in > 0 ? infty_norm(C.col(k)) : T(0),
            }));
            if (aux == T(0)) {
              delta(k) = T(1);
            } else {
              delta(k) = T(1) / (aux + machine_eps);
            }
            break;
          }
        }
      }

      for (isize k = 0; k < n_eq; ++k) {
        T aux = sqrt(infty_norm(A.row(k)));
        if (aux == T(0)) {
          delta(n + k) = T(1);
        } else {
          delta(n + k) = T(1) / (aux + machine_eps);
        }
      }
      for (isize k = 0; k < n_in; ++k) {
        T aux = sqrt(infty_norm(C.row(k)));
        if (aux == T(0)) {
          delta(k + n + n_eq) = T(1);
        } else {
          delta(k + n + n_eq) = T(1) / (aux + machine_eps);
        }
      }
    }
    {

      // normalize A and C
      A = delta.segment(n, n_eq).asDiagonal() * A * delta.head(n).asDiagonal();
      C = delta.tail(n_in).asDiagonal() * C * delta.head(n).asDiagonal();
      // normalize vectors
      g.array() *= delta.head(n).array();
      b.array() *= delta.middleRows(n, n_eq).array();
      u.array() *= delta.tail(n_in).array();
      l.array() *= delta.tail(n_in).array();

      // normalize H
      switch (sym) {
        case Symmetry::upper: {
          // upper triangular part
          for (isize j = 0; j < n; ++j) {
            H.col(j).head(j + 1) *= delta(j);
          }
          // normalisation des lignes
          for (isize i = 0; i < n; ++i) {
            H.row(i).tail(n - i) *= delta(i);
          }
          break;
        }
        case Symmetry::lower: {
          // lower triangular part
          for (isize j = 0; j < n; ++j) {
            H.col(j).tail(n - j) *= delta(j);
          }
          // normalisation des lignes
          for (isize i = 0; i < n; ++i) {
            H.row(i).head(i + 1) *= delta(i);
          }
          break;
        }
        case Symmetry::general: {
          // all matrix
          H = delta.head(n).asDiagonal() * H * delta.head(n).asDiagonal();
          break;
        }
        default:
          break;
      }

      // additional normalization for the cost function
      switch (sym) {
        case Symmetry::upper: {
          // upper triangular part
          T tmp = T(0);
          for (isize j = 0; j < n; ++j) {
            tmp += proxqp::dense::infty_norm(H.row(j).tail(n - j));
          }
          gamma = 1 / std::max(tmp / T(n), T(1));
          break;
        }
        case Symmetry::lower: {
          // lower triangular part
          T tmp = T(0);
          for (isize j = 0; j < n; ++j) {
            tmp += proxqp::dense::infty_norm(H.col(j).tail(n - j));
          }
          gamma = 1 / std::max(tmp / T(n), T(1));
          break;
        }
        case Symmetry::general: {
          // all matrix
          gamma =
            1 /
            std::max(T(1),
                     (H.colwise().template lpNorm<Eigen::Infinity>()).mean());
          break;
        }
        default:
          break;
      }

      g *= gamma;
      H *= gamma;

      S.array() *= delta.array(); // coefficientwise product
      c *= gamma;
    }
  }
  return c;
}
} // namespace detail

namespace preconditioner {

template<typename T>
struct RuizEquilibration
{
  Vec<T> delta;
  T c;
  isize dim;
  T epsilon;
  i64 max_iter;
  Symmetry sym;

  std::ostream* logger_ptr = nullptr;
  /*!
   * Default constructor.
   * @param dim primal variable dimension.
   * @param n_eq_in number of equality and inequality constraints.
   * @param epsilon_ accuracy required for stopping the ruiz equilibration
   * algorithm.
   * @param max_iter_ maximum number of ruiz equilibration iterations.
   * @param sym_ symetry option format of quadratic cost matrix.
   * @param logger parameter for printing or not intermediary results.
   */
  explicit RuizEquilibration(isize dim_,
                             isize n_eq_in,
                             T epsilon_ = T(1e-3),
                             i64 max_iter_ = 10,
                             Symmetry sym_ = Symmetry::general,
                             std::ostream* logger = nullptr)
    : delta(Vec<T>::Ones(dim_ + n_eq_in))
    , c(1)
    , dim(dim_)
    , epsilon(epsilon_)
    , max_iter(max_iter_)
    , sym(sym_)
    , logger_ptr(logger)
  {
  }
  /*!
   * Prints ruiz equilibrator scaling variables.
   */
  void print() const
  {
    // CHANGE: endl to newline
    *logger_ptr << " delta : " << delta << "\n\n";
    *logger_ptr << " c : " << c << "\n\n";
  }
  /*!
   * Determines memory requirements for executing the equilibrator.
   * @param tag tag for specifying entry type.
   * @param n dimension of the primal variable of the model.
   * @param n_eq number of equality constraints.
   * @param n_in number of inequality constraints.
   */
  static auto scale_qp_in_place_req(proxsuite::linalg::veg::Tag<T> tag,
                                    isize n,
                                    isize n_eq,
                                    isize n_in)
    -> proxsuite::linalg::veg::dynstack::StackReq
  {
    return proxsuite::linalg::dense::temp_vec_req(tag, n + n_eq + n_in);
  }

  // H_new = c * head @ H @ head
  // A_new = tail @ A @ head
  // g_new = c * head @ g
  // b_new = tail @ b
  /*!
   * Scales the qp performing the ruiz equilibrator algorithm considering user
   * options.
   * @param qp qp to be scaled (in place).
   * @param execute_preconditioner bool variable specifying whether the qp is
   * scaled using current equilibrator scaling variables, or performing anew the
   * algorithm.
   * @param settings solver's settings.
   * @param stack stack variable used by the equilibrator.
   */
  void scale_qp_in_place(QpViewBoxMut<T> qp,
                         bool execute_preconditioner,
                         const isize max_iter,
                         const T epsilon,
                         proxsuite::linalg::veg::dynstack::DynStackMut stack)
  {
    if (execute_preconditioner) {
      delta.setOnes();
      c = detail::ruiz_scale_qp_in_place({ proxqp::from_eigen, delta },
                                         logger_ptr,
                                         qp,
                                         epsilon,
                                         max_iter,
                                         sym,
                                         stack);
    } else {

      auto H = qp.H.to_eigen();
      auto g = qp.g.to_eigen();
      auto A = qp.A.to_eigen();
      auto b = qp.b.to_eigen();
      auto C = qp.C.to_eigen();
      auto u = qp.u.to_eigen();
      auto l = qp.l.to_eigen();
      isize n = qp.H.rows;
      isize n_eq = qp.A.rows;
      isize n_in = qp.C.rows;

      // normalize A and C
      A = delta.segment(n, n_eq).asDiagonal() * A * delta.head(n).asDiagonal();
      C = delta.tail(n_in).asDiagonal() * C * delta.head(n).asDiagonal();

      // normalize H
      switch (sym) {
        case Symmetry::upper: {
          // upper triangular part
          for (isize j = 0; j < n; ++j) {
            H.col(j).head(j + 1) *= delta(j);
          }
          // normalisation des lignes
          for (isize i = 0; i < n; ++i) {
            H.row(i).tail(n - i) *= delta(i);
          }
          break;
        }
        case Symmetry::lower: {
          // lower triangular part
          for (isize j = 0; j < n; ++j) {
            H.col(j).tail(n - j) *= delta(j);
          }
          // normalisation des lignes
          for (isize i = 0; i < n; ++i) {
            H.row(i).head(i + 1) *= delta(i);
          }
          break;
        }
        case Symmetry::general: {
          // all matrix
          H = delta.head(n).asDiagonal() * H * delta.head(n).asDiagonal();
          break;
        }
        default:
          break;
      }

      // normalize vectors
      g.array() *= delta.head(n).array();
      b.array() *= delta.segment(n, n_eq).array();
      l.array() *= delta.tail(n_in).array();
      u.array() *= delta.tail(n_in).array();

      g *= c;
      H *= c;
    }
  }
  /*!
   * Scales the qp performing the ruiz equilibrator algorithm considering user
   * options.
   * @param qp qp model.
   * @param scaled_qp qp to be scaled.
   * @param tmp_delta_preallocated temporary variable used for performing the
   * equilibration.
   */
  void scale_qp(const QpViewBox<T> qp,
                QpViewBoxMut<T> scaled_qp,
                VectorViewMut<T> tmp_delta_preallocated) const
  {

    /*
     * scaled_qp is scaled, whereas first qp is not
     * the procedure computes as well equilibration parameters using default
     * parameters
     */

    scaled_qp.H.to_eigen() = qp.H.to_eigen();
    scaled_qp.A.to_eigen() = qp.A.to_eigen();
    scaled_qp.C.to_eigen() = qp.C.to_eigen();
    scaled_qp.g.to_eigen() = qp.g.to_eigen();
    scaled_qp.b.to_eigen() = qp.b.to_eigen();
    scaled_qp.d.to_eigen() = qp.d.to_eigen();

    scale_qp_in_place(scaled_qp, tmp_delta_preallocated, epsilon, max_iter);
  }
  // modifies variables in place
  /*!
   * Scales a primal variable in place.
   * @param primal primal variable.
   */
  void scale_primal_in_place(VectorViewMut<T> primal) const
  {
    primal.to_eigen().array() /= delta.array().head(dim);
  }
  /*!
   * Scales a dual variable in place.
   * @param dual dual variable (includes all equalities and inequalities
   * constraints).
   */
  void scale_dual_in_place(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() = dual.as_const().to_eigen().array() /
                              delta.tail(delta.size() - dim).array() * c;
  }
  /*!
   * Scales a dual equality constrained variable in place.
   * @param dual dual variable (includes equalities constraints only).
   */
  void scale_dual_in_place_eq(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() =
      dual.as_const().to_eigen().array() /
      delta.middleRows(dim, dual.to_eigen().size()).array() * c;
  }
  /*!
   * Scales a dual inequality constrained variable in place.
   * @param dual dual variable (includes inequalities constraints only).
   */
  void scale_dual_in_place_in(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() = dual.as_const().to_eigen().array() /
                              delta.tail(dual.to_eigen().size()).array() * c;
  }
  /*!
   * Unscales a primal variable in place.
   * @param primal primal variable.
   */
  void unscale_primal_in_place(VectorViewMut<T> primal) const
  {
    primal.to_eigen().array() *= delta.array().head(dim);
  }
  /*!
   * Unscales a dual variable in place.
   * @param dual dual variable (includes equalities constraints only).
   */
  void unscale_dual_in_place(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() = dual.as_const().to_eigen().array() *
                              delta.tail(delta.size() - dim).array() / c;
  }
  /*!
   * Unscales a dual equality constrained variable in place.
   * @param dual dual variable (includes equalities constraints only).
   */
  void unscale_dual_in_place_eq(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() =
      dual.as_const().to_eigen().array() *
      delta.middleRows(dim, dual.to_eigen().size()).array() / c;
  }
  /*!
   * Unscales a dual inequality constrained variable in place.
   * @param dual dual variable (includes inequalities constraints only).
   */
  void unscale_dual_in_place_in(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() = dual.as_const().to_eigen().array() *
                              delta.tail(dual.to_eigen().size()).array() / c;
  }
  // modifies residuals in place
  /*!
   * Scales a primal residual in place.
   * @param primal primal residual (includes equality and inequality
   * constraints)
   */
  void scale_primal_residual_in_place(VectorViewMut<T> primal) const
  {
    primal.to_eigen().array() *= delta.tail(delta.size() - dim).array();
  }

  /*!
   * Scales a primal equality constraint residual in place.
   * @param primal primal equality constraint residual.
   */
  void scale_primal_residual_in_place_eq(VectorViewMut<T> primal_eq) const
  {
    primal_eq.to_eigen().array() *=
      delta.middleRows(dim, primal_eq.to_eigen().size()).array();
  }
  /*!
   * Scales a primal inequality constraint residual in place.
   * @param primal primal inequality constraint residual.
   */
  void scale_primal_residual_in_place_in(VectorViewMut<T> primal_in) const
  {
    primal_in.to_eigen().array() *=
      delta.tail(primal_in.to_eigen().size()).array();
  }
  /*!
   * Scales a dual residual in place.
   * @param dual dual residual.
   */
  void scale_dual_residual_in_place(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() *= delta.head(dim).array() * c;
  }
  /*!
   * Unscales a primal residual in place.
   * @param primal primal residual (includes equality and inequality
   * constraints).
   */
  void unscale_primal_residual_in_place(VectorViewMut<T> primal) const
  {
    primal.to_eigen().array() /= delta.tail(delta.size() - dim).array();
  }
  /*!
   * Unscales a primal equality constraint residual in place.
   * @param primal primal equality constraint residual.
   */
  void unscale_primal_residual_in_place_eq(VectorViewMut<T> primal_eq) const
  {
    primal_eq.to_eigen().array() /=
      delta.middleRows(dim, primal_eq.to_eigen().size()).array();
  }
  /*!
   * Unscales a primal inequality constraint residual in place.
   * @param primal primal inequality constraint residual.
   */
  void unscale_primal_residual_in_place_in(VectorViewMut<T> primal_in) const
  {
    primal_in.to_eigen().array() /=
      delta.tail(primal_in.to_eigen().size()).array();
  }
  /*!
   * Unscales a dual residual in place.
   * @param dual dual residual.
   */
  void unscale_dual_residual_in_place(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() /= delta.head(dim).array() * c;
  }
};

} // namespace preconditioner
} // namespace dense
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_DENSE_PRECOND_RUIZ_HPP */
