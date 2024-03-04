//
// Copyright (c) 2022 INRIA
//
/** \file */

#ifndef PROXSUITE_PROXQP_SPARSE_PRECOND_RUIZ_HPP
#define PROXSUITE_PROXQP_SPARSE_PRECOND_RUIZ_HPP

#include "proxsuite/proxqp/sparse/fwd.hpp"

namespace proxsuite {
namespace proxqp {
namespace sparse {

namespace preconditioner {
enum struct Symmetry
{
  LOWER,
  UPPER,
};

namespace detail {
template<typename T, typename I>
void
rowwise_infty_norm(T* row_norm, proxsuite::linalg::sparse::MatRef<T, I> m)
{
  using namespace proxsuite::linalg::sparse::util;

  I const* mi = m.row_indices();
  T const* mx = m.values();

  for (usize j = 0; j < usize(m.ncols()); ++j) {
    auto col_start = m.col_start(j);
    auto col_end = m.col_end(j);

    for (usize p = col_start; p < col_end; ++p) {
      usize i = zero_extend(mi[p]);
      T mij = fabs(mx[p]);
      row_norm[i] = std::max(row_norm[i], mij);
    }
  }
}

template<typename T, typename I>
void
colwise_infty_norm_symhi(T* col_norm, proxsuite::linalg::sparse::MatRef<T, I> h)
{
  using namespace proxsuite::linalg::sparse::util;

  I const* hi = h.row_indices();
  T const* hx = h.values();

  for (usize j = 0; j < usize(h.ncols()); ++j) {
    auto col_start = h.col_start(j);
    auto col_end = h.col_end(j);

    T norm_j = 0;

    for (usize p = col_start; p < col_end; ++p) {
      usize i = zero_extend(hi[p]);
      if (i > j) {
        break;
      }

      T hij = fabs(hx[p]);
      norm_j = std::max(norm_j, hij);
      col_norm[i] = std::max(col_norm[i], hij);
    }

    col_norm[j] = norm_j;
  }
}

template<typename T, typename I>
void
colwise_infty_norm_symlo(T* col_norm, proxsuite::linalg::sparse::MatRef<T, I> h)
{
  using namespace proxsuite::linalg::sparse::util;

  I const* hi = h.row_indices();
  T const* hx = h.values();

  for (usize j = 0; j < usize(h.ncols()); ++j) {
    auto col_start = h.col_start(j);
    auto col_end = h.col_end(j);

    T norm_j = 0;

    if (col_end > col_start) {
      usize p = col_end;
      while (true) {
        --p;
        usize i = zero_extend(hi[p]);
        if (i < j) {
          break;
        }

        T hij = fabs(hx[p]);
        norm_j = std::max(norm_j, hij);
        col_norm[i] = std::max(col_norm[i], hij);

        if (p <= col_start) {
          break;
        }
      }
    }
    col_norm[j] = std::max(col_norm[j], norm_j);
  }
}

template<typename T, typename I>
auto
ruiz_scale_qp_in_place( //
  VectorViewMut<T> delta_,
  QpViewMut<T, I> qp,
  T epsilon,
  isize max_iter,
  Symmetry sym,
  proxsuite::linalg::veg::dynstack::DynStackMut stack) -> T
{

  T c = 1;
  auto S = delta_.to_eigen();

  isize n = qp.H.nrows();
  isize n_eq = qp.AT.ncols();
  isize n_in = qp.CT.ncols();

  T gamma = 1;
  i64 iter = 1;

  LDLT_TEMP_VEC(T, delta, n + n_eq + n_in, stack);

  I* Hi = qp.H.row_indices_mut();
  T* Hx = qp.H.values_mut();

  I* ATi = qp.AT.row_indices_mut();
  T* ATx = qp.AT.values_mut();

  I* CTi = qp.CT.row_indices_mut();
  T* CTx = qp.CT.values_mut();

  T const machine_eps = std::numeric_limits<T>::epsilon();

  while (infty_norm((1 - delta.array()).matrix()) > epsilon) {
    if (iter == max_iter) {
      break;
    } else {
      ++iter;
    }

    // norm_infty of each column of A (resp. C), i.e.,
    // each row of AT (resp. CT)
    {
      auto _a_infty_norm = stack.make_new(proxsuite::linalg::veg::Tag<T>{}, n);
      auto _c_infty_norm = stack.make_new(proxsuite::linalg::veg::Tag<T>{}, n);
      auto _h_infty_norm = stack.make_new(proxsuite::linalg::veg::Tag<T>{}, n);
      T* a_infty_norm = _a_infty_norm.ptr_mut();
      T* c_infty_norm = _c_infty_norm.ptr_mut();
      T* h_infty_norm = _h_infty_norm.ptr_mut();

      detail::rowwise_infty_norm(a_infty_norm, qp.AT.as_const());
      detail::rowwise_infty_norm(c_infty_norm, qp.CT.as_const());
      switch (sym) {
        case Symmetry::LOWER: {
          detail::colwise_infty_norm_symlo(h_infty_norm, qp.H.as_const());
          break;
        }
        case Symmetry::UPPER: {
          detail::colwise_infty_norm_symhi(h_infty_norm, qp.H.as_const());
          break;
        }
      }

      for (isize j = 0; j < n; ++j) {
        delta(j) = T(1) / (machine_eps + sqrt(std::max({
                                           h_infty_norm[j],
                                           a_infty_norm[j],
                                           c_infty_norm[j],
                                         })));
      }
    }
    using namespace proxsuite::linalg::sparse::util;
    for (usize j = 0; j < usize(n_eq); ++j) {
      T a_row_norm = 0;
      qp.AT.to_eigen();
      usize col_start = qp.AT.col_start(j);
      usize col_end = qp.AT.col_end(j);

      for (usize p = col_start; p < col_end; ++p) {
        T aji = fabs(ATx[p]);
        a_row_norm = std::max(a_row_norm, aji);
      }

      delta(n + isize(j)) = T(1) / (machine_eps + sqrt(a_row_norm));
    }

    for (usize j = 0; j < usize(n_in); ++j) {
      T c_row_norm = 0;
      usize col_start = qp.CT.col_start(j);
      usize col_end = qp.CT.col_end(j);

      for (usize p = col_start; p < col_end; ++p) {
        T cji = fabs(CTx[p]);
        c_row_norm = std::max(c_row_norm, cji);
      }

      delta(n + n_eq + isize(j)) = T(1) / (machine_eps + sqrt(c_row_norm));
    }

    // normalize A
    for (usize j = 0; j < usize(n_eq); ++j) {
      usize col_start = qp.AT.col_start(j);
      usize col_end = qp.AT.col_end(j);

      T delta_j = delta(n + isize(j));

      for (usize p = col_start; p < col_end; ++p) {
        usize i = zero_extend(ATi[p]);
        T& aji = ATx[p];
        T delta_i = delta(isize(i));
        aji = delta_i * (aji * delta_j);
      }
    }

    // normalize C
    for (usize j = 0; j < usize(n_in); ++j) {
      usize col_start = qp.CT.col_start(j);
      usize col_end = qp.CT.col_end(j);

      T delta_j = delta(n + n_eq + isize(j));

      for (usize p = col_start; p < col_end; ++p) {
        usize i = zero_extend(CTi[p]);
        T& cji = CTx[p];
        T delta_i = delta(isize(i));
        cji = delta_i * (cji * delta_j);
      }
    }

    // normalize H
    switch (sym) {
      case Symmetry::LOWER: {
        for (usize j = 0; j < usize(n); ++j) {
          usize col_start = qp.H.col_start(j);
          usize col_end = qp.H.col_end(j);
          T delta_j = delta(isize(j));

          if (col_end > col_start) {
            usize p = col_end;
            while (true) {
              --p;
              usize i = zero_extend(Hi[p]);
              if (i < j) {
                break;
              }
              Hx[p] = delta_j * Hx[p] * delta(isize(i));

              if (p <= col_start) {
                break;
              }
            }
          }
        }
        break;
      }
      case Symmetry::UPPER: {
        for (usize j = 0; j < usize(n); ++j) {
          usize col_start = qp.H.col_start(j);
          usize col_end = qp.H.col_end(j);
          T delta_j = delta(isize(j));

          for (usize p = col_start; p < col_end; ++p) {
            usize i = zero_extend(Hi[p]);
            if (i > j) {
              break;
            }
            Hx[p] = delta_j * Hx[p] * delta(isize(i));
          }
        }
        break;
      }
    }

    // normalize vectors
    qp.g.to_eigen().array() *= delta.head(n).array();
    qp.b.to_eigen().array() *= delta.segment(n, n_eq).array();
    qp.l.to_eigen().array() *= delta.tail(n_in).array();
    qp.u.to_eigen().array() *= delta.tail(n_in).array();

    // additional normalization
    auto _h_infty_norm = stack.make_new(proxsuite::linalg::veg::Tag<T>{}, n);
    T* h_infty_norm = _h_infty_norm.ptr_mut();

    switch (sym) {
      case Symmetry::LOWER: {
        detail::colwise_infty_norm_symlo(h_infty_norm, qp.H.as_const());
        break;
      }
      case Symmetry::UPPER: {
        detail::colwise_infty_norm_symhi(h_infty_norm, qp.H.as_const());
        break;
      }
    }

    T avg = 0;
    for (isize i = 0; i < n; ++i) {
      avg += h_infty_norm[i];
    }
    avg /= T(n);

    gamma = 1 / std::max(avg, T(1));

    qp.g.to_eigen() *= gamma;
    qp.H.to_eigen() *= gamma;

    S.array() *= delta.array();
    c *= gamma;
  }
  return c;
}
} // namespace detail

template<typename T, typename I>
struct RuizEquilibration
{
  Vec<T> delta;
  isize n;
  T c;
  T epsilon;
  i64 max_iter;
  Symmetry sym;

  std::ostream* logger_ptr = nullptr;

  RuizEquilibration(isize n_,
                    isize n_eq_in,
                    T epsilon_ = T(1e-3),
                    i64 max_iter_ = 10,
                    Symmetry sym_ = Symmetry::UPPER,
                    std::ostream* logger = nullptr)
    : delta(Vec<T>::Ones(n_ + n_eq_in))
    , n(n_)
    , c(1)
    , epsilon(epsilon_)
    , max_iter(max_iter_)
    , sym(sym_)
    , logger_ptr(logger)
  {
    delta.setOnes();
  }

  static auto scale_qp_in_place_req(proxsuite::linalg::veg::Tag<T> tag,
                                    isize n,
                                    isize n_eq,
                                    isize n_in)
    -> proxsuite::linalg::veg::dynstack::StackReq
  {
    return proxsuite::linalg::dense::temp_vec_req(tag, n + n_eq + n_in) &
           proxsuite::linalg::veg::dynstack::StackReq::with_len(tag, 3 * n);
  }

  void scale_qp_in_place(QpViewMut<T, I> qp,
                         bool execute_preconditioner,
                         const isize max_iter,
                         const T epsilon,
                         proxsuite::linalg::veg::dynstack::DynStackMut stack)
  {
    if (execute_preconditioner) {
      delta.setOnes();
      c = detail::ruiz_scale_qp_in_place( //
        { proxqp::from_eigen, delta },
        qp,
        epsilon,
        max_iter,
        sym,
        stack);
    } else {
      using proxsuite::linalg::sparse::util::zero_extend;
      isize n = qp.H.nrows();
      isize n_eq = qp.AT.ncols();
      isize n_in = qp.CT.ncols();

      I* Hi = qp.H.row_indices_mut();
      T* Hx = qp.H.values_mut();

      I* ATi = qp.AT.row_indices_mut();
      T* ATx = qp.AT.values_mut();

      I* CTi = qp.CT.row_indices_mut();
      T* CTx = qp.CT.values_mut();

      // normalize A
      for (usize j = 0; j < usize(n_eq); ++j) {
        usize col_start = qp.AT.col_start(j);
        usize col_end = qp.AT.col_end(j);

        T delta_j = delta(n + isize(j));

        for (usize p = col_start; p < col_end; ++p) {
          usize i = zero_extend(ATi[p]);
          T& aji = ATx[p];
          T delta_i = delta(isize(i));
          aji = delta_i * (aji * delta_j);
        }
      }

      // normalize C
      for (usize j = 0; j < usize(n_in); ++j) {
        usize col_start = qp.CT.col_start(j);
        usize col_end = qp.CT.col_end(j);

        T delta_j = delta(n + n_eq + isize(j));

        for (usize p = col_start; p < col_end; ++p) {
          usize i = zero_extend(CTi[p]);
          T& cji = CTx[p];
          T delta_i = delta(isize(i));
          cji = delta_i * (cji * delta_j);
        }
      }

      // normalize H
      switch (sym) {
        case Symmetry::LOWER: {
          for (usize j = 0; j < usize(n); ++j) {
            usize col_start = qp.H.col_start(j);
            usize col_end = qp.H.col_end(j);
            T delta_j = delta(isize(j));

            if (col_end > col_start) {
              usize p = col_end;
              while (true) {
                --p;
                usize i = zero_extend(Hi[p]);
                if (i < j) {
                  break;
                }
                Hx[p] = delta_j * Hx[p] * delta(isize(i));

                if (p <= col_start) {
                  break;
                }
              }
            }
          }
          break;
        }
        case Symmetry::UPPER: {
          for (usize j = 0; j < usize(n); ++j) {
            usize col_start = qp.H.col_start(j);
            usize col_end = qp.H.col_end(j);
            T delta_j = delta(isize(j));

            for (usize p = col_start; p < col_end; ++p) {
              usize i = zero_extend(Hi[p]);
              if (i > j) {
                break;
              }
              Hx[p] = delta_j * Hx[p] * delta(isize(i));
            }
          }
          break;
        }
      }

      // normalize vectors
      qp.g.to_eigen().array() *= delta.head(n).array();
      qp.b.to_eigen().array() *= delta.segment(n, n_eq).array();
      qp.l.to_eigen().array() *= delta.tail(n_in).array();
      qp.u.to_eigen().array() *= delta.tail(n_in).array();

      qp.g.to_eigen() *= c;
      qp.H.to_eigen() *= c;
    }
  }

  // modifies variables in place
  void scale_primal_in_place(VectorViewMut<T> primal) const
  {
    primal.to_eigen().array() /= delta.array().head(n);
  }
  void scale_dual_in_place(VectorViewMut<T> dual)
  {
    dual.to_eigen().array() = dual.as_const().to_eigen().array() /
                              delta.tail(delta.size() - n).array() * c;
  }

  void scale_dual_in_place_eq(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() =
      dual.as_const().to_eigen().array() /
      delta.middleRows(n, dual.to_eigen().size()).array() * c;
  }
  void scale_dual_in_place_in(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() = dual.as_const().to_eigen().array() /
                              delta.tail(dual.to_eigen().size()).array() * c;
  }

  void unscale_primal_in_place(VectorViewMut<T> primal) const
  {
    primal.to_eigen().array() *= delta.array().head(n);
  }
  void unscale_dual_in_place(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() = dual.as_const().to_eigen().array() *
                              delta.tail(delta.size() - n).array() / c;
  }

  void unscale_dual_in_place_eq(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() =
      dual.as_const().to_eigen().array() *
      delta.middleRows(n, dual.to_eigen().size()).array() / c;
  }

  void unscale_dual_in_place_in(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() = dual.as_const().to_eigen().array() *
                              delta.tail(dual.to_eigen().size()).array() / c;
  }
  // modifies residuals in place
  void scale_primal_residual_in_place(VectorViewMut<T> primal) const
  {
    primal.to_eigen().array() *= delta.tail(delta.size() - n).array();
  }

  void scale_primal_residual_in_place_eq(VectorViewMut<T> primal_eq) const
  {
    primal_eq.to_eigen().array() *=
      delta.middleRows(n, primal_eq.to_eigen().size()).array();
  }
  void scale_primal_residual_in_place_in(VectorViewMut<T> primal_in) const
  {
    primal_in.to_eigen().array() *=
      delta.tail(primal_in.to_eigen().size()).array();
  }
  void scale_dual_residual_in_place(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() *= delta.head(n).array() * c;
  }
  void unscale_primal_residual_in_place(VectorViewMut<T> primal) const
  {
    primal.to_eigen().array() /= delta.tail(delta.size() - n).array();
  }
  void unscale_primal_residual_in_place_eq(VectorViewMut<T> primal_eq) const
  {
    primal_eq.to_eigen().array() /=
      delta.middleRows(n, primal_eq.to_eigen().size()).array();
  }
  void unscale_primal_residual_in_place_in(VectorViewMut<T> primal_in) const
  {
    primal_in.to_eigen().array() /=
      delta.tail(primal_in.to_eigen().size()).array();
  }
  void unscale_dual_residual_in_place(VectorViewMut<T> dual) const
  {
    dual.to_eigen().array() /= delta.head(n).array() * c;
  }
};

} // namespace preconditioner

} // namespace sparse
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_SPARSE_PRECOND_RUIZ_HPP */
