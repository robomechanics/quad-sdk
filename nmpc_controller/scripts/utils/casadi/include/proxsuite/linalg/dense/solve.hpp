/** \file */
//
// Copyright (c) 2022 INRIA
//
#ifndef PROXSUITE_LINALG_DENSE_LDLT_SOLVE_HPP
#define PROXSUITE_LINALG_DENSE_LDLT_SOLVE_HPP

#include "proxsuite/linalg/dense/core.hpp"
#include <Eigen/Core>

namespace proxsuite {
namespace linalg {
namespace dense {
namespace _detail {
template<typename Mat, typename Rhs>
void
solve_impl(Mat ld, Rhs rhs)
{
  auto l = ld.template triangularView<Eigen::UnitLower>();
  auto lt = util::trans(ld).template triangularView<Eigen::UnitUpper>();
  auto d = util::diagonal(ld);

  l.solveInPlace(rhs);
  rhs = rhs.cwiseQuotient(d);
  lt.solveInPlace(rhs);
}
} // namespace _detail
template<typename Mat, typename Rhs>
void
solve(Mat const& mat, Rhs&& rhs)
{
  _detail::solve_impl(util::to_view(mat), util::to_view_dyn_rows(rhs));
}
} // namespace dense
} // namespace linalg
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_LINALG_DENSE_LDLT_SOLVE_HPP */
