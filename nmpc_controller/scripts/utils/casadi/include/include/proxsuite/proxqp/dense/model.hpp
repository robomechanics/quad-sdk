//
// Copyright (c) 2022 INRIA
//
/** \file */
#ifndef PROXSUITE_PROXQP_DENSE_MODEL_HPP
#define PROXSUITE_PROXQP_DENSE_MODEL_HPP

#include <Eigen/Core>
#include "proxsuite/linalg/veg/type_traits/core.hpp"
#include "proxsuite/proxqp/dense/fwd.hpp"
#include "proxsuite/proxqp/sparse/model.hpp"

namespace proxsuite {
namespace proxqp {
namespace dense {
///
/// @brief This class stores the model of the QP problem.
///
/*!
 * Model class of the dense solver storing the QP problem.
 */
template<typename T>
struct Model
{

  ///// QP STORAGE
  Mat<T> H;
  Vec<T> g;
  Mat<T> A;
  Mat<T> C;
  Vec<T> b;
  Vec<T> u;
  Vec<T> l;

  ///// model sizes
  isize dim;
  isize n_eq;
  isize n_in;
  isize n_total;

  /*!
   * Default constructor.
   * @param dim primal variable dimension.
   * @param n_eq number of equality constraints.
   * @param n_in number of inequality constraints.
   */
  Model(isize dim, isize n_eq, isize n_in)
    : H(dim, dim)
    , g(dim)
    , A(n_eq, dim)
    , C(n_in, dim)
    , b(n_eq)
    , u(n_in)
    , l(n_in)
    , dim(dim)
    , n_eq(n_eq)
    , n_in(n_in)
    , n_total(dim + n_eq + n_in)
  {
    PROXSUITE_THROW_PRETTY(dim == 0,
                           std::invalid_argument,
                           "wrong argument size: the dimension wrt the primal "
                           "variable x should be strictly positive.");

    const T infinite_bound_value = helpers::infinite_bound<T>::value();

    H.setZero();
    g.setZero();
    A.setZero();
    C.setZero();
    b.setZero();
    u.fill(+infinite_bound_value); // in case it appears u is nullopt (i.e., the
                                   // problem is only lower bounded)
    l.fill(-infinite_bound_value); // in case it appears l is nullopt (i.e., the
                                   // problem is only upper bounded)
  }

  proxsuite::proxqp::sparse::SparseModel<T> to_sparse()
  {
    SparseMat<T> H_sparse = H.sparseView();
    SparseMat<T> A_sparse = A.sparseView();
    SparseMat<T> C_sparse = C.sparseView();
    proxsuite::proxqp::sparse::SparseModel<T> res{ H_sparse, g, A_sparse, b,
                                                   C_sparse, u, l };
    return res;
  }

  bool is_valid()
  {
#define PROXSUITE_CHECK_SIZE(size, expected_size)                              \
  if (size != 0) {                                                             \
    if (!(size == expected_size))                                              \
      return false;                                                            \
  }

    // check that all matrices and vectors of qpmodel have the correct size
    // and that H and C have expected properties
    PROXSUITE_CHECK_SIZE(g.size(), dim);
    PROXSUITE_CHECK_SIZE(b.size(), n_eq);
    PROXSUITE_CHECK_SIZE(u.size(), n_in);
    PROXSUITE_CHECK_SIZE(l.size(), n_in);
    if (H.size()) {
      PROXSUITE_CHECK_SIZE(H.rows(), dim);
      PROXSUITE_CHECK_SIZE(H.cols(), dim);
      if (!H.isApprox(H.transpose(), 0.0))
        return false;
    }
    if (A.size()) {
      PROXSUITE_CHECK_SIZE(A.rows(), n_eq);
      PROXSUITE_CHECK_SIZE(A.cols(), dim);
    }
    if (C.size()) {
      PROXSUITE_CHECK_SIZE(C.rows(), n_in);
      PROXSUITE_CHECK_SIZE(C.cols(), dim);
      if (C.isZero())
        return false;
    }
    return true;
#undef PROXSUITE_CHECK_SIZE
  }
};

template<typename T>
bool
operator==(const Model<T>& model1, const Model<T>& model2)
{
  bool value = model1.dim == model2.dim && model1.n_eq == model2.n_eq &&
               model1.n_in == model2.n_in && model1.n_total == model2.n_total &&
               model1.H == model2.H && model1.g == model2.g &&
               model1.A == model2.A && model1.b == model2.b &&
               model1.C == model2.C && model1.l == model2.l &&
               model1.u == model2.u;
  return value;
}

template<typename T>
bool
operator!=(const Model<T>& model1, const Model<T>& model2)
{
  return !(model1 == model2);
}

} // namespace dense
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_DENSE_MODEL_HPP */
