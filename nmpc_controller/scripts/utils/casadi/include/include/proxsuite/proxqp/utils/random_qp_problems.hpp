#ifndef PROXSUITE_PROXQP_UTILS_RANDOM_QP_PROBLEMS_HPP
#define PROXSUITE_PROXQP_UTILS_RANDOM_QP_PROBLEMS_HPP

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <utility>
#include <proxsuite/proxqp/dense/views.hpp>
#include <proxsuite/proxqp/dense/model.hpp>
#include <proxsuite/proxqp/sparse/model.hpp>
#include <map>
#include <random>

namespace proxsuite {
namespace proxqp {
namespace utils {

using c_int = long long;
using c_float = double;

namespace proxqp = proxsuite::proxqp;

template<typename T, proxqp::Layout L>
using Mat =
  Eigen::Matrix<T,
                Eigen::Dynamic,
                Eigen::Dynamic,
                (L == proxqp::colmajor) ? Eigen::ColMajor : Eigen::RowMajor>;
template<typename T>
using Vec = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template<typename Scalar>
using SparseMat = Eigen::SparseMatrix<Scalar, Eigen::ColMajor, c_int>;

using namespace proxqp;
namespace eigen {
template<typename T>
void
llt_compute( //
  Eigen::LLT<T>& out,
  T const& mat)
{
  out.compute(mat);
}
template<typename T>
void
ldlt_compute( //
  Eigen::LDLT<T>& out,
  T const& mat)
{
  out.compute(mat);
}
LDLT_EXPLICIT_TPL_DECL(2, llt_compute<Mat<f32, colmajor>>);
LDLT_EXPLICIT_TPL_DECL(2, ldlt_compute<Mat<f32, colmajor>>);
LDLT_EXPLICIT_TPL_DECL(2, llt_compute<Mat<f32, rowmajor>>);
LDLT_EXPLICIT_TPL_DECL(2, ldlt_compute<Mat<f32, rowmajor>>);

LDLT_EXPLICIT_TPL_DECL(2, llt_compute<Mat<f64, colmajor>>);
LDLT_EXPLICIT_TPL_DECL(2, ldlt_compute<Mat<f64, colmajor>>);
LDLT_EXPLICIT_TPL_DECL(2, llt_compute<Mat<f64, rowmajor>>);
LDLT_EXPLICIT_TPL_DECL(2, ldlt_compute<Mat<f64, rowmajor>>);
} // namespace eigen
namespace rand {

using proxqp::u32;
using proxqp::u64;

#ifdef _MSC_VER
/* Using the MSCV compiler on Windows causes problems because the type uint128
is not available. Therefore, we use a random number generator from the stdlib
instead of our custom Lehmer random number generator. The necessary lehmer
functions used in in our code are remplaced with calls to the stdlib.*/
std::mt19937 gen(1234);
std::uniform_real_distribution<> uniform_dist(0.0, 1.0);
std::normal_distribution<double> normal_dist;
using u128 = u64;
inline auto
uniform_rand() -> double
{
  double output = double(uniform_dist(gen));
  return output;
}
inline auto
lehmer_global() -> u128&
{
  static u64 output = gen();
  return output;
}

inline void
set_seed(u64 seed)
{
  gen.seed(seed);
}

inline auto
normal_rand() -> double
{
  return normal_dist(gen);
}
#else
using u128 = __uint128_t;

constexpr u128 lehmer64_constant(0xda942042e4dd58b5);
inline auto
lehmer_global() -> u128&
{
  static u128 g_lehmer64_state = lehmer64_constant * lehmer64_constant;
  return g_lehmer64_state;
}

inline auto
lehmer64() -> u64
{ // [0, 2^64)
  lehmer_global() *= lehmer64_constant;
  return u64(lehmer_global() >> u128(64U));
}

inline void
set_seed(u64 seed)
{
  lehmer_global() = u128(seed) + 1;
  lehmer64();
  lehmer64();
}

inline auto
uniform_rand() -> double
{ // [0, 2^53]
  u64 a = lehmer64() / (1U << 11U);
  return double(a) / double(u64(1) << 53U);
}
inline auto
normal_rand() -> double
{
  static const double pi2 = std::atan(static_cast<double>(1)) * 8;

  double u1 = uniform_rand();
  double u2 = uniform_rand();

  double ln = std::log(u1);
  double sqrt = std::sqrt(-2 * ln);

  return sqrt * std::cos(pi2 * u2);
}
#endif

template<typename Scalar>
auto
vector_rand(isize nrows) -> Vec<Scalar>
{
  auto v = Vec<Scalar>(nrows);

  for (isize i = 0; i < nrows; ++i) {
    v(i) = Scalar(rand::normal_rand());
  }

  return v;
}
template<typename Scalar>
auto
matrix_rand(isize nrows, isize ncols) -> Mat<Scalar, colmajor>
{
  auto v = Mat<Scalar, colmajor>(nrows, ncols);

  for (isize i = 0; i < nrows; ++i) {
    for (isize j = 0; j < ncols; ++j) {
      v(i, j) = Scalar(rand::normal_rand());
    }
  }

  return v;
}

namespace detail {
template<typename Scalar>
auto
orthonormal_rand_impl(isize n) -> Mat<Scalar, colmajor>
{
  auto mat = rand::matrix_rand<Scalar>(n, n);
  auto qr = mat.householderQr();
  Mat<Scalar, colmajor> q = qr.householderQ();
  return q;
}
using Input = std::pair<u128, isize>;
} // namespace detail

template<typename Scalar>
auto
orthonormal_rand(isize n) -> Mat<Scalar, colmajor> const&
{

  static auto cache = std::map<detail::Input, Mat<Scalar, colmajor>>{};
  auto input = detail::Input{ lehmer_global(), n };
  auto it = cache.find(input);
  if (it == cache.end()) {
    auto res = cache.insert({
      input,
      detail::orthonormal_rand_impl<Scalar>(n),
    });
    it = res.first;
  }
  return (*it).second;
}

template<typename Scalar>
auto
positive_definite_rand(isize n, Scalar cond) -> Mat<Scalar, colmajor>
{
  auto const& q = rand::orthonormal_rand<Scalar>(n);
  auto d = Vec<Scalar>(n);

  {
    using std::exp;
    using std::log;
    Scalar diff = log(cond);
    for (isize i = 0; i < n; ++i) {
      d(i) = exp(Scalar(i) / Scalar(n) * diff);
    }
  }

  return q * d.asDiagonal() * q.transpose();
}

template<typename Scalar>
auto
sparse_positive_definite_rand(isize n, Scalar cond, Scalar p)
  -> SparseMat<Scalar>
{
  auto H = SparseMat<Scalar>(n, n);

  for (isize i = 0; i < n; ++i) {
    auto random = Scalar(rand::normal_rand());
    H.insert(i, i) = random;
  }

  for (isize i = 0; i < n; ++i) {
    for (isize j = i + 1; j < n; ++j) {
      auto urandom = rand::uniform_rand();
      if (urandom < p / 2) {
        auto random = Scalar(rand::normal_rand());
        H.insert(i, j) = random;
      }
    }
  }

  Mat<Scalar, colmajor> H_dense = H.toDense();
  Vec<Scalar> eigh =
    H_dense.template selfadjointView<Eigen::Upper>().eigenvalues();

  Scalar min = eigh.minCoeff();
  Scalar max = eigh.maxCoeff();

  // new_min = min + rho
  // new_max = max + rho
  //
  // (max + rho)/(min + rho) = cond
  // 1 + (max - min) / (min + rho) = cond
  // (max - min) / (min + rho) = cond - 1
  // min + rho = (max - min) / (cond - 1)
  // rho = (max - min)/(cond - 1) - min
  Scalar rho = (max - min) / (cond - 1) - min;
  if (max == min) {
    rho += 1;
  }

  for (isize i = 0; i < n; ++i) {
    H.coeffRef(i, i) += rho;
  }

  H.makeCompressed();
  return H;
}

template<typename Scalar>
auto
sparse_positive_definite_rand_compressed(isize n, Scalar rho, Scalar p)
  -> SparseMat<Scalar>
{
  auto H = SparseMat<Scalar>(n, n);

  H.setZero();

  for (isize i = 0; i < n; ++i) {
    for (isize j = i + 1; j < n; ++j) {
      auto urandom = rand::uniform_rand();
      if (urandom < p) {
        auto random = Scalar(rand::normal_rand());
        H.insert(i, j) = random;
      }
    }
  }
  Mat<Scalar, colmajor> H_dense = H.toDense();
  Vec<Scalar> eigh =
    H_dense.template selfadjointView<Eigen::Upper>().eigenvalues();
  Scalar min = eigh.minCoeff();
  for (isize i = 0; i < n; ++i) {
    H.coeffRef(i, i) += (rho + abs(min));
  }

  H.makeCompressed();
  return H;
}

template<typename Scalar>
auto
sparse_positive_definite_rand_not_compressed(isize n, Scalar rho, Scalar p)
  -> Mat<Scalar, colmajor>
{
  auto H = Mat<Scalar, colmajor>(n, n);
  H.setZero();

  for (isize i = 0; i < n; ++i) {
    for (isize j = 0; j < n; ++j) {
      auto urandom = rand::uniform_rand();
      if (urandom < p / 2) {
        auto random = Scalar(rand::normal_rand());
        H(i, j) = random;
      }
    }
  }

  H = ((H + H.transpose()) * 0.5)
        .eval(); // safe no aliasing :
                 // https://eigen.tuxfamily.org/dox/group__TopicAliasing.html
  // H.array() /= 2.;
  Vec<Scalar> eigh = H.template selfadjointView<Eigen::Upper>().eigenvalues();
  Scalar min = eigh.minCoeff();
  H.diagonal().array() += (rho + abs(min));

  return H;
}

template<typename Scalar>
auto
sparse_matrix_rand(isize nrows, isize ncols, Scalar p) -> SparseMat<Scalar>
{
  auto A = SparseMat<Scalar>(nrows, ncols);

  for (isize i = 0; i < nrows; ++i) {
    for (isize j = 0; j < ncols; ++j) {
      if (rand::uniform_rand() < p) {
        A.insert(i, j) = Scalar(rand::normal_rand());
      }
    }
  }
  A.makeCompressed();
  return A;
}

template<typename Scalar>
auto
sparse_matrix_rand_not_compressed(isize nrows, isize ncols, Scalar p)
  -> Mat<Scalar, colmajor>
{
  auto A = Mat<Scalar, colmajor>(nrows, ncols);
  A.setZero();
  for (isize i = 0; i < nrows; ++i) {
    for (isize j = 0; j < ncols; ++j) {
      if (rand::uniform_rand() < p) {
        A(i, j) = Scalar(rand::normal_rand());
      }
    }
  }
  return A;
}

} // namespace rand
using proxqp::usize;

namespace osqp {
auto
to_sparse(Mat<c_float, colmajor> const& mat) -> SparseMat<c_float>;
auto
to_sparse_sym(Mat<c_float, colmajor> const& mat) -> SparseMat<c_float>;
} // namespace osqp

template<typename T>
auto
matmul_impl( //
  Mat<T, proxqp::colmajor> const& lhs,
  Mat<T, proxqp::colmajor> const& rhs) -> Mat<T, proxqp::colmajor>
{
  return lhs.operator*(rhs);
}
template<typename To, typename From>
auto
mat_cast(Mat<From, proxqp::colmajor> const& from) -> Mat<To, proxqp::colmajor>
{
  return from.template cast<To>();
}
LDLT_EXPLICIT_TPL_DECL(2, matmul_impl<long double>);
LDLT_EXPLICIT_TPL_DECL(1, mat_cast<proxqp::f64, long double>);
LDLT_EXPLICIT_TPL_DECL(1, mat_cast<proxqp::f32, long double>);

template<typename MatLhs, typename MatRhs, typename T = typename MatLhs::Scalar>
auto
matmul(MatLhs const& a, MatRhs const& b) -> Mat<T, proxqp::colmajor>
{
  using Upscaled = typename std::
    conditional<std::is_floating_point<T>::value, long double, T>::type;

  return mat_cast<T, Upscaled>(matmul_impl<Upscaled>(
    Mat<T, proxqp::colmajor>(a).template cast<Upscaled>(),
    Mat<T, proxqp::colmajor>(b).template cast<Upscaled>()));
}

template<typename MatLhs,
         typename MatMid,
         typename MatRhs,
         typename T = typename MatLhs::Scalar>
auto
matmul3(MatLhs const& a, MatMid const& b, MatRhs const& c)
  -> Mat<T, proxqp::colmajor>
{
  return matmul(matmul(a, b), c);
}

VEG_TAG(from_data, FromData);

struct EigenNoAlloc
{
  EigenNoAlloc(EigenNoAlloc&&) = delete;
  EigenNoAlloc(EigenNoAlloc const&) = delete;
  auto operator=(EigenNoAlloc&&) -> EigenNoAlloc& = delete;
  auto operator=(EigenNoAlloc const&) -> EigenNoAlloc& = delete;

#if defined(EIGEN_RUNTIME_NO_MALLOC)
  EigenNoAlloc() noexcept { Eigen::internal::set_is_malloc_allowed(false); }
  ~EigenNoAlloc() noexcept { Eigen::internal::set_is_malloc_allowed(true); }
#else
  EigenNoAlloc() = default;
#endif
};

template<typename Scalar>
proxsuite::proxqp::dense::Model<Scalar>
dense_unconstrained_qp(proxqp::isize dim,
                       Scalar sparsity_factor,
                       Scalar strong_convexity_factor = Scalar(1e-2))
{

  Mat<Scalar, colmajor> H =
    rand::sparse_positive_definite_rand_not_compressed<Scalar>(
      dim, strong_convexity_factor, sparsity_factor);
  Vec<Scalar> g = rand::vector_rand<Scalar>(dim);
  Mat<Scalar, colmajor> A =
    rand::sparse_matrix_rand_not_compressed<Scalar>(0, dim, sparsity_factor);
  Vec<Scalar> b = rand::vector_rand<Scalar>(0);
  Mat<Scalar, colmajor> C =
    rand::sparse_matrix_rand_not_compressed<Scalar>(0, dim, sparsity_factor);
  Vec<Scalar> u = rand::vector_rand<Scalar>(0);
  Vec<Scalar> l = rand::vector_rand<Scalar>(0);
  proxsuite::proxqp::dense::Model<Scalar> model(dim, 0, 0);
  model.H = H;
  model.g = g;
  return model;
}

template<typename Scalar>
proxsuite::proxqp::dense::Model<Scalar>
dense_strongly_convex_qp(proxqp::isize dim,
                         proxqp::isize n_eq,
                         proxqp::isize n_in,
                         Scalar sparsity_factor,
                         Scalar strong_convexity_factor = Scalar(1e-2))
{

  Mat<Scalar, colmajor> H =
    rand::sparse_positive_definite_rand_not_compressed<Scalar>(
      dim, strong_convexity_factor, sparsity_factor);
  Vec<Scalar> g = rand::vector_rand<Scalar>(dim);
  Mat<Scalar, colmajor> A =
    rand::sparse_matrix_rand_not_compressed<Scalar>(n_eq, dim, sparsity_factor);
  Mat<Scalar, colmajor> C =
    rand::sparse_matrix_rand_not_compressed<Scalar>(n_in, dim, sparsity_factor);

  Vec<Scalar> x_sol = rand::vector_rand<Scalar>(dim);
  auto delta = Vec<Scalar>(n_in);

  for (proxqp::isize i = 0; i < n_in; ++i) {
    delta(i) = rand::uniform_rand();
  }

  Vec<Scalar> u = C * x_sol + delta;
  Vec<Scalar> b = A * x_sol;
  Vec<Scalar> l(n_in);
  l.setZero();
  l.array() -= 1.e20;

  proxsuite::proxqp::dense::Model<Scalar> model(dim, n_eq, n_in);
  model.H = H;
  model.g = g;
  model.A = A;
  model.b = b;
  model.C = C;
  model.u = u;
  model.l = l;
  return model;
}

template<typename Scalar>
proxsuite::proxqp::dense::Model<Scalar>
dense_not_strongly_convex_qp(proxqp::isize dim,
                             proxqp::isize n_eq,
                             proxqp::isize n_in,
                             Scalar sparsity_factor)
{

  Mat<Scalar, colmajor> H =
    rand::sparse_positive_definite_rand_not_compressed<Scalar>(
      dim, Scalar(0), sparsity_factor);
  Mat<Scalar, colmajor> A =
    rand::sparse_matrix_rand_not_compressed<Scalar>(n_eq, dim, sparsity_factor);
  Mat<Scalar, colmajor> C =
    rand::sparse_matrix_rand_not_compressed<Scalar>(n_in, dim, sparsity_factor);

  Vec<Scalar> x_sol = rand::vector_rand<Scalar>(dim);
  Vec<Scalar> y_sol = rand::vector_rand<Scalar>(n_eq);
  Vec<Scalar> z_sol = rand::vector_rand<Scalar>(n_in);
  auto delta = Vec<Scalar>(n_in);

  for (proxqp::isize i = 0; i < n_in; ++i) {
    delta(i) = rand::uniform_rand();
  }
  auto Cx = C * x_sol;
  Vec<Scalar> u = Cx + delta;
  Vec<Scalar> b = A * x_sol;
  Vec<Scalar> l = Cx - delta;
  Vec<Scalar> g = -(H * x_sol + C.transpose() * z_sol + A.transpose() * y_sol);

  proxsuite::proxqp::dense::Model<Scalar> model(dim, n_eq, n_in);
  model.H = H;
  model.g = g;
  model.A = A;
  model.b = b;
  model.C = C;
  model.u = u;
  model.l = l;
  return model;
}

template<typename Scalar>
proxsuite::proxqp::dense::Model<Scalar>
dense_degenerate_qp(proxqp::isize dim,
                    proxqp::isize n_eq,
                    proxqp::isize n_in,
                    Scalar sparsity_factor,
                    Scalar strong_convexity_factor = Scalar(1e-2))
{

  Mat<Scalar, colmajor> H =
    rand::sparse_positive_definite_rand_not_compressed<Scalar>(
      dim, strong_convexity_factor, sparsity_factor);
  Vec<Scalar> g = rand::vector_rand<Scalar>(dim);
  Mat<Scalar, colmajor> A =
    rand::sparse_matrix_rand_not_compressed<Scalar>(n_eq, dim, sparsity_factor);
  Mat<Scalar, colmajor> C = Mat<Scalar, proxqp::colmajor>(2 * n_in, dim);

  Vec<Scalar> x_sol = rand::vector_rand<Scalar>(dim);
  auto delta = Vec<Scalar>(2 * n_in);

  for (proxqp::isize i = 0; i < 2 * n_in; ++i) {
    delta(i) = rand::uniform_rand();
  }
  Vec<Scalar> b = A * x_sol;

  auto C_ =
    rand::sparse_matrix_rand_not_compressed<Scalar>(n_in, dim, sparsity_factor);
  C.setZero();
  C.block(0, 0, n_in, dim) = C_;
  C.block(n_in, 0, n_in, dim) = C_;
  Vec<Scalar> u = C * x_sol + delta;
  Vec<Scalar> l(2 * n_in);
  l.setZero();
  l.array() -= 1.e20;

  proxsuite::proxqp::dense::Model<Scalar> model(dim, n_eq, n_in);
  model.H = H;
  model.g = g;
  model.A = A;
  model.b = b;
  model.C = C;
  model.u = u;
  model.l = l;
  return model;
}

template<typename Scalar>
proxsuite::proxqp::dense::Model<Scalar>
dense_box_constrained_qp(proxqp::isize dim,
                         proxqp::isize n_eq,
                         proxqp::isize n_in,
                         Scalar sparsity_factor,
                         Scalar strong_convexity_factor = Scalar(1e-2))
{

  Mat<Scalar, colmajor> H =
    rand::sparse_positive_definite_rand_not_compressed<Scalar>(
      dim, strong_convexity_factor, sparsity_factor);
  Vec<Scalar> g = rand::vector_rand<Scalar>(dim);
  Mat<Scalar, colmajor> A =
    rand::sparse_matrix_rand_not_compressed<Scalar>(n_eq, dim, sparsity_factor);
  Mat<Scalar, colmajor> C = Mat<Scalar, proxqp::colmajor>(n_in, dim);

  Vec<Scalar> x_sol = rand::vector_rand<Scalar>(dim);
  auto delta = Vec<Scalar>(n_in);

  for (proxqp::isize i = 0; i < n_in; ++i) {
    delta(i) = rand::uniform_rand();
  }
  Vec<Scalar> b = A * x_sol;
  C.setZero();
  C.diagonal().array() += 1;
  Vec<Scalar> u = x_sol + delta;
  Vec<Scalar> l = x_sol - delta;
  proxsuite::proxqp::dense::Model<Scalar> model(dim, n_eq, n_in);
  model.H = H;
  model.g = g;
  model.A = A;
  model.b = b;
  model.C = C;
  model.u = u;
  model.l = l;
  return model;
}

template<typename Scalar>
proxsuite::proxqp::sparse::SparseModel<Scalar>
sparse_strongly_convex_qp(proxqp::isize dim,
                          proxqp::isize n_eq,
                          proxqp::isize n_in,
                          Scalar sparsity_factor,
                          Scalar strong_convexity_factor = Scalar(1e-2))
{

  SparseMat<Scalar> H = rand::sparse_positive_definite_rand_compressed<Scalar>(
    dim, strong_convexity_factor, sparsity_factor);
  Vec<Scalar> g = rand::vector_rand<Scalar>(dim);
  SparseMat<Scalar> A =
    rand::sparse_matrix_rand<Scalar>(n_eq, dim, sparsity_factor);
  SparseMat<Scalar> C =
    rand::sparse_matrix_rand<Scalar>(n_in, dim, sparsity_factor);

  Vec<Scalar> x_sol = rand::vector_rand<Scalar>(dim);
  auto delta = Vec<Scalar>(n_in);

  for (proxqp::isize i = 0; i < n_in; ++i) {
    delta(i) = rand::uniform_rand();
  }

  Vec<Scalar> u = C * x_sol + delta;
  Vec<Scalar> b = A * x_sol;
  Vec<Scalar> l(n_in);
  l.setZero();
  l.array() -= 1.e20;

  proxsuite::proxqp::sparse::SparseModel<Scalar> res{ H, g, A, b, C, u, l };
  return res;
}

} // namespace utils
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_UTILS_RANDOM_QP_PROBLEMS_HPP   \
        */
