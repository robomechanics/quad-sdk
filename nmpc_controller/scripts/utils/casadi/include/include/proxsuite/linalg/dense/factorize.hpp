/** \file */
//
// Copyright (c) 2022 INRIA
//
#ifndef PROXSUITE_LINALG_DENSE_LDLT_FACTORIZE_HPP
#define PROXSUITE_LINALG_DENSE_LDLT_FACTORIZE_HPP

#include "proxsuite/linalg/dense/core.hpp"
#include <algorithm>
#include <proxsuite/linalg/veg/memory/dynamic_stack.hpp>

namespace proxsuite {
namespace linalg {
namespace dense {
namespace _detail {

template<typename T>
VEG_NO_INLINE void
compute_permutation_impl(isize* perm_indices,
                         isize* perm_inv_indices,
                         isize n,
                         T const* diagonal_data,
                         isize stride)
{
  for (isize k = 0; k < n; ++k) {
    perm_indices[k] = k;
  }

  {
    std::sort(perm_indices,
              perm_indices + n,
              [diagonal_data, stride](isize i, isize j) noexcept -> bool {
                using std::fabs;
                auto lhs = fabs(diagonal_data[stride * i]);
                auto rhs = fabs(diagonal_data[stride * j]);
                if (lhs == rhs) {
                  return i < j;
                }
                return lhs > rhs;
              });
  }

  for (isize k = 0; k < n; ++k) {
    perm_inv_indices[perm_indices[k]] = k;
  }
}

template<typename Diag>
VEG_NO_INLINE void
compute_permutation(isize* perm_indices,
                    isize* perm_inv_indices,
                    Diag const& diagonal)
{
  _detail::compute_permutation_impl<typename Diag::Scalar>(
    perm_indices,
    perm_inv_indices,
    diagonal.rows(),
    diagonal.data(),
    diagonal.innerStride());
}

template<typename Mat, typename Work>
void
apply_permutation_tri_lower(Mat&& mat, Work&& work, isize const* perm_indices)
{
  using T = typename proxsuite::linalg::veg::uncvref_t<Mat>::Scalar;

  isize n = mat.rows();
  VEG_ASSERT_ALL_OF( //
    n == mat.rows(),
    n == mat.cols(),
    n == work.rows(),
    n == work.cols());

  auto mat_coeff = [&](isize i, isize j) noexcept -> T& {
    return i >= j ? mat(i, j) : mat(j, i);
  };

  for (isize j = 0; j < n; ++j) {
    for (isize i = j; i < n; ++i) {
      work(i, j) = mat_coeff(perm_indices[i], perm_indices[j]);
    }
  }

  mat.template triangularView<Eigen::Lower>() =
    work.template triangularView<Eigen::Lower>();
}

template<typename Mat>
void
factorize_unblocked_impl(Mat mat,
                         proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  // left looking cholesky
  // https://en.wikipedia.org/wiki/Cholesky_decomposition#LDL_decomposition_2

  using T = typename Mat::Scalar;
  isize n = mat.rows();
  if (n == 0) {
    return;
  }

  auto _work = stack.make_new_for_overwrite( //
    proxsuite::linalg::veg::Tag<T>{},
    n,
    _detail::align<T>());
  auto work_storage =
    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>, Eigen::Unaligned>{
      _work.ptr_mut(),
      n,
      1,
    };

  isize j = 0;
  while (true) {
    /*
     *     L00
     * l = l10  1
     *     L20 l21 L22
     *
     *     D0
     * d =    d1
     *           D2
     *
     * compute d1 and l21
     */

    auto l10 = util::subcols(util::row(mat, j), 0, j);
    auto d0 = util::subrows(mat.diagonal(), 0, j);
    auto work = util::subrows(work_storage, 0, j);

    work = util::trans(l10).cwiseProduct(d0);
    mat(j, j) -= work.dot(l10);

    if (j + 1 == n) {
      break;
    }

    isize rem = n - j - 1;

    auto l20 = util::submatrix(mat, j + 1, 0, rem, j);
    auto l21 = util::subrows(util::col(mat, j), j + 1, rem);

    util::noalias_mul_add(l21, l20, work, T(-1));
    l21 *= 1 / mat(j, j);
    ++j;
  }
}

template<typename Mat>
void
factorize_blocked_impl(Mat mat,
                       isize block_size,
                       proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  // right looking blocked cholesky

  using T = typename Mat::Scalar;
  VEG_ASSERT(mat.rows() == mat.cols());

  isize n = mat.rows();

  if (n == 0) {
    return;
  }

  isize j = 0;
  while (true) {
    isize bs = min2(n - j, block_size);

    auto ld11 = util::submatrix(mat, j, j, bs, bs);
    auto d1 = util::diagonal(ld11);
    _detail::factorize_unblocked_impl(ld11, stack);

    if (j + bs == n) {
      break;
    }
    isize rem = n - j - bs;

    isize work_stride = _detail::adjusted_stride<T>(rem);

    auto _work = stack.make_new_for_overwrite( //
      proxsuite::linalg::veg::Tag<T>{},
      bs * work_stride,
      _detail::align<T>());

    auto work = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>,
                           Eigen::Unaligned,
                           Eigen::OuterStride<Eigen::Dynamic>>{
      _work.ptr_mut(),
      rem,
      bs,
      Eigen::OuterStride<Eigen::Dynamic>{ work_stride },
    };

    auto l21 = util::submatrix(mat, j + bs, j, rem, bs);

    util::trans(ld11)
      .template triangularView<Eigen::UnitUpper>()
      .template solveInPlace<Eigen::OnTheRight>(l21);

    work = l21;
    l21 = l21 * d1.asDiagonal().inverse();

    auto l22 = util::submatrix(mat, j + bs, j + bs, rem, rem);

    l22.template triangularView<Eigen::Lower>() -= l21 * util::trans(work);
    j += bs;
  }
}

using factorize_recursive_threshold =
  proxsuite::linalg::veg::meta::constant<isize, 32>;

template<typename Mat>
void
factorize_recursive_impl(Mat mat,
                         proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  // right looking recursive cholesky

  using T = typename Mat::Scalar;
  VEG_ASSERT(mat.rows() == mat.cols());

  isize n = mat.rows();

  if (n < factorize_recursive_threshold::value) {
    _detail::factorize_unblocked_impl(mat, stack);
  } else {
    /*
     *     L00
     * L = L10 L11
     *
     *     D0
     * D =    D1
     *
     * compute L00 and D0 recursively
     * compute L10 by solving a triangular system
     * compute L11 recursively
     */
    isize bs = (n + 1) / 2;
    isize rem = n - bs;

    auto l00 = util::submatrix(mat, 0, 0, bs, bs);

    auto l10 = util::submatrix(mat, bs, 0, rem, bs);
    auto l11 = util::submatrix(mat, bs, bs, rem, rem);

    _detail::factorize_recursive_impl(l00, stack);
    auto d0 = util::diagonal(l00);

    isize work_stride = _detail::adjusted_stride<T>(rem);

    util::trans(l00)
      .template triangularView<Eigen::UnitUpper>()
      .template solveInPlace<Eigen::OnTheRight>(l10);

    {
      auto _work = stack.make_new_for_overwrite( //
        proxsuite::linalg::veg::Tag<T>{},
        bs * work_stride,
        _detail::align<T>());

      auto work = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>,
                             Eigen::Unaligned,
                             Eigen::OuterStride<Eigen::Dynamic>>{
        _work.ptr_mut(),
        rem,
        bs,
        Eigen::OuterStride<Eigen::Dynamic>{ work_stride },
      };
      work = l10;
      l10 = l10 * d0.asDiagonal().inverse();

      l11.template triangularView<Eigen::Lower>() -= l10 * util::trans(work);
    }

    _detail::factorize_recursive_impl(l11, stack);
  }
}
} // namespace _detail
template<typename T>
auto
factorize_unblocked_req(proxsuite::linalg::veg::Tag<T> /*tag*/,
                        isize n) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return {
    n * isize{ sizeof(T) },
    _detail::align<T>(),
  };
}

template<typename T>
auto
factorize_blocked_req(proxsuite::linalg::veg::Tag<T> tag,
                      isize n,
                      isize block_size) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return proxsuite::linalg::dense::factorize_unblocked_req(tag, block_size) |
         proxsuite::linalg::veg::dynstack::StackReq{
           _detail::adjusted_stride<T>(
             _detail::max2(n - block_size, isize(0))) *
             block_size * isize{ sizeof(T) },
           _detail::align<T>(),
         };
}

template<typename T>
auto
factorize_recursive_req(proxsuite::linalg::veg::Tag<T> tag, isize n) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  auto req0 = proxsuite::linalg::dense::factorize_unblocked_req(
    tag, _detail::min2(n, _detail::factorize_recursive_threshold::value));
  if (n < _detail::factorize_recursive_threshold::value) {
    return req0;
  }
  isize bs = (n + 1) / 2;
  isize rem = n - bs;
  return req0 | proxsuite::linalg::veg::dynstack::StackReq{
    bs * _detail::adjusted_stride<T>(rem) * isize{ sizeof(T) },
    _detail::align<T>(),
  };
}

template<typename Mat>
void
factorize_unblocked(Mat&& mat,
                    proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  _detail::factorize_unblocked_impl(util::to_view_dyn(mat), stack);
}
template<typename Mat>
void
factorize_blocked(Mat&& mat,
                  isize block_size,
                  proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  _detail::factorize_blocked_impl(util::to_view_dyn(mat), block_size, stack);
}
template<typename Mat>
void
factorize_recursive(Mat&& mat,
                    proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  _detail::factorize_recursive_impl(util::to_view_dyn(mat), stack);
}

template<typename T>
auto
factorize_req(proxsuite::linalg::veg::Tag<T> tag, isize n) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return proxsuite::linalg::dense::factorize_blocked_req(tag, n, 128) |
         proxsuite::linalg::dense::factorize_recursive_req(tag, n);
}

template<typename Mat>
void
factorize(Mat&& mat, proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  isize n = mat.rows();
  if (n > 2048) {
    proxsuite::linalg::dense::factorize_blocked(mat, 128, stack);
  } else {
    proxsuite::linalg::dense::factorize_recursive(mat, stack);
  }
}
} // namespace dense
} // namespace linalg
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_LINALG_DENSE_LDLT_FACTORIZE_HPP */
