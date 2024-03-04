/** \file */
//
// Copyright (c) 2022 INRIA
//
#ifndef PROXSUITE_LINALG_DENSE_LDLT_MODIFY_HPP
#define PROXSUITE_LINALG_DENSE_LDLT_MODIFY_HPP

#include "proxsuite/linalg/dense/core.hpp"
#include "proxsuite/linalg/dense/update.hpp"
#include "proxsuite/linalg/dense/factorize.hpp"
#include <algorithm>
#include <proxsuite/linalg/veg/memory/dynamic_stack.hpp>

namespace proxsuite {
namespace linalg {
namespace dense {
namespace _detail {

template<typename Mat>
void
delete_rows_and_cols_triangular_impl(Mat mat, isize const* indices, isize r)
{
  isize n = mat.rows();

  for (isize chunk_j = 0; chunk_j < r + 1; ++chunk_j) {
    isize j_start = chunk_j == 0 ? 0 : indices[chunk_j - 1] + 1;
    isize j_finish = chunk_j == r ? n : indices[chunk_j];

    for (isize j = j_start; j < j_finish; ++j) {
      for (isize chunk_i = chunk_j; chunk_i < r + 1; ++chunk_i) {
        isize i_start = chunk_i == chunk_j ? j : indices[chunk_i - 1] + 1;
        isize i_finish = chunk_i == r ? n : indices[chunk_i];

        if (chunk_i != 0 || chunk_j != 0) {
          std::move( //
            util::matrix_elem_addr(mat, i_start, j),
            util::matrix_elem_addr(mat, i_finish, j),
            util::matrix_elem_addr( //
              mat,
              (i_start - chunk_i),
              (j - chunk_j)));
        }
      }
    }
  }
}
// indices: rows and columns to delete, in strictly increasing order
//          must have at least one element (excluding the end)
// r: count of rows to delete
template<typename Mat>
void
delete_rows_and_cols_triangular(Mat&& mat, isize const* indices, isize r)
{
  _detail::delete_rows_and_cols_triangular_impl(
    util::to_view_dyn(mat), indices, r);
}

struct IndicesR
{
  isize current_col;
  isize current_r;
  isize r;
  isize const* indices;
  VEG_INLINE auto operator()() noexcept -> isize
  {
    if (current_r == r) {
      return current_r;
    }

    while (current_col == indices[current_r] - current_r) {
      ++current_r;
      if (current_r == r) {
        return current_r;
      }
    }
    ++current_col;
    return current_r;
  }
};
template<typename Mat>
void
ldlt_delete_rows_and_cols_impl( //
  Mat ld,
  isize* indices,
  isize r,
  proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  std::sort(indices, indices + r);

  using T = typename Mat::Scalar;

  isize n = ld.rows();
  isize first = indices[0];

  auto w_stride = _detail::adjusted_stride<T>(n - first - r);

  proxsuite::linalg::veg::Tag<T> tag;

  auto _w = stack.make_new(tag, r * w_stride, _detail::align<T>());
  auto _alpha = stack.make_new_for_overwrite(tag, r);

  auto pw = _w.ptr_mut();
  auto palpha = _alpha.ptr_mut();

  for (isize k = 0; k < r; ++k) {
    isize j = indices[k];
    palpha[k] = ld(j, j);
    auto pwk = pw + k * w_stride;

    for (isize chunk_i = k + 1; chunk_i < r + 1; ++chunk_i) {
      isize i_start = indices[chunk_i - 1] + 1;
      isize i_finish = chunk_i == r ? n : indices[chunk_i];

      std::move(util::matrix_elem_addr(ld, i_start, j),
                util::matrix_elem_addr(ld, i_finish, j),
                pwk + i_start - chunk_i - first);
    }
  }
  _detail::delete_rows_and_cols_triangular(ld, indices, r);

  _detail::rank_r_update_clobber_w_impl(
    util::submatrix(ld, first, first, n - first - r, n - first - r),
    pw,
    w_stride,
    palpha,
    IndicesR{ first, 0, r, indices });
}

template<typename Mat, typename A_1>
void
ldlt_insert_rows_and_cols_impl(
  Mat ld,
  isize pos,
  A_1 a_1,
  proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  using T = typename Mat::Scalar;

  proxsuite::linalg::veg::Tag<T> tag;

  isize const new_n = ld.rows();
  isize const r = a_1.cols();
  isize const old_n = new_n - r;

  isize current_col = old_n;
  while (true) {
    if (current_col == pos) {
      break;
    }
    --current_col;

    T* src_col_ptr = util::matrix_elem_addr(ld, 0, current_col);
    T* dest_col_ptr = util::matrix_elem_addr(ld, 0, current_col + r);

    std::move_backward( //
      src_col_ptr + pos,
      src_col_ptr + old_n,
      dest_col_ptr + new_n);

    std::move_backward( //
      src_col_ptr,
      src_col_ptr + pos,
      dest_col_ptr + pos);
  }

  while (true) {
    if (current_col == 0) {
      break;
    }
    --current_col;

    T* src_col_ptr = util::matrix_elem_addr(ld, 0, current_col);
    T* dest_col_ptr = src_col_ptr;

    std::move_backward( //
      src_col_ptr + pos,
      src_col_ptr + old_n,
      dest_col_ptr + new_n);
  }

  auto rem = new_n - pos - r;

  auto ld00 = util::submatrix(ld, 0, 0, pos, pos);
  auto l10 = util::submatrix(ld, pos, 0, r, pos);
  auto l20 = util::submatrix(ld, pos + r, 0, rem, pos);

  auto ld11 = util::submatrix(ld, pos, pos, r, r);
  auto l21 = util::submatrix(ld, pos + r, pos, rem, r);

  auto ld22 = util::submatrix(ld, pos + r, pos + r, rem, rem);

  auto d0 = util::diagonal(ld00).asDiagonal();

  auto a01 = util::subrows(a_1, 0, pos);
  auto a11 = util::subrows(a_1, pos, r);
  auto a21 = util::subrows(a_1, pos + r, rem);

  if (l10.cols() > 0) {
    l10 = util::trans(a01);
    util::trans(ld00) //
      .template triangularView<Eigen::UnitUpper>()
      .template solveInPlace<Eigen::OnTheRight>(l10);

    l10 = l10 * d0.inverse();
  }

  {
    isize tmp_stride = _detail::adjusted_stride<T>(pos);
    auto _tmp = stack.make_new_for_overwrite( //
      tag,
      tmp_stride,
      _detail::align<T>());
    auto d0xl10T = Eigen::Map<Eigen::Matrix< //
                                T,
                                Eigen::Dynamic,
                                A_1::ColsAtCompileTime>,
                              Eigen::Unaligned,
                              Eigen::OuterStride<Eigen::Dynamic>>{
      _tmp.ptr_mut(),
      pos,
      r,
      tmp_stride,
    };

    ld11.template triangularView<Eigen::Lower>() =
      a11.template triangularView<Eigen::Lower>();

    if (l10.cols() > 0) {
      d0xl10T = d0 * util::trans(l10);
      ld11.template triangularView<Eigen::Lower>() -= l10 * d0xl10T;
    }

    l21 = a21;
    util::noalias_mul_add(l21, l20, d0xl10T, T(-1));
  }

  proxsuite::linalg::dense::factorize(ld11, stack);
  util::trans(ld11) //
    .template triangularView<Eigen::UnitUpper>()
    .template solveInPlace<Eigen::OnTheRight>(l21);

  auto d1 = util::diagonal(ld11).asDiagonal();
  l21 = l21 * d1.inverse();

  auto w_stride = _detail::adjusted_stride<T>(rem);
  auto _w = stack.make_new(tag, r * w_stride, _detail::align<T>());
  auto _alpha = stack.make_new_for_overwrite(tag, r);

  auto pw = _w.ptr_mut();
  auto palpha = _alpha.ptr_mut();

  for (isize k = 0; k < r; ++k) {
    palpha[k] = -ld11(k, k);
    T const* src_ptr = util::matrix_elem_addr(l21, 0, k);
    std::copy(src_ptr, src_ptr + rem, pw + k * w_stride);
  }

  _detail::rank_r_update_clobber_w_impl( //
    ld22,
    pw,
    w_stride,
    palpha,
    _detail::ConstantR{ r });
}
} // namespace _detail

template<typename T>
auto
ldlt_delete_rows_and_cols_req(proxsuite::linalg::veg::Tag<T> /*tag*/,
                              isize n,
                              isize r) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{

  auto w_req = proxsuite::linalg::veg::dynstack::StackReq{
    _detail::adjusted_stride<T>(n - r) * r * isize{ sizeof(T) },
    _detail::align<T>(),
  };
  auto alpha_req = proxsuite::linalg::veg::dynstack::StackReq{
    r * isize{ sizeof(T) },
    alignof(T),
  };
  return w_req & alpha_req;
}

template<typename Mat>
void
ldlt_delete_rows_and_cols_sort_indices( //
  Mat&& ld,
  isize* indices,
  isize r,
  proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  _detail::ldlt_delete_rows_and_cols_impl(
    util::to_view_dyn(ld), indices, r, stack);
}

template<typename T>
auto
ldlt_insert_rows_and_cols_req(proxsuite::linalg::veg::Tag<T> tag,
                              isize n,
                              isize r) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  auto factorize_req = proxsuite::linalg::dense::factorize_req(tag, r);

  auto w_req = proxsuite::linalg::veg::dynstack::StackReq{
    _detail::adjusted_stride<T>(n) * r * isize{ sizeof(T) },
    _detail::align<T>(),
  };
  auto alpha_req = proxsuite::linalg::veg::dynstack::StackReq{
    r * isize{ sizeof(T) },
    alignof(T),
  };

  return (w_req & alpha_req) | factorize_req;
}

template<typename Mat, typename A_1>
void
ldlt_insert_rows_and_cols(Mat&& ld,
                          isize pos,
                          A_1 const& a_1,
                          proxsuite::linalg::veg::dynstack::DynStackMut stack)
{
  _detail::ldlt_insert_rows_and_cols_impl(
    util::to_view_dyn(ld), pos, util::to_view_dyn_rows(a_1), stack);
}
} // namespace dense
} // namespace linalg
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_LINALG_DENSE_LDLT_MODIFY_HPP */
