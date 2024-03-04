/** \file */
//
// Copyright (c) 2022 INRIA
//
#ifndef PROXSUITE_LINALG_SPARSE_LDLT_UPDATE_HPP
#define PROXSUITE_LINALG_SPARSE_LDLT_UPDATE_HPP

#include "proxsuite/linalg/sparse/core.hpp"
#include <proxsuite/linalg/veg/tuple.hpp>
#include <algorithm>

namespace proxsuite {
namespace linalg {
namespace sparse {

/*
calcule mémoire nécessaire pour la fonction merge_second_col_into_first
*/
template<typename I>
auto
merge_second_col_into_first_req(proxsuite::linalg::veg::Tag<I> /*tag*/,
                                isize second_size) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return {
    second_size * isize{ sizeof(I) },
    alignof(I),
  };
}

template<typename T, typename I>
auto
merge_second_col_into_first( //
  I* difference,
  T* first_values,
  I* first_ptr,
  PROXSUITE_MAYBE_UNUSED isize first_full_len,
  isize first_initial_len,
  Slice<I> second,
  proxsuite::linalg::veg::DoNotDeduce<I> ignore_threshold_inclusive,
  bool move_values,
  DynStackMut stack) noexcept(false)
  -> proxsuite::linalg::veg::Tuple<SliceMut<T>, SliceMut<I>, SliceMut<I>>
{
  VEG_CHECK_CONCEPT(trivially_copyable<I>);
  VEG_CHECK_CONCEPT(trivially_copyable<T>);

  if (second.len() == 0) {
    return {
      proxsuite::linalg::veg::tuplify,
      { unsafe, from_raw_parts, first_values, first_initial_len },
      { unsafe, from_raw_parts, first_ptr, first_initial_len },
      { unsafe, from_raw_parts, difference, 0 },
    };
  }

  I const* second_ptr = second.ptr();
  usize second_len = usize(second.len());

  usize index_second = 0;

  for (; index_second < second_len; ++index_second) {
    if (second_ptr[index_second] > ignore_threshold_inclusive) {
      break;
    }
  }
  auto ufirst_initial_len = usize(first_initial_len);

  second_ptr += index_second;
  second_len -= index_second;
  index_second = 0;

  proxsuite::linalg::veg::Tag<I> tag{};

  auto _ins_pos = stack.make_new_for_overwrite(tag, isize(second_len));

  I* insert_pos_ptr = _ins_pos.ptr_mut();
  usize insert_count = 0;

  for (usize index_first = 0; index_first < ufirst_initial_len; ++index_first) {
    I current_first = first_ptr[index_first];
    while (true) {
      if (!(index_second < second_len)) {
        break;
      }

      I current_second = second_ptr[index_second];
      if (!(current_second < current_first)) {
        break;
      }

      insert_pos_ptr[insert_count] = I(index_first);
      difference[insert_count] = current_second;
      ++insert_count;
      ++index_second;
    }

    if (index_second == second_len) {
      break;
    }
    if (second_ptr[index_second] == current_first) {
      ++index_second;
    }
  }

  usize remaining_insert_count = insert_count;
  usize first_new_len =
    ufirst_initial_len + insert_count + (second_len - index_second);
  VEG_ASSERT(usize(first_full_len) >= first_new_len);

  usize append_count = second_len - index_second;
  std::memmove( //
    difference + insert_count,
    second_ptr + index_second,
    append_count * sizeof(I));
  std::memmove( //
    first_ptr + (ufirst_initial_len + insert_count),
    second_ptr + index_second,
    append_count * sizeof(I));
  if (move_values) {
    for (usize i = 0; i < append_count; ++i) {
      first_values[i + ufirst_initial_len + insert_count] = 0;
    }
  }

  while (remaining_insert_count != 0) {

    usize old_insert_pos = usize(insert_pos_ptr[remaining_insert_count - 1]);
    usize range_size =
      (remaining_insert_count == insert_count)
        ? ufirst_initial_len - old_insert_pos
        : usize(insert_pos_ptr[remaining_insert_count]) - old_insert_pos;

    usize old_pos = old_insert_pos;
    usize new_pos = old_pos + remaining_insert_count;

    std::memmove( //
      first_ptr + new_pos,
      first_ptr + old_pos,
      range_size * sizeof(I));
    if (move_values) {
      std::memmove( //
        first_values + new_pos,
        first_values + old_pos,
        range_size * sizeof(T));
      first_values[new_pos - 1] = 0;
    }

    first_ptr[new_pos - 1] = difference[remaining_insert_count - 1];
    --remaining_insert_count;
  }

  return {
    proxsuite::linalg::veg::tuplify,
    { unsafe, from_raw_parts, first_values, isize(first_new_len) },
    { unsafe, from_raw_parts, first_ptr, isize(first_new_len) },
    { unsafe, from_raw_parts, difference, isize(insert_count + append_count) },
  };
}

/*!
 * Computes the memory requirements for rank one update.
 *
 * @param n dimension of matrix
 * @param id_perm whether the permutation is implicitly the identity or not
 * @param col_nnz number of nnz elts in the update vector
 */
template<typename T, typename I>
auto
rank1_update_req( //
  proxsuite::linalg::veg::Tag<T> /*tag*/,
  proxsuite::linalg::veg::Tag<I> /*tag*/,
  isize n,
  bool id_perm,
  isize col_nnz) noexcept -> proxsuite::linalg::veg::dynstack::StackReq
{
  using proxsuite::linalg::veg::dynstack::StackReq;
  StackReq permuted_indices = { id_perm ? 0 : (col_nnz * isize{ sizeof(I) }),
                                isize{ alignof(I) } };
  StackReq difference = { n * isize{ sizeof(I) }, isize{ alignof(I) } };
  difference = difference & difference;

  StackReq merge = sparse::merge_second_col_into_first_req(
    proxsuite::linalg::veg::Tag<I>{}, n);

  StackReq numerical_workspace = { n * isize{ sizeof(T) },
                                   isize{ alignof(T) } };

  return permuted_indices & ((difference & merge) | numerical_workspace);
}

/*!
 * Performs a rank one update in place. Given ldlt factor l, and d, of a matrix
 * a, this computes the ldlt factors of a + alpha  w w.T It returns a view on
 * the updated factors.
 *
 * @param ld : ldlt factors of a (lower triangular with d on the diagonal)
 * @param etree pointer to the elimination tree
 * @param perm_inv pointer to inverse permutation (for ex AMD). If this is null,
 * the permutation is assumed to be the identity.
 * @param w is the update vector
 * @param alpha is the update coefficient
 * @param stack is the memory stack
 */
template<typename T, typename I>
auto
rank1_update(MatMut<T, I> ld,
             I* etree,
             I const* perm_inv,
             VecRef<T, I> w,
             proxsuite::linalg::veg::DoNotDeduce<T> alpha,
             DynStackMut stack) noexcept(false) -> MatMut<T, I>
{
  VEG_ASSERT(!ld.is_compressed());

  if (w.nnz() == 0) {
    return ld;
  }

  proxsuite::linalg::veg::Tag<I> tag;
  usize n = usize(ld.ncols());
  bool id_perm = perm_inv == nullptr;

  auto _w_permuted_indices =
    stack.make_new_for_overwrite(tag, id_perm ? isize(0) : w.nnz());

  auto w_permuted_indices =
    id_perm ? w.row_indices() : _w_permuted_indices.ptr();
  if (!id_perm) {
    I* pw_permuted_indices = _w_permuted_indices.ptr_mut();
    for (usize k = 0; k < usize(w.nnz()); ++k) {
      usize i = util::zero_extend(w.row_indices()[k]);
      pw_permuted_indices[k] = perm_inv[i];
    }
    std::sort(pw_permuted_indices, pw_permuted_indices + w.nnz());
  }

  auto sx = util::sign_extend;
  auto zx = util::zero_extend;
  // symbolic update
  {
    usize current_col = zx(w_permuted_indices[0]);

    auto _difference =
      stack.make_new_for_overwrite(tag, isize(n - current_col));
    auto _difference_backup =
      stack.make_new_for_overwrite(tag, isize(n - current_col));

    auto merge_col = w_permuted_indices;
    isize merge_col_len = w.nnz();
    I* difference = _difference.ptr_mut();

    while (true) {
      usize old_parent = sx(etree[isize(current_col)]);

      usize current_ptr_idx = zx(ld.col_ptrs()[isize(current_col)]);
      usize next_ptr_idx = zx(ld.col_ptrs()[isize(current_col) + 1]);

      VEG_BIND(auto,
               (_, new_current_col, computed_difference),
               sparse::merge_second_col_into_first(
                 difference,
                 ld.values_mut() + (current_ptr_idx + 1),
                 ld.row_indices_mut() + (current_ptr_idx + 1),
                 isize(next_ptr_idx - current_ptr_idx),
                 isize(zx(ld.nnz_per_col()[isize(current_col)])) - 1,
                 proxsuite::linalg::veg::Slice<I>{
                   unsafe, from_raw_parts, merge_col, merge_col_len },
                 I(current_col),
                 true,
                 stack));

      (void)_;
      ld._set_nnz(ld.nnz() + new_current_col.len() + 1 -
                  isize(ld.nnz_per_col()[isize(current_col)]));
      ld.nnz_per_col_mut()[isize(current_col)] = I(new_current_col.len() + 1);

      usize new_parent =
        (new_current_col.len() == 0) ? usize(-1) : sx(new_current_col[0]);

      if (new_parent == usize(-1)) {
        break;
      }

      if (new_parent == old_parent) {
        merge_col = computed_difference.ptr();
        merge_col_len = computed_difference.len();
        difference = _difference_backup.ptr_mut();
      } else {
        merge_col = new_current_col.ptr();
        merge_col_len = new_current_col.len();
        difference = _difference.ptr_mut();
        etree[isize(current_col)] = I(new_parent);
      }

      current_col = new_parent;
    }
  }

  // numerical update
  {
    usize first_col = zx(w_permuted_indices[0]);
    auto _work =
      stack.make_new_for_overwrite(proxsuite::linalg::veg::Tag<T>{}, isize(n));
    T* pwork = _work.ptr_mut();

    for (usize col = first_col; col != usize(-1); col = sx(etree[isize(col)])) {
      pwork[col] = 0;
    }
    for (usize p = 0; p < usize(w.nnz()); ++p) {
      pwork[id_perm ? zx(w.row_indices()[isize(p)])
                    : zx(perm_inv[w.row_indices()[isize(p)]])] =
        w.values()[isize(p)];
    }

    I const* pldi = ld.row_indices();
    T* pldx = ld.values_mut();

    for (usize col = first_col; col != usize(-1); col = sx(etree[isize(col)])) {
      auto col_start = ld.col_start(col);
      auto col_end = ld.col_end(col);

      T w0 = pwork[col];
      T old_d = pldx[col_start];
      T new_d = old_d + alpha * w0 * w0;
      T beta = alpha * w0 / new_d;
      alpha = alpha - new_d * beta * beta;

      pldx[col_start] = new_d;
      pwork[col] -= w0;

      for (usize p = col_start + 1; p < col_end; ++p) {
        usize i = util::zero_extend(pldi[p]);

        T tmp = pldx[p];
        pwork[i] = pwork[i] - w0 * tmp;
        pldx[p] = tmp + beta * pwork[i];
      }
    }
  }

  return ld;
}
} // namespace sparse
} // namespace linalg
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_LINALG_SPARSE_LDLT_UPDATE_HPP */
