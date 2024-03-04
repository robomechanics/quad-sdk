/** \file */
//
// Copyright (c) 2022 INRIA
//
#ifndef PROXSUITE_LINALG_SPARSE_LDLT_ROWMOD_HPP
#define PROXSUITE_LINALG_SPARSE_LDLT_ROWMOD_HPP

#include "proxsuite/linalg/sparse/update.hpp"
#include <algorithm>

namespace proxsuite {
namespace linalg {
namespace sparse {

/*!
 * Computes the memory requirements for deleting a row and column for the ldlt
 * factors
 *
 * @param n : dimension of the matrix
 * @param max_nnz : upper bound of non zero counts over the columns of the
 * matrix. n is always a valid value.
 */
template<typename T, typename I>
auto
delete_row_req( //
  proxsuite::linalg::veg::Tag<T> /*tag*/,
  proxsuite::linalg::veg::Tag<I> /*tag*/,
  isize n,
  isize max_nnz) noexcept -> proxsuite::linalg::veg::dynstack::StackReq
{
  return sparse::rank1_update_req(proxsuite::linalg::veg::Tag<T>{},
                                  proxsuite::linalg::veg::Tag<I>{},
                                  n,
                                  true,
                                  max_nnz);
}

/*!
 * Given the ldlt factors of matrix a, computes the ldlt factors of the matrix a
 * with row and column at position pos replaced by those of the identity matrix
 * It returns a view of the updated factors.
 *
 * @param ld : the ldlt factors
 * @param etree pointer to the elimination tree
 * @param perm_inv pointer to inverse permutation (for ex AMD). If this is null,
 * the permutation is assumed to be the identity.
 * @param pos position of the row and column to be deleted
 * @param stack is the memory stack
 */
template<typename T, typename I>
auto
delete_row(MatMut<T, I> ld,
           I* etree,
           I const* perm_inv,
           isize pos,
           DynStackMut stack) noexcept(false) -> MatMut<T, I>
{
  // step 1: delete row k from each column
  VEG_ASSERT(!ld.is_compressed());

  // we're actually deleting perm_inv[k], so that k is deleted in the permuted
  // matrix
  usize permuted_pos =
    perm_inv == nullptr ? usize(pos) : util::zero_extend(perm_inv[pos]);

  auto petree = etree;
  I* pldi = ld.row_indices_mut();
  T* pldx = ld.values_mut();
  I* pldnz = ld.nnz_per_col_mut();

  for (usize j = 0; j < permuted_pos; ++j) {
    auto col_start = ld.col_start(j) + 1;
    auto col_end = ld.col_end(j);
    // search for the first row in column j greater than or equal to k
    auto it =
      std::lower_bound(pldi + col_start, pldi + col_end, I(permuted_pos));

    // if an element was found, and it is equal to k
    if ((it != (pldi + col_end)) && *it == I(permuted_pos)) {
      usize it_pos = usize(it - (pldi + col_start));
      usize count = (col_end - col_start - it_pos);
      // shift all the row indices back by one position
      // to delete row k
      std::memmove(it, it + 1, count * sizeof(I));
      T* itx = pldx + col_start + it_pos;

      VEG_CHECK_CONCEPT(trivially_copyable<T>);
      // shift all the values back by one position
      std::memmove(itx, itx + 1, count * sizeof(T));

      // decrement the non zero count
      --pldnz[j];
      ld._set_nnz(ld.nnz() - 1);

      // adjust the parent of j in the elimination tree if necessary
      if (petree[j] == I(permuted_pos)) {
        VEG_ASSERT(it_pos == 0);
        if (pldnz[j] > 1) {
          petree[j] = *it;
        } else {
          petree[j] = I(-1);
        }
      }
    }
  }

  // step 2: set d_kk = 1
  T d_old = ld.values()[ld.col_start(permuted_pos)];
  ld.values_mut()[ld.col_start(permuted_pos)] = 1;

  // step 3: perform rank update
  isize len = isize(util::zero_extend(ld.nnz_per_col()[permuted_pos])) - 1;
  ld = sparse::rank1_update<T, I>( //
    ld,
    etree,
    static_cast<I const*>(nullptr),
    VecRef<T, I>{
      from_raw_parts,
      ld.nrows(),
      len,
      pldi + ld.col_start(permuted_pos) + 1,
      pldx + ld.col_start(permuted_pos) + 1,
    },
    d_old,
    stack);
  // step 4: delete col k_
  ld.nnz_per_col_mut()[permuted_pos] = 1;
  petree[permuted_pos] = I(-1);
  return ld;
}
/*!
 * Computes the memory requirements for adding a row and column for the ldlt
 * factors
 *
 * @param n : dimension of the matrix
 * @param id_perm : whether the permutation corresponds to the identity
 * @param nnz : number of non zero elements in the added vector
 * @param max_nnz : upper bound of non zero counts over the columns of the
 * matrix. n is always a valid value.
 */
template<typename T, typename I>
auto
add_row_req( //
  proxsuite::linalg::veg::Tag<T> /*tag*/,
  proxsuite::linalg::veg::Tag<I> /*tag*/,
  isize n,
  bool id_perm,
  isize nnz,
  isize max_nnz) noexcept -> proxsuite::linalg::veg::dynstack::StackReq
{
  using proxsuite::linalg::veg::dynstack::StackReq;
  auto numerical_work = StackReq{ n * isize{ sizeof(T) }, isize{ alignof(T) } };
  auto permuted_indices =
    StackReq{ (id_perm ? 0 : nnz) * isize{ sizeof(I) }, isize{ alignof(I) } };
  auto pattern_diff = StackReq{ n * isize{ sizeof(I) }, isize{ alignof(I) } };
  auto merge =
    merge_second_col_into_first_req(proxsuite::linalg::veg::Tag<I>{}, n);
  auto update = sparse::rank1_update_req(proxsuite::linalg::veg::Tag<T>{},
                                         proxsuite::linalg::veg::Tag<I>{},
                                         n,
                                         true,
                                         max_nnz);

  auto req = numerical_work;
  req = req & permuted_indices;
  req = req & pattern_diff;
  req = req & merge;
  req = req | update;

  return req;
}
/*!
 * Given the ldlt factors of matrix a, computes the ldlt factors of the matrix a
 * with added row and column at position pos. It is assumed that the row and
 * column are empty except the diagonal element. It returns a view of the
 * updated factors.
 *
 * @param ld : the ldlt factors
 * @param etree pointer to the elimination tree
 * @param perm_inv pointer to inverse permutation (for ex AMD). If this is null,
 * the permutation is assumed to be the identity.
 * @param pos position of the row and column to be added
 * @param new_col : new column to be added without the diagonal element (of size
 * nnz-1)
 * @param diag_element : diagonal element of the added row and column
 * @param stack is the memory stack
 */
template<typename T, typename I>
auto
add_row(MatMut<T, I> ld,
        I* etree,
        I const* perm_inv,
        isize pos,
        VecRef<T, I> new_col,
        proxsuite::linalg::veg::DoNotDeduce<T> diag_element,
        DynStackMut stack) noexcept(false) -> MatMut<T, I>
{
  VEG_ASSERT(!ld.is_compressed());
  bool id_perm = perm_inv == nullptr;
  auto zx = util::zero_extend;

  I* pldp = ld.col_ptrs_mut();
  I* pldnz = ld.nnz_per_col_mut();
  I* pldi = ld.row_indices_mut();
  T* pldx = ld.values_mut();

  // actually inserting in the position perm_inv[k] so that row k is added in
  // the permuted matrix
  usize permuted_pos = id_perm ? usize(pos) : zx(perm_inv[pos]);
  VEG_ASSERT(pldnz[permuted_pos] == 1);

  {
    // allocate workspace for numerical step, storage for the k-th row and k-th
    // column of the new matrix
    auto _lx2_storage = stack.make_new_for_overwrite(
      proxsuite::linalg::veg::Tag<T>{}, ld.nrows());
    auto plx2_storage = _lx2_storage.ptr_mut();

    // allocate workspace for permuted row indices of the new column if
    // necessary
    auto _new_col_permuted_indices = stack.make_new_for_overwrite(
      proxsuite::linalg::veg::Tag<I>{}, id_perm ? isize(0) : new_col.nnz());

    auto new_col_permuted_indices =
      id_perm ? new_col.row_indices() : _new_col_permuted_indices.ptr();

    // copy and sort permuted row indices
    if (!id_perm) {
      I* pnew_col_permuted_indices = _new_col_permuted_indices.ptr_mut();
      for (usize k = 0; k < usize(new_col.nnz()); ++k) {
        usize i = zx(new_col.row_indices()[k]);
        pnew_col_permuted_indices[k] = perm_inv[i];
      }
      std::sort(pnew_col_permuted_indices,
                pnew_col_permuted_indices + new_col.nnz());
    }

    // allocate workspace for non-zero pattern of k-th row
    auto _l12_nnz_pattern = stack.make_new_for_overwrite(
      proxsuite::linalg::veg::Tag<I>{}, isize(permuted_pos));
    auto _difference = stack.make_new_for_overwrite(
      proxsuite::linalg::veg::Tag<I>{}, ld.nrows() - isize(permuted_pos));
    auto pdifference = _difference.ptr_mut();

    auto pl12_nnz_pattern = _l12_nnz_pattern.ptr_mut();
    usize l12_nnz_pattern_count = 0;

    // the non-zero pattern is the set of columns reachable from the non-zero
    // pattern of the added column through graph of L_{1..k,1..k}
    // instead of graph traversal, we can use the k-th elimination subtree as we
    // did in the initial factorization step

    // for each row in the added column
    {
      auto _visited = stack.make_new(proxsuite::linalg::veg::Tag<bool>{},
                                     isize(permuted_pos));
      bool* visited = _visited.ptr_mut();
      for (usize p = 0; p < usize(new_col.nnz()); ++p) {
        auto j = zx(new_col_permuted_indices[p]);
        if (j >= permuted_pos) {
          break;
        }

        // add the ancestors of the corresponding column
        // ancestors are not sorted, but they are added in topological order,
        // which suffices for the triangular solve
        while (true) {
          if (visited[j]) {
            break;
          }
          visited[j] = true;
          pl12_nnz_pattern[l12_nnz_pattern_count] = I(j);
          ++l12_nnz_pattern_count;

          j = util::sign_extend(etree[j]);
          if (j == usize(-1) || j >= permuted_pos || visited[j]) {
            break;
          }
        }
      }
    }
    std::sort(pl12_nnz_pattern, pl12_nnz_pattern + l12_nnz_pattern_count);

    // zero the elements in the non-zero pattern of the solution (new k-th row)
    for (usize p = 0; p < l12_nnz_pattern_count; ++p) {
      plx2_storage[zx(pl12_nnz_pattern[p])] = 0;
    }

    // insert the rhs of the k-th row triangular system in the top part of the
    // storage, and the bottom part of the added column in the bottom part of
    // the storage
    for (usize p = 0; p < usize(new_col.nnz()); ++p) {
      auto j = zx(new_col.row_indices()[p]);
      auto permuted_j = id_perm ? j : zx(perm_inv[j]);
      plx2_storage[permuted_j] = new_col.values()[p];

      // add the row indices of the bottom part of the added column, to the
      // k-th column of L
      if (permuted_j > permuted_pos) {
        usize nz = zx(pldnz[permuted_pos]);
        VEG_ASSERT(nz < (zx(pldp[permuted_pos + 1]) - zx(pldp[permuted_pos])));
        pldi[zx(pldp[permuted_pos]) + nz] = I(permuted_j);
        ++pldnz[permuted_pos];
        ld._set_nnz(ld.nnz() + 1);
      }
    }
    // sort the added row indices
    std::sort(pldi + zx(pldp[permuted_pos]) + 1,
              pldi + zx(pldp[permuted_pos]) + zx(pldnz[permuted_pos]));

    // TODO: fuse loops?

    for (usize p = 0; p < l12_nnz_pattern_count; ++p) {
      usize j = zx(pl12_nnz_pattern[p]);
      auto col_start = ld.col_start(j);
      auto col_end = ld.col_end(j);

      // update the pattern of the k-th column of L, with that of the bottom
      // part of the j-th column of L, ignoring the elements less than or equal
      // to k
      VEG_BIND(auto,
               (_, new_current_col, computed_difference),
               sparse::merge_second_col_into_first(
                 pdifference,
                 static_cast<T*>(nullptr),
                 pldi + (zx(pldp[permuted_pos]) + 1),
                 isize(zx(pldp[permuted_pos + 1]) - zx(pldp[permuted_pos])) - 1,
                 pldnz[permuted_pos] - 1,
                 {
                   unsafe,
                   from_raw_parts,
                   pldi + (zx(pldp[j]) + 1),
                   isize(zx(pldnz[j])) - 1,
                 },
                 I(permuted_pos),
                 false,
                 stack));
      (void)_;
      (void)new_current_col;

      // update column and global non-zero count
      pldnz[permuted_pos] += I(computed_difference.len());
      ld._set_nnz(ld.nnz() + computed_difference.len());

      for (usize q = 0; q < usize(computed_difference.len()); ++q) {
        plx2_storage[zx(computed_difference.ptr()[q])] = 0;
      }

      // perform triangular solve and matrix vector product simultaneously
      auto const xj = plx2_storage[j];
      for (usize q = col_start + 1; q < col_end; ++q) {
        auto i = zx(pldi[q]);
        plx2_storage[i] -= pldx[q] * xj;
      }
    }

    // insert the k-th row into L
    for (usize p = 0; p < l12_nnz_pattern_count; ++p) {
      // for each column in the non-zero pattern of the k-th row

      usize j = zx(pl12_nnz_pattern[p]);
      auto col_start = ld.col_start(j);
      auto col_end = ld.col_end(j);
      T d = pldx[col_start];
      T l12_elem = plx2_storage[j];
      diag_element -= l12_elem * l12_elem / d;

      // check that we have enough space to insert one element
      VEG_ASSERT(zx(pldnz[j]) < (zx(pldp[j + 1]) - zx(pldp[j])));

      // find the first element greater than k
      auto it =
        std::lower_bound(pldi + col_start, pldi + col_end, I(permuted_pos));

      // if it is the first element, update the elimination tree so that k is
      // the new parent of column j
      if (it == (pldi + col_start + 1)) {
        etree[j] = I(permuted_pos);
      }

      // shift the row indices  up by one position to provide enough space for
      // the new element
      std::memmove( //
        it + 1,
        it,
        usize((pldi + col_end) - it) * sizeof(I));

      VEG_CHECK_CONCEPT(trivially_copyable<T>);

      // shift the values  up by one position to provide enough space for the
      // new element
      std::memmove( //
        pldx + (it - pldi) + 1,
        pldx + (it - pldi),
        usize((pldi + col_end) - it) * sizeof(T));

      // insert the new row index k
      *it = I(permuted_pos);
      // insert the new corresponding value
      *(pldx + (it - pldi)) = l12_elem / d;
      // update the non-zero count
      ++pldnz[j];
      ld._set_nnz(ld.nnz() + 1);
    }

    // insert the k-th column of L
    {
      usize col_start = ld.col_start(permuted_pos);
      usize col_end = ld.col_end(permuted_pos);
      pldx[col_start] = diag_element;
      for (usize p = col_start + 1; p < col_end; ++p) {
        pldx[p] = plx2_storage[zx(pldi[p])] / diag_element;
      }
    }
  }

  // set the parent of the k-th column of L
  if (pldnz[permuted_pos] > 1) {
    etree[permuted_pos] = pldi[ld.col_start(permuted_pos) + 1];
  }

  isize len = isize(util::zero_extend(ld.nnz_per_col()[permuted_pos])) - 1;
  // perform the rank update with the newly added column
  ld = sparse::rank1_update<T, I>(ld,
                                  etree,
                                  static_cast<I const*>(nullptr),
                                  VecRef<T, I>{
                                    from_raw_parts,
                                    ld.nrows(),
                                    len,
                                    pldi + ld.col_start(permuted_pos) + 1,
                                    pldx + ld.col_start(permuted_pos) + 1,
                                  },
                                  -diag_element,
                                  stack);

  return ld;
}
} // namespace sparse
} // namespace linalg
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_LINALG_SPARSE_LDLT_ROWMOD_HPP */
