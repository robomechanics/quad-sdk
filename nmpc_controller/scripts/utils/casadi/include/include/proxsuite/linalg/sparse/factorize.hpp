/** \file */
//
// Copyright (c) 2022 INRIA
//
#ifndef PROXSUITE_LINALG_SPARSE_LDLT_FACTORIZE_HPP
#define PROXSUITE_LINALG_SPARSE_LDLT_FACTORIZE_HPP

#include "proxsuite/linalg/sparse/core.hpp"
#include <Eigen/OrderingMethods>

namespace proxsuite {
namespace linalg {
namespace sparse {

template<typename I>
auto
transpose_req(proxsuite::linalg::veg::Tag<I> /*tag*/, isize nrows) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return { nrows * isize(sizeof(I)), isize(alignof(I)) };
}

// at = a.T
template<typename T, typename I>
void
transpose( //
  MatMut<T, I> at,
  MatRef<T, I> a,
  DynStackMut stack) noexcept(VEG_CONCEPT(nothrow_copyable<T>))
{
  using namespace _detail;

  VEG_ASSERT_ALL_OF( //
    at.is_compressed(),
    at.nrows() == a.ncols(),
    at.ncols() == a.nrows(),
    at.nnz() == a.nnz());

  auto pai = a.row_indices();
  auto pax = a.values();

  auto patp = at.col_ptrs_mut();
  auto pati = at.row_indices_mut();
  auto patx = at.values_mut();

  auto _work = stack.make_new(proxsuite::linalg::veg::Tag<I>{}, at.ncols());
  auto work = _work.ptr_mut();

  // work[i] = num zeros in ith row of A
  if (a.is_compressed()) {
    for (usize p = 0; p < usize(a.nnz()); ++p) {
      util::wrapping_inc(mut(work[util::zero_extend(pai[p])]));
    }
  } else {
    for (usize j = 0; j < a.ncols(); ++j) {
      isize col_start = a.col_start(j);
      isize col_end = a.col_end(j);
      for (isize p = col_start; p < col_end; ++p) {
        util::wrapping_inc(mut(work[util::zero_extend(pai[p])]));
      }
    }
  }

  // compute the cumulative sum
  for (usize j = 0; j < usize(at.ncols()); ++j) {
    patp[j + 1] = util::checked_non_negative_plus(patp[j], work[j]);
    work[j] = patp[j];
  }

  for (usize j = 0; j < usize(a.ncols()); ++j) {
    auto col_start = a.col_start(j);
    auto col_end = a.col_end(j);
    for (usize p = col_start; p < col_end; ++p) {
      auto i = util::zero_extend(pai[p]);
      auto q = util::zero_extend(work[i]);

      pati[q] = j;
      patx[q] = pax[p];
      util::wrapping_inc(mut(work[i]));
    }
  }
}

template<typename I>
auto
transpose_symbolic_req(proxsuite::linalg::veg::Tag<I> /*tag*/,
                       isize nrows) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return { nrows * isize(sizeof(I)), isize(alignof(I)) };
}

template<typename I>
void
transpose_symbolic( //
  SymbolicMatMut<I> at,
  SymbolicMatRef<I> a,
  DynStackMut stack) noexcept
{
  using namespace _detail;

  VEG_ASSERT_ALL_OF( //
    at.is_compressed(),
    at.nrows() == a.ncols(),
    at.ncols() == a.nrows(),
    at.nnz() == a.nnz());

  auto pai = a.row_indices();

  auto patp = at.col_ptrs_mut();
  auto pati = at.row_indices_mut();

  auto _work = stack.make_new(proxsuite::linalg::veg::Tag<I>{}, at.ncols());
  auto work = _work.ptr_mut();

  // work[i] = num zeros in ith row of A
  for (usize p = 0; p < usize(a.nnz()); ++p) {
    util::wrapping_inc(mut(work[util::zero_extend(pai[p])]));
  }

  // compute the cumulative sum
  for (usize j = 0; j < usize(at.ncols()); ++j) {
    patp[j + 1] = util::checked_non_negative_plus(patp[j], work[j]);
    work[j] = patp[j];
  }

  for (usize j = 0; j < usize(a.ncols()); ++j) {
    auto col_start = a.col_start(j);
    auto col_end = a.col_end(j);
    for (usize p = col_start; p < col_end; ++p) {
      auto i = util::zero_extend(pai[p]);
      auto q = util::zero_extend(work[i]);

      pati[q] = I(j);
      util::wrapping_inc(mut(work[i]));
    }
  }
}

/*!
 * `l` is unit lower triangular whose diagonal elements are ignored.
 * Solves `l×y = x` and store the solution in `x`.
 *
 * @param x RHS of the system, solution storage.
 * @param l matrix to be inverted.
 */
template<typename T, typename I>
void
dense_lsolve(DenseVecMut<T> x, MatRef<T, I> l) noexcept(false)
{
  using namespace _detail;

  VEG_ASSERT_ALL_OF( //
    l.nrows() == l.ncols(),
    x.nrows() == l.nrows()
    /* l is unit lower triangular */
  );

  usize n = usize(l.nrows());

  auto pli = l.row_indices();
  auto plx = l.values();

  auto px = x.as_slice_mut().ptr_mut();

  for (usize j = 0; j < n; ++j) {
    auto const xj = px[j];
    auto col_start = l.col_start(j);
    auto col_end = l.col_end(j);

    // skip the diagonal entry
    for (usize p = col_start + 1; p < col_end; ++p) {
      auto i = util::zero_extend(pli[p]);
      px[i] -= plx[p] * xj;
    }
  }
}

/*!
 * `l` is unit lower triangular whose diagonal elements are ignored.
 * Solves `l.T×y = x` and store the solution in `x`.
 *
 * @param x RHS of the system, solution storage.
 * @param l matrix to be inverted.
 */
template<typename T, typename I>
void
dense_ltsolve(DenseVecMut<T> x, MatRef<T, I> l) noexcept(false)
{
  using namespace _detail;

  VEG_ASSERT_ALL_OF( //
    l.nrows() == l.ncols(),
    x.nrows() == l.nrows()
    /* l is unit lower triangular */
  );

  usize n = usize(l.nrows());

  auto pli = l.row_indices();
  auto plx = l.values();

  auto px = x.as_slice_mut().ptr_mut();

  usize j = n;
  while (true) {
    if (j == 0) {
      break;
    }
    --j;

    auto col_start = l.col_start(j);
    auto col_end = l.col_end(j);
    T acc0 = 0;
    T acc1 = 0;
    T acc2 = 0;
    T acc3 = 0;

    // skip the diagonal entry
    usize pstart = col_start + 1;
    usize pcount = col_end - pstart;

    usize p = pstart;
    for (; p < pstart + pcount / 4 * 4; p += 4) {
      auto i0 = util::zero_extend(pli[p + 0]);
      auto i1 = util::zero_extend(pli[p + 1]);
      auto i2 = util::zero_extend(pli[p + 2]);
      auto i3 = util::zero_extend(pli[p + 3]);
      acc0 += plx[p + 0] * px[i0];
      acc1 += plx[p + 1] * px[i1];
      acc2 += plx[p + 2] * px[i2];
      acc3 += plx[p + 3] * px[i3];
    }
    for (; p < pstart + pcount; ++p) {
      auto i0 = util::zero_extend(pli[p + 0]);
      acc0 += plx[p + 0] * px[i0];
    }

    acc0 = (acc0 + acc1) + (acc2 + acc3);

    px[j] -= acc0;
  }
}

/*!
 * Computes the stack memory requirements of etree computation.
 *
 * @param n dimension of the matrix.
 */
template<typename I>
auto
etree_req(proxsuite::linalg::veg::Tag<I> /*tag*/, isize n) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return { n * isize{ sizeof(I) }, alignof(I) };
}

/*!
 * Computes the elimination tree of the cholesky factor  of `a` of size `n`.
 * `a` is considered symmetric but should only contain terms from the upper
 * triangular part.
 *
 * @param parent pointer to the elimination tree storage, of size `n`.
 * @param a symbolic structure of the matrix to be factorized.
 * @param stack temporary allocation stack
 */
template<typename I>
VEG_INLINE void
etree( //
  I* parent,
  SymbolicMatRef<I> a,
  DynStackMut stack) noexcept
{
  using namespace _detail;

  usize n = usize(a.ncols());
  auto pai = a.row_indices();

  auto _work =
    stack.make_new_for_overwrite(proxsuite::linalg::veg::Tag<I>{}, isize(n));
  auto pancestors = _work.ptr_mut();

  // for each column of a
  for (usize k = 0; k < n; ++k) {
    parent[k] = I(-1);
    pancestors[k] = I(-1);
    // assuming elimination subtree T_{k-1} is known, compute T_k

    auto col_start = a.col_start(k);
    auto col_end = a.col_end(k);

    // for each non zero element of a
    for (usize p = col_start; p < col_end; ++p) {
      // get the row
      auto i = util::zero_extend(pai[p]);
      // skip if looking at lower triangular half
      if (i >= k) {
        continue;
      }

      // go up towards the root of the tree
      usize node = i;
      auto next = usize(-1);
      while (true) {
        if (node == usize(-1) || node >= k) {
          break;
        }

        // use the highest known ancestor instead of the parent
        next = util::sign_extend(pancestors[node]);

        // set the highest known ancestor to k, since we know we're going
        // to find it eventually
        pancestors[node] = I(k);

        // if there is no highest known ancestor, we must have hit the root of
        // the tree
        // set the parent to k
        if (next == usize(-1)) {
          parent[node] = I(k);
          break;
        }
        // go to the highest ancestor
        node = next;
      }
    }
  }
}

namespace _detail {
inline auto
ereach_req(isize k) noexcept -> proxsuite::linalg::veg::dynstack::StackReq
{
  return { (k + 1) * isize{ sizeof(bool) }, alignof(bool) };
}

// compute the set of reachable nodes from the non zero pattern of a_{.,k}
// not including the node k itself
template<typename I>
VEG_NODISCARD VEG_INLINE auto
ereach(usize& count,
       I* s,
       SymbolicMatRef<I> a,
       I const* parent,
       isize k,
       bool* pmarked) noexcept -> I*
{

  usize n = usize(a.ncols());
  auto k_ = usize(k);

  auto pai = a.row_indices();
  auto pparent = parent;

  auto col_start = a.col_start(k_);
  auto col_end = a.col_end(k_);

  pmarked[k_] = true;

  usize top = n;

  // for each non zero element of a_{.,k}
  for (usize p = col_start; p < col_end; ++p) {
    auto i = util::zero_extend(pai[p]);

    // only triu part of a
    if (i > k_) {
      continue;
    }

    usize len = 0;
    while (true) {
      // if we reach a marked node
      if (pmarked[i]) {
        break;
      }

      // can't overwrite top of the stack since elements of s are unique
      // and s is large enough to hold all the nodes
      s[isize(len)] = I(i);
      util::wrapping_inc(mut(len));

      // mark node i as reached
      pmarked[i] = true;
      i = util::sign_extend(pparent[i]);
    }

    // make sure that we can memmove
    std::memmove( //
      s + (top - len),
      s,
      usize(len) * sizeof(I));

    // move down the top of the stack
    top = util::wrapping_plus(top, -len);
  }

  for (usize q = top; q < n; ++q) {
    pmarked[s[q]] = false;
  }
  pmarked[k_] = false;

  // [top, end[
  count = n - top;
  return s + top;
}
} // namespace _detail

namespace _detail {
// return the next start_index
template<typename I>
VEG_INLINE auto
postorder_depth_first_search( //
  I* post,
  usize root,
  usize start_index,
  I* pstack,
  I* pfirst_child,
  I* pnext_child) noexcept -> usize
{
  using namespace _detail;

  usize top = 0;
  pstack[0] = I(root);

  // stack is non empty
  while (top != usize(-1)) {
    auto current_node = util::zero_extend(pstack[top]);
    auto current_child = util::sign_extend(pfirst_child[current_node]);

    // no more children
    if (current_child == usize(-1)) {
      post[start_index] = I(current_node);
      ++start_index;

      // pop node from the stack
      util::wrapping_dec(mut(top));
    } else {
      // add current child to the stack
      util::wrapping_inc(mut(top));
      pstack[top] = I(current_child);

      // next child is now the first child
      pfirst_child[current_node] = pnext_child[current_child];
    }
  }
  return start_index;
}
} // namespace _detail

/*!
 * Computes the memory requirements of the postordering of the cholesky
 * factorization.
 *
 * @param n dimension of the matrix to be factorized.
 */
template<typename I>
auto
postorder_req(proxsuite::linalg::veg::Tag<I> /*tag*/, isize n) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return { (3 * n) * isize(sizeof(I)), alignof(I) };
}

/*!
 * Computes the postordering of the cholesky factorization of dimension `n`.
 *
 * @param post storage for the postordering, of size `n`
 * @param parent pointer to the elimination tree
 * @param n dimension of the matrix to be factorized
 * @param stack temporary allocation stack
 */
template<typename I>
void
postorder(I* post, I const* parent, isize n, DynStackMut stack) noexcept
{
  using namespace _detail;

  auto _work = stack.make_new_for_overwrite(proxsuite::linalg::veg::Tag<I>{},
                                            3 * isize(n));
  I* pwork = _work.ptr_mut();

  I* pstack = pwork;
  I* pfirst_child = pstack + n;
  I* pnext_child = pfirst_child + n;

  // no children are found yet
  for (usize j = 0; j < usize(n); ++j) {
    pfirst_child[j] = I(-1);
  }

  for (usize _j = 0; _j < usize(n); ++_j) {
    // traverse in reverse order, since the children appear in reverse order
    // of insertion in the linked list
    usize j = usize(n) - 1 - _j;

    // if not a root node
    if (parent[isize(j)] != I(-1)) {
      // next child of this node is the previous first child
      pnext_child[j] = pfirst_child[util::zero_extend(parent[isize(j)])];
      // set this node to be the new first child
      pfirst_child[util::zero_extend(parent[isize(j)])] = I(j);
    }
  }

  usize start_index = 0;
  for (usize root = 0; root < usize(n); ++root) {
    if (parent[isize(root)] == I(-1)) {
      start_index = _detail::postorder_depth_first_search(
        post, root, start_index, pstack, pfirst_child, pnext_child);
    }
  }
}

namespace _detail {
// returns -2 if j is not a leaf
// returns -1 if j is a first leaf
// returns the least common ancestor of j and the previous j otherwise
template<typename I>
VEG_INLINE auto
least_common_ancestor(usize i,
                      usize j,
                      I const* pfirst,
                      I* pmax_first,
                      I* pprev_leaf,
                      I* pancestor) noexcept -> I
{
  using namespace _detail;

  // if upper triangular part, or not a leaf
  // leaves always have a new larger value of pfirst
  // add 1 to get the correct result when comparing with -1
  if (i <= j || util::wrapping_plus(pfirst[j], I(1)) <=
                  util::wrapping_plus(pmax_first[i], I(1))) {
    return I(-2);
  }

  // update the largest max_first
  // no need to compare because the value of j increases
  // inbetween successive calls, and so does first_j
  pmax_first[i] = pfirst[j];

  // get the previous j
  usize j_prev = util::sign_extend(pprev_leaf[i]);

  // set the previous j to the current j
  pprev_leaf[i] = I(j);

  // if first leaf
  if (j_prev == usize(-1)) {
    return I(-1);
  }

  // else, subsequent leaf
  // get the least common ancestor of j and j_prev
  usize lca = j_prev;
  while (true) {
    if (lca == util::zero_extend(pancestor[lca])) {
      break;
    }
    lca = util::zero_extend(pancestor[lca]);
  }

  // compress the path to speed up the subsequent calls
  // to this function
  usize node = j_prev;
  while (true) {
    if (node == lca) {
      break;
    }
    usize next = util::zero_extend(pancestor[node]);
    pancestor[node] = I(lca);
    node = next;
  }

  return I(lca);
}
} // namespace _detail

template<typename I>
auto
column_counts_req(proxsuite::linalg::veg::Tag<I> tag,
                  isize n,
                  isize nnz) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  using proxsuite::linalg::veg::dynstack::StackReq;
  return StackReq{
    isize{ sizeof(I) } * (1 + 5 * n + nnz),
    alignof(I),
  } & sparse::transpose_symbolic_req(tag, n);
}

template<typename I>
void
column_counts(I* counts,
              SymbolicMatRef<I> a,
              I const* parent,
              I const* post,
              DynStackMut stack) noexcept
{
  // https://youtu.be/uZKJPTo4dZs
  using namespace _detail;
  usize n = usize(a.nrows());
  auto _at_work = stack.make_new_for_overwrite(proxsuite::linalg::veg::Tag<I>{},
                                               1 + 5 * isize(n) + a.nnz());
  auto pat_work = _at_work.ptr_mut();
  pat_work[0] = 0;
  pat_work[n] = I(a.nnz());

  SymbolicMatMut<I> at{
    from_raw_parts, isize(n), isize(n),         a.nnz(),
    pat_work,       nullptr,  pat_work + n + 1,
  };
  sparse::transpose_symbolic(at, a, stack);

  auto patp = at.col_ptrs();
  auto pati = at.row_indices();

  auto pwork = pat_work + n + 1 + a.nnz();

  auto pdelta = counts;

  auto pfirst = pwork;
  auto pmax_first = pwork + n;
  auto pprev_leaf = pwork + 2 * n;
  auto pancestor = pwork + 3 * n;

  auto pcounts = counts;
  auto ppost = post;
  auto pparent = parent;

  for (usize i = 0; i < 3 * n; ++i) {
    pwork[i] = I(-1);
  }
  for (usize i = 0; i < n; ++i) {
    pancestor[i] = I(i);
  }

  // for each column in a
  for (usize k = 0; k < n; ++k) {
    // in postordered fashion
    auto j = util::zero_extend(ppost[k]);

    // if first_j isn't computed, j must be a leaf
    // because if it's not a leaf then first_j will be initialized from
    // the init loop of its first descendant
    //
    // in which case initialize delta_j to 1
    pdelta[j] = (pfirst[j] == I(-1)) ? I(1) : I(0);

    // init loop
    while (true) {
      // while j is not a root, and the first descendant of j isn't computed
      // set the first descendant of j to k, as well as all of its ancestors
      // that don't yet have a first descendant
      if (j == usize(-1) || pfirst[j] != I(-1)) {
        break;
      }
      pfirst[j] = I(k);
      j = util::sign_extend(pparent[j]);
    }
  }

  // for each node
  for (usize k = 0; k < n; ++k) {
    // in postordered fashion
    auto j = util::zero_extend(ppost[k]);

    // if this node is the child of some other node
    if (pparent[j] != I(-1)) {
      // decrement the delta of that node
      // corresponding to the correction term e_j
      util::wrapping_dec(mut(pdelta[util::zero_extend(pparent[j])]));
    }

    auto col_start = util::zero_extend(patp[j]);
    auto col_end = util::zero_extend(patp[j + 1]);

    // iterate over lower triangular half of a
    for (usize p = col_start; p < col_end; ++p) {
      auto i = util::zero_extend(pati[p]);
      I lca = _detail::least_common_ancestor( //
        i,
        j,
        pfirst,
        pmax_first,
        pprev_leaf,
        pancestor);

      // if j is a leaf of T^i
      if (lca != I(-2)) {
        util::wrapping_inc(mut(pdelta[j]));

        // if j is a subsequent leaf
        if (lca != I(-1)) {
          util::wrapping_dec(mut(pdelta[util::zero_extend(lca)]));
        }
      }
    }

    if (pparent[j] != -1) {
      // set the ancestor of j
      pancestor[j] = pparent[j];
    }
  }

  // sum up the deltas
  for (usize j = 0; j < n; ++j) {
    if (parent[isize(j)] != I(-1)) {
      pcounts[util::zero_extend(parent[isize(j)])] = util::wrapping_plus(
        pcounts[util::zero_extend(parent[isize(j)])], pcounts[j]);
    }
  }
}

template<typename I>
auto
amd_req(proxsuite::linalg::veg::Tag<I> /*tag*/, isize /*n*/, isize nnz) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return { nnz * isize{ sizeof(char) }, alignof(char) };
}

template<typename I>
void
amd(I* perm, SymbolicMatRef<I> mat, DynStackMut stack) noexcept
{
  // TODO: reimplement amd under BSD-3
  // https://github.com/DrTimothyAldenDavis/SuiteSparse/tree/master/AMD

  isize n = mat.nrows();
  isize nnz = mat.nnz();

  Eigen::PermutationMatrix<-1, -1, I> perm_eigen;
  auto _ = stack.make_new(proxsuite::linalg::veg::Tag<char>{}, nnz);

  Eigen::AMDOrdering<I>{}(
    Eigen::Map<Eigen::SparseMatrix<char, Eigen::ColMajor, I> const>{
      n,
      n,
      nnz,
      mat.col_ptrs(),
      mat.row_indices(),
      _.ptr(),
      mat.nnz_per_col(),
    }
      .template selfadjointView<Eigen::Upper>(),

    perm_eigen);
  std::memmove( //
    perm,
    perm_eigen.indices().data(),
    usize(n) * sizeof(I));
}

namespace _detail {
template<typename I>
void
inv_perm(I* perm_inv, I const* perm, isize n) noexcept
{
  for (usize i = 0; i < usize(n); ++i) {
    perm_inv[util::zero_extend(perm[i])] = I(i);
  }
}

template<typename I>
auto
symmetric_permute_symbolic_req(proxsuite::linalg::veg::Tag<I> /*tag*/,
                               isize n) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return { n * isize{ sizeof(I) }, alignof(I) };
}
template<typename I>
auto
symmetric_permute_req(proxsuite::linalg::veg::Tag<I> /*tag*/, isize n) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  return { n * isize{ sizeof(I) }, alignof(I) };
}

template<typename I>
void
symmetric_permute_common(usize n,
                         I const* pperm_inv,
                         SymbolicMatRef<I> old_a,
                         I* pnew_ap,
                         I* pcol_counts)
{
  for (usize old_j = 0; old_j < n; ++old_j) {
    usize new_j = util::zero_extend(pperm_inv[old_j]);

    auto col_start = old_a.col_start(old_j);
    auto col_end = old_a.col_end(old_j);

    for (usize p = col_start; p < col_end; ++p) {
      usize old_i = util::zero_extend(old_a.row_indices()[p]);

      if (old_i <= old_j) {
        usize new_i = util::zero_extend(pperm_inv[old_i]);
        util::wrapping_inc(mut(pcol_counts[new_i > new_j ? new_i : new_j]));
      }
    }
  }

  pnew_ap[0] = I(0);
  for (usize i = 0; i < n; ++i) {
    pnew_ap[i + 1] =
      util::checked_non_negative_plus(pnew_ap[i], pcol_counts[i]);
    pcol_counts[i] = pnew_ap[i];
  }
}

template<typename I>
void
symmetric_permute_symbolic(SymbolicMatMut<I> new_a,
                           SymbolicMatRef<I> old_a,
                           I const* perm_inv,
                           DynStackMut stack) noexcept
{

  usize n = usize(new_a.nrows());

  auto _work = stack.make_new(proxsuite::linalg::veg::Tag<I>{}, isize(n));
  I* pcol_counts = _work.ptr_mut();

  VEG_ASSERT(new_a.is_compressed());
  auto pold_ai = old_a.row_indices();

  auto pnew_ap = new_a.col_ptrs_mut();
  auto pnew_ai = new_a.row_indices_mut();

  auto pperm_inv = perm_inv;

  _detail::symmetric_permute_common(n, pperm_inv, old_a, pnew_ap, pcol_counts);

  auto pcurrent_row_index = pcol_counts;

  for (usize old_j = 0; old_j < n; ++old_j) {
    usize new_j = util::zero_extend(pperm_inv[old_j]);

    auto col_start = old_a.col_start(old_j);
    auto col_end = old_a.col_end(old_j);

    for (usize p = col_start; p < col_end; ++p) {
      usize old_i = util::zero_extend(pold_ai[p]);

      if (old_i <= old_j) {
        usize new_i = util::zero_extend(pperm_inv[old_i]);

        usize new_max = new_i > new_j ? new_i : new_j;
        usize new_min = new_i < new_j ? new_i : new_j;

        auto row_idx = pcurrent_row_index[new_max];
        pnew_ai[row_idx] = I(new_min);
        pcurrent_row_index[new_max] = util::wrapping_plus(row_idx, I(1));
      }
    }
  }
}

template<typename T, typename I>
void
symmetric_permute(MatMut<T, I> new_a,
                  MatRef<T, I> old_a,
                  I const* perm_inv,
                  DynStackMut stack) noexcept(VEG_CONCEPT(nothrow_copyable<T>))
{
  usize n = usize(new_a.nrows());
  auto _work = stack.make_new(proxsuite::linalg::veg::Tag<I>{}, isize(n));
  I* pcol_counts = _work.ptr_mut();

  VEG_ASSERT(new_a.is_compressed());
  auto pold_ai = old_a.row_indices();

  auto pnew_ap = new_a.col_ptrs_mut();
  auto pnew_ai = new_a.row_indices_mut();

  auto pperm_inv = perm_inv;

  _detail::symmetric_permute_common(
    n, pperm_inv, old_a.symbolic(), pnew_ap, pcol_counts);

  auto pcurrent_row_index = pcol_counts;

  auto pold_ax = old_a.values();
  auto pnew_ax = new_a.values_mut();
  for (usize old_j = 0; old_j < n; ++old_j) {
    usize new_j = util::zero_extend(pperm_inv[old_j]);

    auto col_start = old_a.col_start(old_j);
    auto col_end = old_a.col_end(old_j);

    for (usize p = col_start; p < col_end; ++p) {
      usize old_i = util::zero_extend(pold_ai[p]);

      if (old_i <= old_j) {
        usize new_i = util::zero_extend(pperm_inv[old_i]);

        usize new_max = new_i > new_j ? new_i : new_j;
        usize new_min = new_i < new_j ? new_i : new_j;

        auto row_idx = pcurrent_row_index[new_max];
        pnew_ai[row_idx] = I(new_min);
        pnew_ax[row_idx] = pold_ax[p];
        pcurrent_row_index[new_max] = util::wrapping_plus(row_idx, I(1));
      }
    }
  }
}
} // namespace _detail

enum struct Ordering : unsigned char
{
  natural,
  user_provided,
  amd,
  ENUM_END,
};

/*!
 * Computes the stack memory requirements of symbolic factorization.
 *
 * @param n dimension of the matrix to be factorized.
 * @param nnz number of non zeros of the matrix to be factorized.
 * @param o the kind of permutation that is applied to the matrix before
 * factorization.
 */
template<typename I>
auto
factorize_symbolic_req(proxsuite::linalg::veg::Tag<I> tag,
                       isize n,
                       isize nnz,
                       Ordering o) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  using proxsuite::linalg::veg::dynstack::StackReq;
  constexpr isize sz{ sizeof(I) };
  constexpr isize al{ alignof(I) };

  StackReq perm_req{ 0, al };
  StackReq amd_req{ 0, al };
  switch (o) {
    case Ordering::natural:
      break;
    case Ordering::amd:
      amd_req =
        StackReq{ n * sz, al } & StackReq{ sparse::amd_req(tag, n, nnz) };
      HEDLEY_FALL_THROUGH;
    case Ordering::user_provided:
      perm_req = perm_req & StackReq{ (n + 1 + nnz) * sz, al };
      perm_req = perm_req & _detail::symmetric_permute_symbolic_req(tag, n);
    default:
      break;
  }

  StackReq parent_req = { n * sz, al };
  StackReq post_req = { n * sz, al };

  StackReq etree_req = sparse::etree_req(tag, n);
  StackReq postorder_req = sparse::postorder_req(tag, n);
  StackReq colcount_req = sparse::column_counts_req(tag, n, nnz);

  return amd_req              //
         | (perm_req          //
            & (parent_req     //
               & (etree_req   //
                  | (post_req //
                     & (postorder_req | colcount_req)))));
}

/*!
 * Performs symbolic factorization and computed the number of non-zeros in each
 * column of the cholesky factor of dimension `n`.
 *
 * @param nnz_per_col storage for non-zeros per column, of size `n`
 * @param etree storage for elimination tree, of size `n`
 * @param perm_inv storage for inverse permutation, of size `n`
 * @param perm optionally user-provided permutation, either null or of size `n`
 * @param a matrix to be symbolically factorized
 * @param stack temporary allocation stack
 */
template<typename I>
void
factorize_symbolic_non_zeros(I* nnz_per_col,
                             I* etree,
                             I* perm_inv,
                             I const* perm,
                             SymbolicMatRef<I> a,
                             DynStackMut stack) noexcept
{

  bool id_perm = perm_inv == nullptr;
  bool user_perm = perm != nullptr;

  Ordering o = user_perm ? Ordering::user_provided
               : id_perm ? Ordering::natural
                         : Ordering::amd;

  proxsuite::linalg::veg::Tag<I> tag{};

  usize n = usize(a.ncols());

  switch (o) {
    case Ordering::natural:
      break;

    case Ordering::amd: {
      auto amd_perm = stack.make_new_for_overwrite(tag, isize(n));
      sparse::amd(amd_perm.ptr_mut(), a, stack);
      perm = amd_perm.ptr();
    }
      HEDLEY_FALL_THROUGH;
    case Ordering::user_provided: {
      _detail::inv_perm(perm_inv, perm, isize(n));
    }
    default:
      break;
  }

  auto _permuted_a_col_ptrs =
    stack //
      .make_new_for_overwrite(tag, id_perm ? 0 : (a.ncols() + 1));
  auto _permuted_a_row_indices =
    stack //
      .make_new_for_overwrite(tag, id_perm ? 0 : (a.nnz()));

  if (!id_perm) {
    _permuted_a_col_ptrs.as_mut()[0] = 0;
    _permuted_a_col_ptrs.as_mut()[isize(n)] = I(a.nnz());
    SymbolicMatMut<I> permuted_a{
      from_raw_parts,
      isize(n),
      isize(n),
      a.nnz(),
      _permuted_a_col_ptrs.ptr_mut(),
      nullptr,
      _permuted_a_row_indices.ptr_mut(),
    };
    _detail::symmetric_permute_symbolic(permuted_a, a, perm_inv, stack);
  }

  SymbolicMatRef<I> permuted_a = id_perm ? a
                                         : SymbolicMatRef<I>{
                                             from_raw_parts,
                                             isize(n),
                                             isize(n),
                                             a.nnz(),
                                             _permuted_a_col_ptrs.ptr(),
                                             nullptr,
                                             _permuted_a_row_indices.ptr(),
                                           };

  sparse::etree(etree, permuted_a, stack);

  auto _post = stack.make_new_for_overwrite(tag, isize(n));
  sparse::postorder(_post.ptr_mut(), etree, isize(n), stack);
  sparse::column_counts(nnz_per_col, permuted_a, etree, _post.ptr(), stack);
}

/*!
 * Performs symbolic factorization and computed the column pointers for each
 * column of the cholesky factor of dimension `n`.
 *
 * @param col_ptrs storage for column pointers, of size `n + 1`
 * @param etree storage for elimination tree, of size `n`
 * @param perm_inv storage for inverse permutation, of size `n`
 * @param perm optionally user-provided permutation, either null or of size `n`
 * @param a matrix to be symbolically factorized
 * @param stack temporary allocation stack
 */
template<typename I>
void
factorize_symbolic_col_counts(I* col_ptrs,
                              I* etree,
                              I* perm_inv,
                              I const* perm,
                              SymbolicMatRef<I> a,
                              DynStackMut stack) noexcept
{

  sparse::factorize_symbolic_non_zeros( //
    col_ptrs + 1,
    etree,
    perm_inv,
    perm,
    a,
    stack);

  usize n = usize(a.ncols());
  auto pcol_ptrs = col_ptrs;
  pcol_ptrs[0] = I(0);
  for (usize i = 0; i < n; ++i) {
    pcol_ptrs[i + 1] =
      util::checked_non_negative_plus(pcol_ptrs[i + 1], pcol_ptrs[i]);
  }
}

/*!
 * Computes the stack memory requirements of numerical factorization.
 *
 * @param n dimension of the matrix to be factorized.
 * @param a_nnz number of non zeros of the matrix to be factorized.
 * @param o the kind of permutation that is applied to the matrix before
 * factorization.
 */
template<typename T, typename I>
auto
factorize_numeric_req(proxsuite::linalg::veg::Tag<T> /*ttag*/,
                      proxsuite::linalg::veg::Tag<I> /*itag*/,
                      isize n,
                      isize a_nnz,
                      Ordering o) noexcept
  -> proxsuite::linalg::veg::dynstack::StackReq
{
  using proxsuite::linalg::veg::dynstack::StackReq;

  constexpr isize sz{ sizeof(I) };
  constexpr isize al{ alignof(I) };

  constexpr isize tsz{ sizeof(T) };
  constexpr isize tal{ alignof(T) };

  bool id_perm = o == Ordering::natural;

  auto symb_perm_req = StackReq{ sz * (id_perm ? 0 : (n + 1 + a_nnz)), al };
  auto num_perm_req = StackReq{ tsz * (id_perm ? 0 : a_nnz), tal };
  return num_perm_req                         //
         & (StackReq{ tsz * n, tal }          //
            & (symb_perm_req                  //
               & (StackReq{ 2 * n * sz, al }  //
                  & (StackReq{ n * tsz, tal } //
                     & StackReq{ n * isize{ sizeof(bool) }, alignof(bool) }))));
}

/*!
 * Performs numerical `LDLT` factorization, assuming the symbolic factorization
 * and column counts have already been computed. `L` and `D` are stored in the
 * same matrix, with the elements of `D` replacing the implicit diagonal `1`
 * element of each column of `L`.
 *
 * @param values pointer to the values of the factorization
 * @param row_indices pointer to the row indices of the factorization
 * @param diag_to_add pointer to a vector that is added to the diagonal of the
 * matrix during factorization, if `diag_to_add` and `perm` are both non null
 * @param perm pointer to the pre-computed permutation that is applied to
 * `diag`.
 * @param col_ptrs pointer to the already computed column pointers
 * @param etree pointer to the already computed elimination tree
 * @param perm_inv pointer to the already computed inverse permutation. Must be
 * the inverse of `perm`
 * @param a matrix to be factorized
 * @param stack temporary allocation stack
 */
template<typename T, typename I>
void
factorize_numeric( //
  T* values,
  I* row_indices,
  proxsuite::linalg::veg::DoNotDeduce<T const*> diag_to_add,
  proxsuite::linalg::veg::DoNotDeduce<I const*> perm,
  I const* col_ptrs,
  I const* etree,
  I const* perm_inv,
  MatRef<T, I> a,
  DynStackMut stack) noexcept(false)
{
  using namespace _detail;
  isize n = a.nrows();

  bool id_perm = perm_inv == nullptr;

  proxsuite::linalg::veg::Tag<I> tag{};

  auto _permuted_a_values = stack.make_new_for_overwrite(
    proxsuite::linalg::veg::Tag<T>{}, id_perm ? 0 : a.nnz());

  auto _x = stack.make_new_for_overwrite(proxsuite::linalg::veg::Tag<T>{}, n);

  auto _permuted_a_col_ptrs =
    stack.make_new_for_overwrite(tag, id_perm ? 0 : (a.ncols() + 1));
  auto _permuted_a_row_indices =
    stack.make_new_for_overwrite(tag, id_perm ? 0 : a.nnz());

  if (!id_perm) {
    _permuted_a_col_ptrs.as_mut()[0] = 0;
    _permuted_a_col_ptrs.as_mut()[n] = I(a.nnz());
    MatMut<T, I> permuted_a{
      from_raw_parts,
      n,
      n,
      a.nnz(),
      _permuted_a_col_ptrs.ptr_mut(),
      nullptr,
      _permuted_a_row_indices.ptr_mut(),
      _permuted_a_values.ptr_mut(),
    };
    _detail::symmetric_permute(permuted_a, a, perm_inv, stack);
  }

  MatRef<T, I> permuted_a = id_perm ? a
                                    : MatRef<T, I>{
                                        from_raw_parts,
                                        isize(n),
                                        isize(n),
                                        a.nnz(),
                                        _permuted_a_col_ptrs.ptr(),
                                        nullptr,
                                        _permuted_a_row_indices.ptr(),
                                        _permuted_a_values.ptr(),
                                      };

  auto _current_row_index = stack.make_new_for_overwrite(tag, n);
  auto _ereach_stack_storage = stack.make_new_for_overwrite(tag, n);

  I* pcurrent_row_index = _current_row_index.ptr_mut();
  T* px = _x.ptr_mut();

  std::memcpy( //
    pcurrent_row_index,
    col_ptrs,
    usize(n) * sizeof(I));
  for (usize i = 0; i < usize(n); ++i) {
    px[i] = 0;
  }

  // compute the iter-th row of L using the iter-th column of permuted_a
  // the diagonal element is filled with the diagonal of D instead of 1
  I const* plp = col_ptrs;

  auto _marked = stack.make_new(proxsuite::linalg::veg::Tag<bool>{}, n);
  for (usize iter = 0; iter < usize(n); ++iter) {
    usize ereach_count = 0;
    auto ereach_stack = _detail::ereach(ereach_count,
                                        _ereach_stack_storage.ptr_mut(),
                                        permuted_a.symbolic(),
                                        etree,
                                        isize(iter),
                                        _marked.ptr_mut());

    auto pereach_stack = ereach_stack;

    I const* pai = permuted_a.row_indices();
    T const* pax = permuted_a.values();

    I* pli = row_indices;
    T* plx = values;

    {
      auto col_start = permuted_a.col_start(iter);
      auto col_end = permuted_a.col_end(iter);

      // scatter permuted_a column into x
      // untouched columns are already zeroed

      for (usize p = col_start; p < col_end; ++p) {
        auto i = util::zero_extend(pai[p]);
        px[i] = pax[p];
      }
    }
    T d = px[iter] + ((diag_to_add == nullptr || perm == nullptr)
                        ? T(0)
                        : diag_to_add[util::zero_extend(perm[iter])]);

    // zero for next iteration
    px[iter] = 0;

    for (usize q = 0; q < ereach_count; ++q) {
      usize j = util::zero_extend(pereach_stack[q]);
      auto col_start = util::zero_extend(plp[j]);
      auto row_idx = util::zero_extend(pcurrent_row_index[j]) + 1;

      T const xj = px[j];
      T const dj = plx[col_start];
      T const lkj = xj / dj;

      // zero for the next iteration
      px[j] = 0;

      // skip first element, to put diagonal there later
      for (usize p = col_start + 1; p < row_idx; ++p) {
        auto i = util::zero_extend(pli[p]);
        px[i] -= plx[p] * xj;
      }

      d -= lkj * xj;

      pli[row_idx] = I(iter);
      plx[row_idx] = lkj;
      pcurrent_row_index[j] = I(row_idx);
    }
    {
      auto col_start = util::zero_extend(plp[iter]);
      pli[col_start] = I(iter);
      plx[col_start] = d;
    }
  }
}
} // namespace sparse
} // namespace linalg
} // namespace proxsuite
#endif /* end of include guard PROXSUITE_LINALG_SPARSE_LDLT_FACTORIZE_HPP */
