/** \file */
//
// Copyright (c) 2022 INRIA
//
#ifndef PROXSUITE_LINALG_DENSE_LDLT_UPDATE_HPP
#define PROXSUITE_LINALG_DENSE_LDLT_UPDATE_HPP

#include "proxsuite/linalg/dense/core.hpp"

namespace proxsuite {
namespace linalg {
namespace dense {
namespace _detail {
inline auto
bytes_to_prev_aligned(void* ptr, usize align) noexcept -> isize
{
  using UPtr = std::uintptr_t;

  UPtr mask = align - 1;
  UPtr iptr = UPtr(ptr);
  UPtr aligned_ptr = iptr & ~mask;
  return isize(aligned_ptr - iptr);
}
inline auto
bytes_to_next_aligned(void* ptr, usize align) noexcept -> isize
{
  using UPtr = std::uintptr_t;

  UPtr mask = align - 1;
  UPtr iptr = UPtr(ptr);
  UPtr aligned_ptr = (iptr + mask) & ~mask;
  return isize(aligned_ptr - iptr);
}

template<usize... Is, typename Fn>
VEG_INLINE void
unroll_impl(proxsuite::linalg::veg::meta::index_sequence<Is...> /*unused*/,
            Fn fn)
{
  VEG_EVAL_ALL(fn(Is));
}

template<usize N, typename Fn>
VEG_INLINE void
unroll(Fn fn)
{
  _detail::unroll_impl(proxsuite::linalg::veg::meta::make_index_sequence<N>{},
                       VEG_FWD(fn));
}

template<typename T, usize N>
struct RankUpdateLoadW
{
  _simd::Pack<T, N>* p_wr;
  T const* pw;
  isize w_stride;

  VEG_INLINE void operator()(usize i) const
  {
    p_wr[i] = _simd::Pack<T, N>::load_unaligned(pw + w_stride * isize(i));
  }
};

template<typename T, usize N>
struct RankUpdateUpdateWAndL
{
  _simd::Pack<T, N>* p_wr;
  _simd::Pack<T, N>& p_in_l;
  _simd::Pack<T, N> const* p_p;
  _simd::Pack<T, N> const* p_mu;

  VEG_INLINE void operator()(usize i) const
  {
    p_wr[i] = _simd::Pack<T, N>::fnmadd(p_p[i], p_in_l, p_wr[i]);
    p_in_l = _simd::Pack<T, N>::fmadd(p_mu[i], p_wr[i], p_in_l);
  }
};

template<typename T, usize N>
struct RankUpdateStoreW
{
  _simd::Pack<T, N> const* p_wr;
  T* pw;
  isize w_stride;

  VEG_INLINE void operator()(usize i) const
  {
    p_wr[i].store_unaligned(pw + w_stride * isize(i));
  }
};

template<usize R, typename T, usize N>
VEG_INLINE void
rank_r_update_inner_loop_iter( //
  _simd::Pack<T, N> const* p_p,
  _simd::Pack<T, N> const* p_mu,
  T* inout_l,
  T* pw,
  isize w_stride)
{

  _simd::Pack<T, N> p_wr[R];
  _detail::unroll<R>(RankUpdateLoadW<T, N>{ p_wr, pw, w_stride });
  _simd::Pack<T, N> p_in_l = _simd::Pack<T, N>::load_unaligned(inout_l);
  _detail::unroll<R>(RankUpdateUpdateWAndL<T, N>{ p_wr, p_in_l, p_p, p_mu });
  _detail::unroll<R>(RankUpdateStoreW<T, N>{ p_wr, pw, w_stride });

  p_in_l.store_unaligned(inout_l);
}

template<bool VECTORIZABLE>
struct RankRUpdateLoopImpl;

template<typename T, usize N>
struct RankUpdateLoadPMu
{
  _simd::Pack<T, N>* p_p;
  _simd::Pack<T, N>* p_mu;
  T const* p;
  T const* mu;
  VEG_INLINE void operator()(usize i) const
  {
    p_p[i] = _simd::Pack<T, N>::broadcast(p[i]);
    p_mu[i] = _simd::Pack<T, N>::broadcast(mu[i]);
  }
};

template<>
struct RankRUpdateLoopImpl<false>
{
  template<usize R, typename T>
  VEG_INLINE static void fn(isize n,
                            T* inout_l,
                            T* pw,
                            isize w_stride,
                            T const* p,
                            T const* mu) noexcept
  {
    using Pack_ = _simd::Pack<T, 1>;
    Pack_ p_p[R];
    Pack_ p_mu[R];

    _detail::unroll<R>(RankUpdateLoadPMu<T, 1>{ p_p, p_mu, p, mu });

    auto inout_l_finish = inout_l + n;
    while (inout_l < inout_l_finish) {
      _detail::rank_r_update_inner_loop_iter<R>(
        p_p, p_mu, inout_l, pw, w_stride);
      ++inout_l;
      ++pw;
    }
  }
};

template<>
struct RankRUpdateLoopImpl<true>
{
  template<usize R, typename T>
  VEG_INLINE static void fn(isize n,
                            T* inout_l,
                            T* pw,
                            isize w_stride,
                            T const* p,
                            T const* mu) noexcept
  {

    // best perf if beginning of each pw is aligned
    // should be enforced by the Ldlt class

    using Info = _simd::NativePackInfo<T>;
    constexpr usize N = Info::N;
    auto inout_l_vectorized_end = inout_l + usize(n) / N * N;
    auto inout_l_end = inout_l + usize(n);

    {
      using Pack = _simd::NativePack<T>;
      Pack p_p[R];
      Pack p_mu[R];

      _detail::unroll<R>(RankUpdateLoadPMu<T, N>{ p_p, p_mu, p, mu });

      while (inout_l < inout_l_vectorized_end) {
        _detail::rank_r_update_inner_loop_iter<R>(
          p_p, p_mu, inout_l, pw, w_stride);
        inout_l += N;
        pw += N;
      }
    }
    {
      using Pack_ = _simd::Pack<T, 1>;
      Pack_ p_p[R];
      Pack_ p_mu[R];

      _detail::unroll<R>(RankUpdateLoadPMu<T, 1>{ p_p, p_mu, p, mu });

      while (inout_l < inout_l_end) {
        _detail::rank_r_update_inner_loop_iter<R>(
          p_p, p_mu, inout_l, pw, w_stride);
        ++inout_l;
        ++pw;
      }
    }
  }
};

template<usize R, typename T>
VEG_INLINE void
rank_r_update_inner_loop(isize n,
                         T* inout_l,
                         T* pw,
                         isize w_stride,
                         T const* p,
                         T const* mu)
{
  RankRUpdateLoopImpl<should_vectorize<T>::value>::template fn<R>(
    n, inout_l, pw, w_stride, p, mu);
}

template<typename LD, typename T, typename Fn>
void
rank_r_update_clobber_w_impl( //
  LD ld,
  T* pw,
  isize w_stride,
  T* palpha,
  Fn r_fn)
{
  static_assert(LD::InnerStrideAtCompileTime == 1, ".");
  static_assert(!bool(LD::IsRowMajor), ".");

  isize n = ld.rows();

  for (isize j = 0; j < n; ++j) {
    isize r = r_fn();

    isize r_done = 0;
    if (!(r_done < r)) {
      continue;
    }

    while (true) {
      isize r_chunk = min2(isize(4), r - r_done);

      T p_array[4];
      T mu_array[4];

      T dj = ld(j, j);
      for (isize k = 0; k < r_chunk; ++k) {
        auto& p = (+p_array)[k];
        auto& mu = (+mu_array)[k];
        auto& alpha = palpha[r_done + k];

        p = pw[(r_done + k) * w_stride];
        T new_dj = dj + (alpha * p) * p;
        mu = (alpha * p) / new_dj;
        alpha -= new_dj * (mu * mu);

        dj = new_dj;
      }
      ld(j, j) = dj;

      isize rem = n - j - 1;

      using FnType = void (*)(isize, T*, T*, isize, T const*, T const*);
      FnType fn_table[] = {
        rank_r_update_inner_loop<1, T>,
        rank_r_update_inner_loop<2, T>,
        rank_r_update_inner_loop<3, T>,
        rank_r_update_inner_loop<4, T>,
      };

      (*fn_table[r_chunk - 1])( //
        rem,
        util::matrix_elem_addr(ld, j + 1, j),
        pw + 1 + r_done * w_stride,
        w_stride,
        p_array,
        mu_array);

      r_done += r_chunk;
      if (!(r_done < r)) {
        break;
      }
    }
    ++pw;
  }
}
struct ConstantR
{
  isize r;
  VEG_INLINE auto operator()() const noexcept -> isize { return r; }
};
} // namespace _detail

template<typename LD,
         typename W,
         typename T = typename proxsuite::linalg::veg::uncvref_t<LD>::Scalar>
void
rank_1_update_clobber_w(LD&& ld,
                        W&& w,
                        proxsuite::linalg::veg::DoNotDeduce<T> alpha)
{
  _detail::rank_r_update_clobber_w_impl( //
    util::to_view_dyn(ld),
    w.data(),
    0,
    proxsuite::linalg::veg::mem::addressof(alpha),
    _detail::ConstantR{ 1 });
}

template<typename LD,
         typename W,
         typename A,
         typename T = typename proxsuite::linalg::veg::uncvref_t<LD>::Scalar>
void
rank_r_update_clobber_inputs(LD&& ld, W&& w, A&& alpha)
{
  isize r = w.cols();
  _detail::rank_r_update_clobber_w_impl( //
    util::to_view_dyn(ld),
    w.data(),
    w.outerStride(),
    alpha.data(),
    _detail::ConstantR{ r });
}
} // namespace dense
} // namespace linalg
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_LINALG_DENSE_LDLT_UPDATE_HPP */
