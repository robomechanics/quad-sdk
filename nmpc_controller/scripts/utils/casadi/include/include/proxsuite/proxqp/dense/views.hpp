//
// Copyright (c) 2022 INRIA
//
/**
 * @file views.hpp
 */
#ifndef PROXSUITE_PROXQP_DENSE_VIEWS_HPP
#define PROXSUITE_PROXQP_DENSE_VIEWS_HPP

#include <proxsuite/linalg/veg/type_traits/core.hpp>
#include <proxsuite/linalg/veg/util/dbg.hpp>
#include <cstring>
#include <type_traits>
#include <Eigen/Core>

#define LDLT_CONCEPT(...)                                                      \
  VEG_CONCEPT_MACRO(::proxsuite::proxqp::concepts, __VA_ARGS__)
#define LDLT_CHECK_CONCEPT(...)                                                \
  VEG_CHECK_CONCEPT_MACRO(::proxqp::concepts, __VA_ARGS__)

namespace proxsuite {
namespace proxqp {

using usize = decltype(sizeof(0));
namespace detail {
template<typename Fn>
struct FnInfo;
template<typename Ret_, typename... Args>
struct FnInfo<auto(Args...)->Ret_>
{
  template<usize I>
  using Arg = proxsuite::linalg::veg::ith<I, Args...>;
  using Ret = Ret_;
};
} // namespace detail

#define LDLT_IMPL_GET_PARAM(Fn, Idx)                                           \
  typename ::proxsuite::proxqp::detail::FnInfo<                                \
    decltype Fn /* NOLINT */>::template Arg<(Idx)>,

#define LDLT_IMPL_GET_PARAMS_0(NParams, ...)                                   \
  __VEG_PP_TUPLE_FOR_EACH(LDLT_IMPL_GET_PARAM,                                 \
                          (__VA_ARGS__),                                       \
                          __VEG_PP_MAKE_TUPLE(__VEG_IMPL_PP_DEC(NParams)))

#define LDLT_IMPL_GET_PARAMS_1(NParams, ...)

#define LDLT_IMPL_GET_PARAMS(NParams, ...)                                     \
  __VEG_PP_CAT2(LDLT_IMPL_GET_PARAMS_, __VEG_IMPL_PP_IS_1(NParams))            \
  (NParams, __VA_ARGS__)

#define LDLT_EXPLICIT_TPL_DEF(NParams, ...)                                    \
  template auto __VA_ARGS__(                                                   \
    LDLT_IMPL_GET_PARAMS(NParams, __VA_ARGS__)                                 \
      typename ::proxsuite::proxqp::detail::FnInfo<                            \
        decltype(__VA_ARGS__)>::template Arg<(NParams)-1>)                     \
    ->typename ::proxsuite::proxqp::detail::FnInfo<decltype(__VA_ARGS__)>::Ret
#define LDLT_EXPLICIT_TPL_DECL(NParams, ...)                                   \
  extern LDLT_EXPLICIT_TPL_DEF(NParams, __VA_ARGS__)

using proxsuite::linalg::veg::i32;
using proxsuite::linalg::veg::i64;
using proxsuite::linalg::veg::isize;
using proxsuite::linalg::veg::u32;
using proxsuite::linalg::veg::u64;
using proxsuite::linalg::veg::usize;
using f32 = float;
using f64 = double;
namespace detail {

struct NoCopy
{
  NoCopy() = default;
  ~NoCopy() = default;

  NoCopy(NoCopy const&) = delete;
  NoCopy(NoCopy&&) = delete;
  auto operator=(NoCopy const&) -> NoCopy& = delete;
  auto operator=(NoCopy&&) -> NoCopy& = delete;
};

template<typename Fn>
struct Defer /* NOLINT */
{
  Fn fn;
  NoCopy _;

  VEG_INLINE ~Defer() noexcept(noexcept(VEG_FWD(fn)())) { VEG_FWD(fn)(); }
};

namespace nb {
struct defer
{
  template<typename Fn>
  VEG_INLINE constexpr auto operator()(Fn fn) const -> Defer<Fn>
  {
    return { VEG_FWD(fn), {} };
  }
};
struct max2
{
  template<typename T>
  VEG_INLINE constexpr auto operator()(T const& a, T const& b) const -> T const&
  {
    return a > b ? a : b;
  }
};
struct min2
{
  template<typename T>
  VEG_INLINE constexpr auto operator()(T a, T b) const -> T
  {
    return (a < b) ? a : b;
  }
};

} // namespace nb

template<typename T>
constexpr auto
min_list_impl(T init, T const* arr, usize n) noexcept -> T
{
  return (n == 0)
           ? init
           : nb::min2{}(init, detail::min_list_impl(*arr, arr + 1, n - 1));
}
template<typename T, usize N>
constexpr auto
cx_min_list(T const (&arr)[N]) noexcept -> T
{
  return detail::min_list_impl( //
    arr[0],
    arr + 1,
    N - 1);
}

namespace nb {
struct max_list
{
  template<typename T>
  VEG_INLINE auto operator()(std::initializer_list<T> list) const -> T
  {
    T const* data = list.begin();
    isize len = isize(list.size());

    T current_max = data[0];
    for (isize i = 1; i < len; ++i) {
      if (data[i] > current_max) {
        current_max = data[i];
      }
    }
    return current_max;
  }
};
} // namespace nb
VEG_NIEBLOID(defer);
VEG_NIEBLOID(max2);
VEG_NIEBLOID(min2);
VEG_NIEBLOID(max_list);

template<typename T, bool = std::is_floating_point<T>::value>
struct SetZeroImpl
{
  static void fn(T* dest, usize n)
  {
    for (usize i = 0; i < n; ++i) {
      *dest = 0;
    }
  }
};

template<typename T>
struct SetZeroImpl<T, true>
{
  static void fn(T* dest, usize n)
  {
    // TODO: assert bit representation is zero
    std::memset(dest, 0, n * sizeof(T));
  }
};

template<typename T>
void
set_zero(T* dest, usize n)
{
  SetZeroImpl<T>::fn(dest, n);
}

constexpr auto
round_up(isize n, isize k) noexcept -> isize
{
  return (n + k - 1) / k * k;
}
constexpr auto
uround_up(usize n, usize k) noexcept -> usize
{
  return (n + k - 1) / k * k;
}

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

inline auto
next_aligned(void* ptr, usize align) noexcept -> void*
{
  using BytePtr = unsigned char*;
  using VoidPtr = void*;
  return VoidPtr(BytePtr(ptr) + detail::bytes_to_next_aligned(ptr, align));
}
inline auto
prev_aligned(void* ptr, usize align) noexcept -> void*
{
  using BytePtr = unsigned char*;
  using VoidPtr = void*;
  return VoidPtr(BytePtr(ptr) + detail::bytes_to_prev_aligned(ptr, align));
}

} // namespace detail

enum struct Layout : unsigned char
{
  colmajor = 0,
  rowmajor = 1,
};

constexpr Layout colmajor = Layout::colmajor;
constexpr Layout rowmajor = Layout::rowmajor;

constexpr auto
flip_layout(Layout l) noexcept -> Layout
{
  return Layout(1 - u32(l));
}
constexpr auto
to_eigen_layout(Layout l) -> int
{
  return l == colmajor ? Eigen::ColMajor : Eigen::RowMajor;
}
constexpr auto
from_eigen_layout(int l) -> Layout
{
  return (unsigned(l) & Eigen::RowMajorBit) == Eigen::RowMajor ? rowmajor
                                                               : colmajor;
}

static_assert(to_eigen_layout(from_eigen_layout(Eigen::ColMajor)) ==
                Eigen::ColMajor,
              ".");
static_assert(to_eigen_layout(from_eigen_layout(Eigen::RowMajor)) ==
                Eigen::RowMajor,
              ".");

namespace detail {
template<Layout L>
struct ElementAccess;

template<>
struct ElementAccess<Layout::colmajor>
{
  template<typename T>
  VEG_INLINE static constexpr auto offset(T* ptr,
                                          isize row,
                                          isize col,
                                          isize outer_stride) noexcept -> T*
  {
    return ptr + (usize(row) + usize(col) * usize(outer_stride));
  }

  using NextRowStride = Eigen::Stride<0, 0>;
  using NextColStride = Eigen::InnerStride<Eigen::Dynamic>;
  VEG_INLINE static auto next_row_stride(isize outer_stride) noexcept
    -> NextRowStride
  {
    (void)outer_stride;
    return NextRowStride{};
  }
  VEG_INLINE static auto next_col_stride(isize outer_stride) noexcept
    -> NextColStride
  {
    return NextColStride /* NOLINT(modernize-return-braced-init-list) */ (
      outer_stride);
  }

  template<typename T>
  VEG_INLINE static void transpose_if_rowmajor(T* ptr,
                                               isize dim,
                                               isize outer_stride)
  {
    (void)ptr, (void)dim, (void)outer_stride;
  }
};

template<>
struct ElementAccess<Layout::rowmajor>
{
  template<typename T>
  VEG_INLINE static constexpr auto offset(T* ptr,
                                          isize row,
                                          isize col,
                                          isize outer_stride) noexcept -> T*
  {
    return ptr + (usize(col) + usize(row) * usize(outer_stride));
  }

  using NextColStride = Eigen::Stride<0, 0>;
  using NextRowStride = Eigen::InnerStride<Eigen::Dynamic>;
  VEG_INLINE static auto next_col_stride(isize outer_stride) noexcept
    -> NextColStride
  {
    (void)outer_stride;
    return NextColStride{};
  }
  VEG_INLINE static auto next_row_stride(isize outer_stride) noexcept
    -> NextRowStride
  {
    return NextRowStride /* NOLINT(modernize-return-braced-init-list) */ (
      outer_stride);
  }

  template<typename T>
  VEG_INLINE static void transpose_if_rowmajor(T* ptr,
                                               isize dim,
                                               isize outer_stride)
  {
    Eigen::Map<                          //
      Eigen::Matrix<                     //
        T,                               //
        Eigen::Dynamic,                  //
        Eigen::Dynamic                   //
        >,                               //
      Eigen::Unaligned,                  //
      Eigen::OuterStride<Eigen::Dynamic> //
      >{
      ptr,
      dim,
      dim,
      Eigen::OuterStride<Eigen::Dynamic>(outer_stride),
    }
      .transposeInPlace();
  }
};
} // namespace detail

namespace detail {
template<typename T>
struct unlref
{
  using Type = T;
};
template<typename T>
struct unlref<T&>
{
  using Type = T;
};

template<typename T>
auto
is_eigen_matrix_base_impl(Eigen::MatrixBase<T> const volatile*)
  -> proxsuite::linalg::veg::meta::true_type;
auto
is_eigen_matrix_base_impl(void const volatile*)
  -> proxsuite::linalg::veg::meta::false_type;

template<typename T>
auto
is_eigen_owning_matrix_base_impl(Eigen::PlainObjectBase<T> const volatile*)
  -> proxsuite::linalg::veg::meta::true_type;
auto
is_eigen_owning_matrix_base_impl(void const volatile*)
  -> proxsuite::linalg::veg::meta::false_type;

template<typename... Ts>
using Void = void;

template<typename Mat, typename T>
using DataExpr = decltype(static_cast<T*>(VEG_DECLVAL(Mat&).data()));

template<typename Dummy,
         typename Fallback,
         template<typename...>
         class F,
         typename... Ts>
struct DetectedImpl : proxsuite::linalg::veg::meta::false_type
{
  using Type = Fallback;
};

template<typename Fallback, template<typename...> class F, typename... Ts>
struct DetectedImpl<Void<F<Ts...>>, Fallback, F, Ts...>
  : proxsuite::linalg::veg::meta::true_type
{
  using Type = F<Ts...>;
};

template<typename Fallback, template<typename...> class F, typename... Ts>
using Detected = typename DetectedImpl<void, Fallback, F, Ts...>::Type;

template<typename T>
using CompTimeColsImpl =
  proxsuite::linalg::veg::meta::constant<isize, isize(T::ColsAtCompileTime)>;
template<typename T>
using CompTimeRowsImpl =
  proxsuite::linalg::veg::meta::constant<isize, isize(T::RowsAtCompileTime)>;
template<typename T>
using CompTimeInnerStrideImpl =
  proxsuite::linalg::veg::meta::constant<isize,
                                         isize(T::InnerStrideAtCompileTime)>;
template<typename T>
using LayoutImpl = proxsuite::linalg::veg::meta::
  constant<Layout, (bool(T::IsRowMajor) ? rowmajor : colmajor)>;

template<typename T, Layout L>
using EigenMatMap = Eigen::Map<      //
  Eigen::Matrix<                     //
    T,                               //
    Eigen::Dynamic,                  //
    Eigen::Dynamic,                  //
    (L == colmajor)                  //
      ? Eigen::ColMajor              //
      : Eigen::RowMajor              //
    > const,                         //
  Eigen::Unaligned,                  //
  Eigen::OuterStride<Eigen::Dynamic> //
  >;
template<typename T, Layout L>
using EigenMatMapMut = Eigen::Map<   //
  Eigen::Matrix<                     //
    T,                               //
    Eigen::Dynamic,                  //
    Eigen::Dynamic,                  //
    (L == colmajor)                  //
      ? Eigen::ColMajor              //
      : Eigen::RowMajor              //
    >,                               //
  Eigen::Unaligned,                  //
  Eigen::OuterStride<Eigen::Dynamic> //
  >;

template<typename T, typename Stride>
using EigenVecMap = Eigen::Map< //
  Eigen::Matrix<                //
    T,                          //
    Eigen::Dynamic,             //
    1                           //
    > const,                    //
  Eigen::Unaligned,             //
  Stride                        //
  >;
template<typename T, typename Stride>
using EigenVecMapMut = Eigen::Map< //
  Eigen::Matrix<                   //
    T,                             //
    Eigen::Dynamic,                //
    1                              //
    >,                             //
  Eigen::Unaligned,                //
  Stride                           //
  >;

template<typename T, Layout L>
using ColToVec = EigenVecMap<T, typename ElementAccess<L>::NextRowStride>;
template<typename T, Layout L>
using RowToVec = EigenVecMap<T, typename ElementAccess<L>::NextColStride>;
template<typename T, Layout L>
using ColToVecMut = EigenVecMapMut<T, typename ElementAccess<L>::NextRowStride>;
template<typename T, Layout L>
using RowToVecMut = EigenVecMapMut<T, typename ElementAccess<L>::NextColStride>;

template<typename T>
using VecMap = EigenVecMap<T, Eigen::Stride<0, 0>>;
template<typename T>
using VecMapMut = EigenVecMapMut<T, Eigen::Stride<0, 0>>;

} // namespace detail

template<typename T>
using unref = typename detail::unlref<T&>::Type;

namespace eigen {
template<typename T>
using CompTimeCols =
  detail::Detected<proxsuite::linalg::veg::meta::constant<isize, 0>,
                   detail::CompTimeColsImpl,
                   T>;
template<typename T>
using CompTimeRows =
  detail::Detected<proxsuite::linalg::veg::meta::constant<isize, 0>,
                   detail::CompTimeRowsImpl,
                   T>;
template<typename T>
using CompTimeInnerStride =
  detail::Detected<proxsuite::linalg::veg::meta::constant<isize, 0>,
                   detail::CompTimeInnerStrideImpl,
                   T>;
template<typename T>
using GetLayout =
  detail::Detected<proxsuite::linalg::veg::meta::
                     constant<Layout, Layout(static_cast<unsigned char>(-1))>,
                   detail::LayoutImpl,
                   T>;
} // namespace eigen

namespace concepts {
VEG_DEF_CONCEPT(typename T, rvalue_ref, std::is_rvalue_reference<T>::value);
VEG_DEF_CONCEPT(typename T, lvalue_ref, std::is_lvalue_reference<T>::value);
VEG_DEF_CONCEPT((template<typename...> class F, typename... Ts),
                detected,
                detail::DetectedImpl<void, void, F, Ts...>::value);

namespace aux {
VEG_DEF_CONCEPT((typename Mat, typename T),
                has_data_expr,
                LDLT_CONCEPT(detected<detail::DataExpr, Mat, T>));

VEG_DEF_CONCEPT((typename Mat),
                matrix_base,
                decltype(detail::is_eigen_matrix_base_impl(
                  static_cast<Mat*>(nullptr)))::value);

VEG_DEF_CONCEPT((typename Mat),
                is_plain_object_base,
                decltype(detail::is_eigen_owning_matrix_base_impl(
                  static_cast<Mat*>(nullptr)))::value);

VEG_DEF_CONCEPT((typename Mat),
                tmp_matrix,
                (LDLT_CONCEPT(aux::is_plain_object_base<unref<Mat>>) &&
                 !LDLT_CONCEPT(lvalue_ref<Mat>)));
} // namespace aux

VEG_DEF_CONCEPT((typename Mat, typename T),
                eigen_view,
                (LDLT_CONCEPT(aux::matrix_base<unref<Mat>>) &&
                 LDLT_CONCEPT(aux::has_data_expr<Mat, T const>)));

VEG_DEF_CONCEPT((typename Mat, typename T),
                eigen_view_mut,
                (LDLT_CONCEPT(aux::matrix_base<unref<Mat>>) &&
                 LDLT_CONCEPT(aux::has_data_expr<Mat, T>) &&
                 !LDLT_CONCEPT(aux::tmp_matrix<Mat>)));

VEG_DEF_CONCEPT((typename Mat, typename T),
                eigen_strided_vector_view,
                (LDLT_CONCEPT(eigen_view<Mat, T>) &&
                 (eigen::CompTimeCols<unref<Mat>>::value == 1)));

VEG_DEF_CONCEPT((typename Mat, typename T),
                eigen_strided_vector_view_mut,
                (LDLT_CONCEPT(eigen_view_mut<Mat, T>) &&
                 (eigen::CompTimeCols<unref<Mat>>::value == 1)));

VEG_DEF_CONCEPT((typename Mat, typename T),
                eigen_vector_view,
                (LDLT_CONCEPT(eigen_strided_vector_view<Mat, T>) &&
                 (eigen::CompTimeInnerStride<unref<Mat>>::value == 1)));

VEG_DEF_CONCEPT((typename Mat, typename T),
                eigen_vector_view_mut,
                (LDLT_CONCEPT(eigen_strided_vector_view_mut<Mat, T>) &&
                 (eigen::CompTimeInnerStride<unref<Mat>>::value == 1)));
} // namespace concepts

inline namespace tags {
VEG_TAG(from_ptr_size, FromPtrSize);
VEG_TAG(from_ptr_size_stride, FromPtrSizeStride);
VEG_TAG(from_ptr_rows_cols_stride, FromPtrRowsColsStride);
VEG_TAG(from_eigen, FromEigen);
} // namespace tags

template<typename T>
struct VectorView
{
  T const* data;
  isize dim;

  VEG_INLINE
  VectorView(FromPtrSize /*tag*/, T const* _data, isize _dim) noexcept
    : data(_data)
    , dim(_dim)
  {
  }

  VEG_TEMPLATE(typename Vec,
               requires(LDLT_CONCEPT(eigen_vector_view<Vec, T>)),
               VEG_INLINE VectorView,
               (/*tag*/, FromEigen),
               (vec, Vec const&))
  noexcept
    : data(vec.data())
    , dim(vec.rows())
  {
  }

  VEG_INLINE auto ptr(isize index) const noexcept -> T const*
  {
    return data + index;
  }
  VEG_INLINE auto operator()(isize index) const noexcept -> T const&
  {
    return *ptr(index);
  }
  VEG_INLINE auto segment(isize i, isize size) const noexcept -> VectorView
  {
    return {
      from_ptr_size,
      data + i,
      size,
    };
  }
  VEG_INLINE auto to_eigen() const -> detail::VecMap<T>
  {
    return detail::VecMap<T>(data, Eigen::Index(dim));
  }
};

template<typename T>
struct VectorViewMut
{
  T* data;
  isize dim;

  VEG_INLINE
  VectorViewMut(FromPtrSize /*tag*/, T* _data, isize _dim) noexcept
    : data(_data)
    , dim(_dim)
  {
  }

  VEG_TEMPLATE(typename Vec,
               requires(LDLT_CONCEPT(eigen_vector_view_mut<Vec, T>)),
               VEG_INLINE VectorViewMut,
               (/*tag*/, FromEigen),
               (vec, Vec&&))
  noexcept
    : data(vec.data())
    , dim(vec.rows())
  {
  }

  VEG_INLINE auto as_const() const noexcept -> VectorView<T>
  {
    return {
      from_ptr_size,
      data,
      dim,
    };
  }
  VEG_INLINE auto ptr(isize index) const noexcept -> T* { return data + index; }
  VEG_INLINE auto operator()(isize index) const noexcept -> T&
  {
    return *ptr(index);
  }
  VEG_INLINE auto segment(isize i, isize size) const noexcept -> VectorViewMut
  {
    return {
      from_ptr_size,
      data + i,
      size,
    };
  }
  VEG_INLINE auto to_eigen() const -> detail::VecMapMut<T>
  {
    return detail::VecMapMut<T>(data, Eigen::Index(dim));
  }
};

template<typename T>
struct StridedVectorView
{
  T const* data;
  isize dim;
  isize stride;

  VEG_INLINE
  StridedVectorView(FromPtrSizeStride /*tag*/,
                    T const* _data,
                    isize _dim,
                    isize _stride) noexcept
    : data(_data)
    , dim(_dim)
    , stride(_stride)
  {
  }

  VEG_TEMPLATE(typename Vec,
               requires(LDLT_CONCEPT(eigen_strided_vector_view<Vec, T>)),
               VEG_INLINE StridedVectorView,
               (/*tag*/, FromEigen),
               (vec, Vec const&))
  noexcept
    : data(vec.data())
    , dim(vec.rows())
    , stride(vec.innerStride())
  {
  }

  VEG_INLINE auto ptr(isize index) const noexcept -> T const*
  {
    return data + stride * index;
  }
  VEG_INLINE auto operator()(isize index) const noexcept -> T const&
  {
    return *ptr(index);
  }
  VEG_INLINE auto segment(isize i, isize size) const noexcept
    -> StridedVectorView
  {
    return {
      from_ptr_size_stride,
      data + stride * i,
      size,
      stride,
    };
  }
  VEG_INLINE auto to_eigen() const
    -> detail::EigenVecMap<T, Eigen::InnerStride<Eigen::Dynamic>>
  {
    return detail::EigenVecMap<T, Eigen::InnerStride<Eigen::Dynamic>>(
      data,
      Eigen::Index(dim),
      Eigen::Index(1),
      Eigen::InnerStride<Eigen::Dynamic>(Eigen::Index(stride)));
  }
};

template<typename T>
struct StridedVectorViewMut
{
  T* data;
  isize dim;
  isize stride;

  VEG_INLINE
  StridedVectorViewMut(FromPtrSizeStride /*tag*/,
                       T* _data,
                       isize _dim,
                       isize _stride) noexcept
    : data(_data)
    , dim(_dim)
    , stride(_stride)
  {
  }

  VEG_TEMPLATE(typename Vec,
               requires(LDLT_CONCEPT(eigen_strided_vector_view_mut<Vec, T>)),
               VEG_INLINE StridedVectorViewMut,
               (/*tag*/, FromEigen),
               (vec, Vec&&))
  noexcept
    : data(vec.data())
    , dim(vec.rows())
    , stride(vec.innerStride())
  {
  }

  VEG_INLINE auto as_const() const noexcept -> StridedVectorView<T>
  {
    return {
      from_ptr_size_stride,
      data,
      dim,
      stride,
    };
  }
  VEG_INLINE auto ptr(isize index) const noexcept -> T*
  {
    return data + stride * index;
  }
  VEG_INLINE auto operator()(isize index) const noexcept -> T&
  {
    return *ptr(index);
  }
  VEG_INLINE auto segment(isize i, isize size) const noexcept
    -> StridedVectorViewMut
  {
    return {
      from_ptr_size_stride,
      data + stride * i,
      size,
      stride,
    };
  }
  VEG_INLINE auto to_eigen() const
    -> detail::EigenVecMapMut<T, Eigen::InnerStride<Eigen::Dynamic>>
  {
    return detail::EigenVecMapMut<T, Eigen::InnerStride<Eigen::Dynamic>>(
      data,
      Eigen::Index(dim),
      Eigen::Index(1),
      Eigen::InnerStride<Eigen::Dynamic>(Eigen::Index(stride)));
  }
};

template<typename T, Layout L>
struct MatrixView
{
  T const* data;
  isize rows;
  isize cols;
  isize outer_stride;

  VEG_INLINE MatrixView(FromPtrRowsColsStride /*tag*/,
                        T const* _data,
                        isize _rows,
                        isize _cols,
                        isize _outer_stride) noexcept
    : data(_data)
    , rows(_rows)
    , cols(_cols)
    , outer_stride(_outer_stride)
  {
  }

  VEG_TEMPLATE(typename Mat,
               requires(LDLT_CONCEPT(eigen_view<Mat, T>) &&
                        eigen::GetLayout<unref<Mat>>::value == L),
               VEG_INLINE MatrixView,
               (/*tag*/, FromEigen),
               (mat, Mat const&))
  noexcept
    : data(mat.data())
    , rows(mat.rows())
    , cols(mat.cols())
    , outer_stride(mat.outerStride())
  {
  }

  VEG_INLINE auto ptr(isize row, isize col) const noexcept -> T const*
  {
    return detail::ElementAccess<L>::offset(data, row, col, outer_stride);
  }
  VEG_INLINE auto operator()(isize row, isize col) const noexcept -> T const&
  {
    return *ptr(row, col);
  }
  VEG_INLINE auto block(isize row,
                        isize col,
                        isize nrows,
                        isize ncols) const noexcept -> MatrixView
  {
    return {
      from_ptr_rows_cols_stride,
      detail::ElementAccess<L>::offset(data, row, col, outer_stride),
      nrows,
      ncols,
      outer_stride,
    };
  }

private:
  VEG_INLINE auto col_impl(
    proxsuite::linalg::veg::meta::constant<Layout, colmajor> /*tag*/,
    isize c) const noexcept -> VectorView<T>
  {
    return {
      from_ptr_size,
      data + c * outer_stride,
      rows,
    };
  }
  VEG_INLINE auto col_impl(
    proxsuite::linalg::veg::meta::constant<Layout, rowmajor> /*tag*/,
    isize c) const noexcept -> StridedVectorView<T>
  {
    return {
      from_ptr_size_stride,
      data + c,
      rows,
      outer_stride,
    };
  }

public:
  VEG_INLINE auto col(isize c) const noexcept -> proxsuite::linalg::veg::meta::
    if_t<(L == colmajor), VectorView<T>, StridedVectorView<T>>
  {
    return col_impl(proxsuite::linalg::veg::meta::constant<Layout, L>{}, c);
  }
  VEG_INLINE auto row(isize r) const noexcept -> proxsuite::linalg::veg::meta::
    if_t<(L == rowmajor), VectorView<T>, StridedVectorView<T>>
  {
    return trans().col(r);
  }
  VEG_INLINE auto trans() const noexcept
    -> MatrixView<T, proxqp::flip_layout(L)>
  {
    return {
      from_ptr_rows_cols_stride, data, cols, rows, outer_stride,
    };
  }
  VEG_INLINE auto to_eigen() const noexcept -> detail::EigenMatMap<T, L>
  {
    return detail::EigenMatMap<T, L>(
      data,
      Eigen::Index(rows),
      Eigen::Index(cols),
      Eigen::OuterStride<Eigen::Dynamic>(Eigen::Index(outer_stride)));
  }
};

template<typename T, Layout L>
struct MatrixViewMut
{
  T* data;
  isize rows;
  isize cols;
  isize outer_stride;

  VEG_INLINE MatrixViewMut(FromPtrRowsColsStride /*tag*/,
                           T* _data,
                           isize _rows,
                           isize _cols,
                           isize _outer_stride) noexcept
    : data(_data)
    , rows(_rows)
    , cols(_cols)
    , outer_stride(_outer_stride)
  {
  }

  VEG_TEMPLATE(typename Mat,
               requires(LDLT_CONCEPT(eigen_view<Mat, T>) &&
                        eigen::GetLayout<unref<Mat>>::value == L),
               VEG_INLINE MatrixViewMut,
               (/*tag*/, FromEigen),
               (mat, Mat&&))
  noexcept
    : data(mat.data())
    , rows(mat.rows())
    , cols(mat.cols())
    , outer_stride(mat.outerStride())
  {
  }

  VEG_INLINE auto ptr(isize row, isize col) const noexcept -> T*
  {
    return detail::ElementAccess<L>::offset(data, row, col, outer_stride);
  }
  VEG_INLINE auto operator()(isize row, isize col) const noexcept -> T&
  {
    return *ptr(row, col);
  }
  VEG_INLINE auto block(isize row,
                        isize col,
                        isize nrows,
                        isize ncols) const noexcept -> MatrixViewMut
  {
    return {
      from_ptr_rows_cols_stride,
      detail::ElementAccess<L>::offset(data, row, col, outer_stride),
      nrows,
      ncols,
      outer_stride,
    };
  }

private:
  VEG_INLINE auto col_impl(
    proxsuite::linalg::veg::meta::constant<Layout, colmajor> /*tag*/,
    isize c) const noexcept -> VectorViewMut<T>
  {
    return {
      from_ptr_size,
      data + c * outer_stride,
      rows,
    };
  }
  VEG_INLINE auto col_impl(
    proxsuite::linalg::veg::meta::constant<Layout, rowmajor> /*tag*/,
    isize c) const noexcept -> StridedVectorViewMut<T>
  {
    return {
      from_ptr_size_stride,
      data + c,
      rows,
      outer_stride,
    };
  }

public:
  VEG_INLINE auto col(isize c) const noexcept -> proxsuite::linalg::veg::meta::
    if_t<(L == colmajor), VectorViewMut<T>, StridedVectorViewMut<T>>
  {
    return col_impl(proxsuite::linalg::veg::meta::constant<Layout, L>{}, c);
  }
  VEG_INLINE auto row(isize r) const noexcept -> proxsuite::linalg::veg::meta::
    if_t<(L == rowmajor), VectorViewMut<T>, StridedVectorViewMut<T>>
  {
    return trans().col(r);
  }
  VEG_INLINE auto trans() const noexcept
    -> MatrixViewMut<T, proxqp::flip_layout(L)>
  {
    return {
      from_ptr_rows_cols_stride, data, cols, rows, outer_stride,
    };
  }
  VEG_INLINE auto to_eigen() const noexcept -> detail::EigenMatMapMut<T, L>
  {
    return detail::EigenMatMapMut<T, L>(
      data,
      Eigen::Index(rows),
      Eigen::Index(cols),
      Eigen::OuterStride<Eigen::Dynamic>(Eigen::Index(outer_stride)));
  }
  VEG_INLINE auto as_const() const noexcept -> MatrixView<T, L>
  {
    return {
      from_ptr_rows_cols_stride, data, rows, cols, outer_stride,
    };
  }
};

template<typename T>
struct LdltView
{
private:
  MatrixView<T, colmajor> ld;

public:
  explicit LdltView(MatrixView<T, colmajor> ld) noexcept
    : ld(ld)
  {
    VEG_DEBUG_ASSERT(ld.rows == ld.cols);
  }

  VEG_INLINE auto l() const noexcept -> MatrixView<T, colmajor> { return ld; }
  VEG_INLINE auto d() const noexcept -> StridedVectorView<T>
  {
    return { from_ptr_size_stride, ld.data, ld.rows, ld.outer_stride + 1 };
  }

  VEG_INLINE auto head(isize k) const -> LdltView
  {
    return LdltView{ ld.block(0, 0, k, k) };
  }
  VEG_INLINE auto tail(isize k) const -> LdltView
  {
    isize n = ld.rows;
    return LdltView{ ld.block(n - k, n - k, k, k) };
  }
};
template<typename T>
struct LdltViewMut
{
private:
  MatrixViewMut<T, colmajor> ld;

public:
  explicit LdltViewMut(MatrixViewMut<T, colmajor> ld) noexcept
    : ld(ld)
  {
    VEG_DEBUG_ASSERT(ld.rows == ld.cols);
  }

  VEG_INLINE auto l() const noexcept -> MatrixView<T, colmajor>
  {
    return ld.as_const();
  }
  VEG_INLINE auto l_mut() const noexcept -> MatrixViewMut<T, colmajor>
  {
    return ld;
  }
  VEG_INLINE auto d() const noexcept -> StridedVectorView<T>
  {
    return { from_ptr_size_stride, ld.data, ld.rows, ld.outer_stride + 1 };
  }
  VEG_INLINE auto d_mut() const noexcept -> StridedVectorViewMut<T>
  {
    return { from_ptr_size_stride, ld.data, ld.rows, ld.outer_stride + 1 };
  }

  VEG_INLINE auto as_const() const noexcept -> LdltView<T>
  {
    return LdltView<T>{ ld.as_const() };
  }

  VEG_INLINE auto head(isize k) const -> LdltViewMut
  {
    return LdltViewMut{ ld.block(0, 0, k, k) };
  }
  VEG_INLINE auto tail(isize k) const -> LdltViewMut
  {
    isize n = ld.rows;
    return LdltViewMut{ ld.block(n - k, n - k, k, k) };
  }
};

namespace detail {
template<typename T>
void
noalias_mul_add(MatrixViewMut<T, colmajor> dst,
                MatrixView<T, colmajor> lhs,
                MatrixView<T, colmajor> rhs,
                T factor)
{

  if ((dst.cols == 0) || (dst.rows == 0) || (lhs.cols == 0)) {
    return;
  }

#if !EIGEN_VERSION_AT_LEAST(3, 3, 8)
#define LAZY_PRODUCT(a, b) a.lazyProduct(b)
#else
#define LAZY_PRODUCT(a, b) a.operator*(b)
#endif

  if (dst.cols == 1 && dst.rows == 1) {
    // dot
    auto rhs_col = rhs.col(0);
    auto lhs_row = lhs.row(0);
    auto lhs_as_col = lhs.col(0);
    lhs_as_col.dim = lhs_row.dim;
    if (lhs_row.stride == 1) {
      dst(0, 0) += factor * lhs_as_col.to_eigen().dot(rhs_col.to_eigen());
    } else {
      dst(0, 0) += factor * lhs_row.to_eigen().dot(rhs_col.to_eigen());
    }
  } else if (dst.cols == 1) {
    // gemv
    auto rhs_col = rhs.col(0);
    auto dst_col = dst.col(0);
    dst_col.to_eigen().noalias().operator+=(
      factor * LAZY_PRODUCT(lhs.to_eigen(), rhs_col.to_eigen()));
  }

#if !EIGEN_VERSION_AT_LEAST(3, 3, 8)
  else if ((dst.rows < 20) && (dst.cols < 20) && (rhs.rows < 20)) {
    // gemm
    // workaround for eigen 3.3.7 bug:
    // https://gitlab.com/libeigen/eigen/-/issues/1562
    using Stride = Eigen::OuterStride<Eigen::Dynamic>;
    using Mat =
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 20, 20>;
    using MapMut = Eigen::Map<Mat, Eigen::Unaligned, Stride>;
    using Map = Eigen::Map<Mat const, Eigen::Unaligned, Stride>;

    MapMut(dst.data, dst.rows, dst.cols, Stride(dst.outer_stride))
      .noalias()
      .
      operator+=(
        factor *
        LAZY_PRODUCT(
          Map(lhs.data, lhs.rows, lhs.cols, Stride(lhs.outer_stride)),
          Map(rhs.data, rhs.rows, rhs.cols, Stride(rhs.outer_stride))));
  }
#endif

  else {
    // gemm
    dst.to_eigen().noalias().operator+=(
      factor * LAZY_PRODUCT(lhs.to_eigen(), rhs.to_eigen()));
  }

#undef LAZY_PRODUCT
}

template<typename T>
void
noalias_mul_add_vec(VectorViewMut<T> dst,
                    MatrixView<T, colmajor> lhs,
                    VectorView<T> rhs,
                    T factor)
{
  detail::noalias_mul_add<T>(
    {
      from_ptr_rows_cols_stride,
      dst.data,
      dst.dim,
      1,
      0,
    },
    lhs,
    {
      from_ptr_rows_cols_stride,
      rhs.data,
      rhs.dim,
      1,
      0,
    },
    VEG_FWD(factor));
}

template<typename T>
auto
dot(StridedVectorView<T> lhs, VectorView<T> rhs) -> T
{
  auto out = T(0);
  detail::noalias_mul_add<T>(
    {
      from_ptr_rows_cols_stride,
      std::addressof(out),
      1,
      1,
      0,
    },
    {
      from_ptr_rows_cols_stride,
      lhs.data,
      1,
      lhs.dim,
      lhs.stride,
    },
    {
      from_ptr_rows_cols_stride,
      rhs.data,
      rhs.dim,
      1,
      0,
    },
    1);
  return out;
}
template<typename T>
void
assign_cwise_prod(VectorViewMut<T> out,
                  StridedVectorView<T> lhs,
                  StridedVectorView<T> rhs)
{
  out.to_eigen() = lhs.to_eigen().cwiseProduct(rhs.to_eigen());
}
template<typename T>
void
assign_scalar_prod(VectorViewMut<T> out, T factor, VectorView<T> in)
{
  out.to_eigen() = in.to_eigen().operator*(factor);
}

template<typename T>
void
trans_tr_unit_up_solve_in_place_on_right(MatrixView<T, colmajor> tr,
                                         MatrixViewMut<T, colmajor> rhs)
{
  if (rhs.cols == 1) {
    tr.to_eigen()
      .transpose()
      .template triangularView<Eigen::UnitUpper>()
      .template solveInPlace<Eigen::OnTheRight>(rhs.col(0).to_eigen());
  } else {
    tr.to_eigen()
      .transpose()
      .template triangularView<Eigen::UnitUpper>()
      .template solveInPlace<Eigen::OnTheRight>(rhs.to_eigen());
  }
}

template<typename T>
void
apply_diag_inv_on_right(MatrixViewMut<T, colmajor> out,
                        StridedVectorView<T> d,
                        MatrixView<T, colmajor> in)
{
  if (out.cols == 1) {
    out.col(0).to_eigen() =
      in.col(0).to_eigen().operator*(d.to_eigen().asDiagonal().inverse());
  } else {
    out.to_eigen() =
      in.to_eigen().operator*(d.to_eigen().asDiagonal().inverse());
  }
}
template<typename T>
void
apply_diag_on_right(MatrixViewMut<T, colmajor> out,
                    StridedVectorView<T> d,
                    MatrixView<T, colmajor> in)
{
  if (out.cols == 1) {
    out.col(0).to_eigen() =
      in.col(0).to_eigen().operator*(d.to_eigen().asDiagonal());
  } else {
    out.to_eigen() = in.to_eigen().operator*(d.to_eigen().asDiagonal());
  }
}

template<typename T>
void
noalias_mul_sub_tr_lo(MatrixViewMut<T, colmajor> out,
                      MatrixView<T, colmajor> lhs,
                      MatrixView<T, rowmajor> rhs)
{
  if (lhs.cols == 1) {
    out.to_eigen().template triangularView<Eigen::Lower>().operator-=(
      lhs.col(0).to_eigen().operator*(
        Eigen::Map<Eigen::Matrix<T, 1, Eigen::Dynamic> const>(
          rhs.data, 1, rhs.cols)));
  } else {
    out.to_eigen().template triangularView<Eigen::Lower>().operator-=(
      lhs.to_eigen().operator*(rhs.to_eigen()));
  }
}

} // namespace detail
} // namespace proxqp
} // namespace proxsuite

namespace proxsuite {
namespace proxqp {

namespace dense {

struct EigenAllowAlloc
{
  bool alloc_was_allowed;
  EigenAllowAlloc(EigenAllowAlloc&&) = delete;
  EigenAllowAlloc(EigenAllowAlloc const&) = delete;
  auto operator=(EigenAllowAlloc&&) -> EigenAllowAlloc& = delete;
  auto operator=(EigenAllowAlloc const&) -> EigenAllowAlloc& = delete;

#if defined(EIGEN_RUNTIME_NO_MALLOC)
  EigenAllowAlloc() noexcept
    : alloc_was_allowed(Eigen::internal::is_malloc_allowed())
  {
    Eigen::internal::set_is_malloc_allowed(true);
  }
  ~EigenAllowAlloc() noexcept
  {
    Eigen::internal::set_is_malloc_allowed(alloc_was_allowed);
  }
#else
  EigenAllowAlloc() = default;
#endif
};

template<typename T>
struct QpView
{

  static constexpr Layout layout = rowmajor;

  MatrixView<T, layout> H;
  VectorView<T> g;

  MatrixView<T, layout> A;
  VectorView<T> b;
  MatrixView<T, layout> C;
  VectorView<T> d;
};

template<typename Scalar>
struct QpViewBox
{
  static constexpr Layout layout = rowmajor;

  MatrixView<Scalar, layout> H;
  VectorView<Scalar> g;

  MatrixView<Scalar, layout> A;
  VectorView<Scalar> b;
  MatrixView<Scalar, layout> C;
  VectorView<Scalar> u;
  VectorView<Scalar> l;
};

template<typename T>
struct QpViewMut
{
  static constexpr Layout layout = rowmajor;

  MatrixViewMut<T, layout> H;
  VectorViewMut<T> g;

  MatrixViewMut<T, layout> A;
  VectorViewMut<T> b;
  MatrixViewMut<T, layout> C;
  VectorViewMut<T> d;

  VEG_INLINE constexpr auto as_const() const noexcept -> QpView<T>
  {
    return {
      H.as_const(), g.as_const(), A.as_const(),
      b.as_const(), C.as_const(), d.as_const(),
    };
  }
};

template<typename Scalar>
struct QpViewBoxMut
{
  static constexpr Layout layout = rowmajor;

  MatrixViewMut<Scalar, layout> H;
  VectorViewMut<Scalar> g;

  MatrixViewMut<Scalar, layout> A;
  VectorViewMut<Scalar> b;
  MatrixViewMut<Scalar, layout> C;
  VectorViewMut<Scalar> u;
  VectorViewMut<Scalar> l;

  VEG_INLINE constexpr auto as_const() const noexcept -> QpViewBox<Scalar>
  {
    return {
      H.as_const(), g.as_const(), A.as_const(), b.as_const(),
      C.as_const(), u.as_const(), l.as_const(),
    };
  }
};

namespace nb {
struct pow
{
  template<typename T>
  auto operator()(T x, T y) const -> T
  {
    using std::pow;
    return pow(x, y);
  }
};
struct infty_norm
{
  template<typename D>
  auto operator()(Eigen::MatrixBase<D> const& mat) const -> typename D::Scalar
  {
    if (mat.rows() == 0 || mat.cols() == 0) {
      return typename D::Scalar(0);
    } else {
      return mat.template lpNorm<Eigen::Infinity>();
    }
  }
};
struct sqrt
{
  template<typename T>
  auto operator()(T x) const -> T
  {
    using std::sqrt;
    return sqrt(x);
  }
};
struct fabs
{
  template<typename T>
  auto operator()(T x) const -> T
  {
    using std::fabs;
    return fabs(x);
  }
};
} // namespace nb
VEG_NIEBLOID(fabs);
VEG_NIEBLOID(sqrt);
VEG_NIEBLOID(pow);
VEG_NIEBLOID(infty_norm);
} // namespace dense
} // namespace proxqp
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_PROXQP_DENSE_VIEWS_HPP */
