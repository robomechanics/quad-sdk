#ifndef VEG_META_INT_DYN_HPP_GC385NKBS
#define VEG_META_INT_DYN_HPP_GC385NKBS

#include "proxsuite/linalg/veg/util/assert.hpp"
#include "proxsuite/linalg/veg/internal/fix_index.hpp"
#include "proxsuite/linalg/veg/internal/narrow.hpp"
#include "proxsuite/linalg/veg/util/compare.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"

namespace proxsuite {
namespace linalg {
namespace veg {

template<Ternary T>
struct Boolean;

template<>
struct Boolean<maybe>
{
  using type = maybe_c;

  constexpr Boolean() = default;
  constexpr Boolean /* NOLINT(hicpp-explicit-conversions) */ (bool _val)
    VEG_NOEXCEPT : val{ _val }
  {
  }
  template<Ternary T>
  VEG_INLINE constexpr Boolean /* NOLINT(hicpp-explicit-conversions)
                                */
    (Boolean<T> /*arg*/) VEG_NOEXCEPT : val(T == yes)
  {
  }

  VEG_NODISCARD VEG_INLINE constexpr friend auto operator!(Boolean arg)
    VEG_NOEXCEPT->Boolean
  {
    return { !arg.val };
  }
  VEG_NODISCARD VEG_INLINE explicit constexpr operator bool() const VEG_NOEXCEPT
  {
    return val;
  }

private:
  bool val = false;
};

struct Dyn
{
  constexpr Dyn() = default;
  constexpr Dyn /* NOLINT(hicpp-explicit-conversions) */ (isize val)
    VEG_NOEXCEPT : m_val(val)
  {
  }
  template<isize N>
  constexpr Dyn /* NOLINT(hicpp-explicit-conversions) */ (Fix<N> /*arg*/)
    VEG_NOEXCEPT : m_val(N)
  {
  }

  VEG_NODISCARD VEG_INLINE explicit constexpr operator isize() const
    VEG_NOEXCEPT
  {
    return m_val;
  }
  VEG_NODISCARD VEG_INLINE constexpr auto operator+() const VEG_NOEXCEPT->Dyn
  {
    return *this;
  }
  VEG_NODISCARD VEG_INLINE constexpr auto operator-() const VEG_NOEXCEPT->Dyn
  {
    return Dyn{ -m_val };
  }

#define VEG_OP(Op, Name, TypeName)                                             \
  VEG_TEMPLATE((typename R),                                                   \
               requires(VEG_CONCEPT(index<R>)),                                \
               VEG_NODISCARD VEG_INLINE constexpr auto                         \
               operator Op,                                                    \
               (b, R))                                                         \
  const VEG_NOEXCEPT->typename _detail::binary_traits<Dyn, R>::TypeName        \
  {                                                                            \
    return _detail::binary_traits<Dyn, R>::Name##_fn(*this, b);                \
  }                                                                            \
  VEG_NOM_SEMICOLON

  VEG_OP(+, add, Add);
  VEG_OP(-, sub, Sub);
  VEG_OP(*, mul, Mul);

#undef VEG_OP

  VEG_TEMPLATE(
    (typename R),
    requires(VEG_CONCEPT(index<R>) &&
             VEG_CONCEPT(index<typename _detail::binary_traits<Dyn, R>::Div>)),
    VEG_NODISCARD VEG_INLINE constexpr auto
    operator/,
    (b, R))
  const VEG_NOEXCEPT->typename _detail::binary_traits<Dyn, R>::Div
  {
    return _detail::binary_traits<Dyn, R>::div_fn(*this, b);
  }

  VEG_TEMPLATE(
    (typename R),
    requires(VEG_CONCEPT(index<R>) &&
             VEG_CONCEPT(index<typename _detail::binary_traits<Dyn, R>::Mod>)),
    VEG_NODISCARD VEG_INLINE constexpr auto
    operator%,
    (b, R))
  const VEG_NOEXCEPT->typename _detail::binary_traits<Dyn, R>::Mod
  {
    return _detail::binary_traits<Dyn, R>::mod_fn(*this, b);
  }

#define VEG_CMP(Name, TypeName, Op)                                            \
  VEG_TEMPLATE((typename R),                                                   \
               requires(VEG_CONCEPT(index<R>)),                                \
               VEG_NODISCARD VEG_INLINE constexpr auto                         \
               operator Op, /* NOLINT */                                       \
               (b, R))                                                         \
  const VEG_NOEXCEPT->typename _detail::binary_traits<Dyn, R>::TypeName        \
  {                                                                            \
    return _detail::binary_traits<Dyn, R>::cmp_##Name##_fn(*this, b);          \
  }                                                                            \
  VEG_NOM_SEMICOLON

  VEG_CMP(eq, CmpEq, ==);
  VEG_CMP(neq, CmpNEq, !=);
  VEG_CMP(lt, CmpLT, <);
  VEG_CMP(le, CmpLE, <=);
  VEG_CMP(gt, CmpGT, >);
  VEG_CMP(ge, CmpGE, >=);

#undef VEG_CMP
private:
  isize m_val = 0;
};

template<Ternary T>
VEG_INLINE constexpr Boolean<T>::Boolean(Boolean<maybe> /*b*/,
                                         Unsafe /*tag*/) VEG_NOEXCEPT
{
}
template<Ternary T>
VEG_INLINE constexpr Boolean<T>::Boolean // NOLINT(hicpp-explicit-conversions)
  (Boolean<maybe> b) VEG_NOEXCEPT
  : Boolean(((void)VEG_INTERNAL_ASSERT_PRECONDITION(b.val == (T == yes)), b),
            unsafe)
{
}

template<isize N>
VEG_INLINE constexpr Fix<N>::Fix(Dyn /*arg*/, Unsafe /*tag*/) VEG_NOEXCEPT
{
}
template<isize N>
VEG_INLINE constexpr Fix<N>::Fix // NOLINT(hicpp-explicit-conversions)
  (Dyn arg) VEG_NOEXCEPT
  : Fix((VEG_INTERNAL_ASSERT_PRECONDITION(isize(arg) == N), arg), unsafe)
{
}

namespace _detail {

template<>
struct binary_traits<Dyn, Dyn>
{
#define VEG_OP(Name, TypeName, Op)                                             \
  using TypeName /* NOLINT(bugprone-macro-parentheses) */ = Dyn;               \
  VEG_NODISCARD VEG_INLINE static constexpr auto Name##_fn(Dyn a, Dyn b)       \
    VEG_NOEXCEPT->TypeName                                                     \
  {                                                                            \
    return { isize(usize(isize(a)) Op usize(isize(b))) };                      \
  }                                                                            \
  static_assert(true, "")

#define VEG_CMP(Name, TypeName, Op)                                            \
  using TypeName /* NOLINT(bugprone-macro-parentheses) */ = Boolean<maybe>;    \
  VEG_NODISCARD VEG_INLINE static constexpr auto Name##_fn(Dyn a, Dyn b)       \
    VEG_NOEXCEPT->TypeName                                                     \
  {                                                                            \
    return (isize(a) Op isize(b));                                             \
  }                                                                            \
  static_assert(true, "")

  VEG_OP(add, Add, +);
  VEG_OP(sub, Sub, -);
  VEG_OP(mul, Mul, *);
  VEG_CMP(cmp_eq, CmpEq, ==);
  VEG_CMP(cmp_neq, CmpNEq, !=);
  VEG_CMP(cmp_lt, CmpLT, <);
  VEG_CMP(cmp_le, CmpLE, <=);
  VEG_CMP(cmp_gt, CmpGT, >);
  VEG_CMP(cmp_ge, CmpGE, >=);

  using Div = Dyn;
  using Mod = Dyn;

  VEG_NODISCARD static constexpr auto div_fn(Dyn a, Dyn b) VEG_NOEXCEPT->Div
  {
    return VEG_INTERNAL_ASSERT_PRECONDITION(isize(b) != isize(0)),
           isize(a) / isize(b);
  }
  VEG_NODISCARD static constexpr auto mod_fn(Dyn a, Dyn b) VEG_NOEXCEPT->Mod
  {

    return VEG_INTERNAL_ASSERT_PRECONDITION(isize(b) != isize(0)),
           isize(a) % isize(b);
  }

#undef VEG_OP
#undef VEG_CMP
};

template<isize N>
struct binary_traits<Fix<N>, Dyn> : binary_traits<Dyn, Dyn>
{};

template<>
struct binary_traits<Fix<0>, Dyn> : binary_traits<Dyn, Dyn>
{
  using Mul = Fix<0>;
  VEG_NODISCARD
  constexpr VEG_INLINE static auto mul_fn(Fix<0> /*a*/,
                                          Dyn /*b*/) VEG_NOEXCEPT->Mul
  {
    return {};
  }
};

template<isize N>
struct binary_traits<Dyn, Fix<N>> : binary_traits<Dyn, Dyn>
{
  using Mul = typename binary_traits<Fix<N>, Dyn>::Mul;
  VEG_INLINE static constexpr auto mul_fn(Dyn a, Fix<N> /*b*/) VEG_NOEXCEPT->Mul
  {
    return binary_traits<Fix<N>, Dyn>::mul_fn({}, a);
  }

  using Div = meta::if_t<N == 0, void, Dyn>;
  using Mod = meta::if_t<N == 0, void, Dyn>;

  VEG_NODISCARD VEG_INLINE static constexpr auto div_fn(Dyn a, Fix<N> /*b*/)
    VEG_NOEXCEPT->Div
  {
    return Div(isize(a) / N);
  }
  VEG_NODISCARD VEG_INLINE static constexpr auto mod_fn(Dyn a, Fix<N> /*b*/)
    VEG_NOEXCEPT->Mod
  {
    return Mod(isize(a) % N);
  }
};

} // namespace _detail

inline namespace literals {
VEG_INLINE constexpr auto operator"" _v(unsigned long long n) VEG_NOEXCEPT->Dyn
{
  return isize(n);
}
} // namespace literals

template<>
struct fmt::Debug<Boolean<maybe>>
{
  static void to_string(fmt::Buffer& out, Ref<Boolean<maybe>> val)
  {
    out.insert(out.size(), "maybe[", 6);
    Debug<bool>::to_string(out, ref(bool(val.get())));
    out.insert(out.size(), "]", 1);
  }
};

template<>
struct fmt::Debug<Dyn>
{
  static void to_string(fmt::Buffer& out, Ref<Dyn> val)
  {
    out.insert(out.size(), "Dyn[", 4);
    Debug<isize>::to_string(out, ref(isize(val.get())));
    out.insert(out.size(), "]", 1);
  }
};
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_META_INT_DYN_HPP_GC385NKBS */
