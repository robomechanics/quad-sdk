#ifndef VEG_MACROS_HPP_HSTLSKZXS
#define VEG_MACROS_HPP_HSTLSKZXS
#include "proxsuite/linalg/veg/internal/external/hedley.ext.hpp"
#include "proxsuite/linalg/veg/internal/typedefs.hpp"
#include "proxsuite/linalg/veg/internal/preprocessor.hpp"
#include "proxsuite/linalg/veg/internal/prologue.hpp"
#include <initializer_list>

////////////////////////////////////////////////////////////////////////////////

#if _MSC_VER
#define PROXSUITE_PRETTY_FUNCTION __FUNCSIG__
#else
#define PROXSUITE_PRETTY_FUNCTION __PRETTY_FUNCTION__
#endif

#define PROXSUITE_THROW_PRETTY(condition, exception, message)                  \
  if (condition) {                                                             \
    std::ostringstream ss;                                                     \
    ss << "From file: " << __FILE__ << "\n";                                   \
    ss << "in function: " << PROXSUITE_PRETTY_FUNCTION << "\n";                \
    ss << "at line: " << __LINE__ << "\n";                                     \
    ss << message << "\n";                                                     \
    throw exception(ss.str());                                                 \
  }

#define PROXSUITE_CHECK_ARGUMENT_SIZE(size, expected_size, message)            \
  if (size != expected_size) {                                                 \
    std::ostringstream oss;                                                    \
    oss << "wrong argument size: expected " << expected_size << ", got "       \
        << size << "\n";                                                       \
    oss << "hint: " << message << std::endl;                                   \
    PROXSUITE_THROW_PRETTY(true, std::invalid_argument, oss.str());            \
  }

#if HEDLEY_MSVC_VERSION_CHECK(14, 0, 0) ||                                     \
  HEDLEY_INTEL_CL_VERSION_CHECK(2021, 1, 0)
#define VEG_DEPRECATED(Reason) __declspec(deprecated(Reason))
#elif HEDLEY_HAS_EXTENSION(attribute_deprecated_with_message) ||               \
  HEDLEY_GCC_VERSION_CHECK(4, 5, 0) || HEDLEY_INTEL_VERSION_CHECK(13, 0, 0) || \
  HEDLEY_ARM_VERSION_CHECK(5, 6, 0) ||                                         \
  HEDLEY_SUNPRO_VERSION_CHECK(5, 13, 0) ||                                     \
  HEDLEY_PGI_VERSION_CHECK(17, 10, 0) || HEDLEY_TI_VERSION_CHECK(18, 1, 0) ||  \
  HEDLEY_TI_ARMCL_VERSION_CHECK(18, 1, 0) ||                                   \
  HEDLEY_TI_CL6X_VERSION_CHECK(8, 3, 0) ||                                     \
  HEDLEY_TI_CL7X_VERSION_CHECK(1, 2, 0) ||                                     \
  HEDLEY_TI_CLPRU_VERSION_CHECK(2, 3, 0)
#define VEG_DEPRECATED(Reason) __attribute__((__deprecated__(Reason)))
#elif defined(VEG_WITH_CXX14_SUPPORT)
#define VEG_DEPRECATED(Reason)                                                 \
  HEDLEY_DIAGNOSTIC_DISABLE_CPP98_COMPAT_WRAP_([[deprecated(Reason)]])
#elif HEDLEY_HAS_ATTRIBUTE(deprecated) || HEDLEY_GCC_VERSION_CHECK(3, 1, 0) || \
  HEDLEY_ARM_VERSION_CHECK(4, 1, 0) || HEDLEY_TI_VERSION_CHECK(15, 12, 0) ||   \
  (HEDLEY_TI_ARMCL_VERSION_CHECK(4, 8, 0) &&                                   \
   defined(__TI_GNU_ATTRIBUTE_SUPPORT__)) ||                                   \
  HEDLEY_TI_ARMCL_VERSION_CHECK(5, 2, 0) ||                                    \
  (HEDLEY_TI_CL2000_VERSION_CHECK(6, 0, 0) &&                                  \
   defined(__TI_GNU_ATTRIBUTE_SUPPORT__)) ||                                   \
  HEDLEY_TI_CL2000_VERSION_CHECK(6, 4, 0) ||                                   \
  (HEDLEY_TI_CL430_VERSION_CHECK(4, 0, 0) &&                                   \
   defined(__TI_GNU_ATTRIBUTE_SUPPORT__)) ||                                   \
  HEDLEY_TI_CL430_VERSION_CHECK(4, 3, 0) ||                                    \
  (HEDLEY_TI_CL6X_VERSION_CHECK(7, 2, 0) &&                                    \
   defined(__TI_GNU_ATTRIBUTE_SUPPORT__)) ||                                   \
  HEDLEY_TI_CL6X_VERSION_CHECK(7, 5, 0) ||                                     \
  HEDLEY_TI_CL7X_VERSION_CHECK(1, 2, 0) ||                                     \
  HEDLEY_TI_CLPRU_VERSION_CHECK(2, 1, 0)
#define VEG_DEPRECATED(Reason) __attribute__((__deprecated__))
#elif HEDLEY_MSVC_VERSION_CHECK(13, 10, 0) ||                                  \
  HEDLEY_PELLES_VERSION_CHECK(6, 50, 0) ||                                     \
  HEDLEY_INTEL_CL_VERSION_CHECK(2021, 1, 0)
#define VEG_DEPRECATED(Reason) __declspec(deprecated)
#elif HEDLEY_IAR_VERSION_CHECK(8, 0, 0)
#define VEG_DEPRECATED(Reason) _Pragma("deprecated")
#else
#define VEG_DEPRECATED(Reason)
#endif

/// \brief Helper to declare that a parameter is unused
#define VEG_UNUSED_VARIABLE(var) (void)(var)
#ifdef NDEBUG
#define VEG_ONLY_USED_FOR_DEBUG(var) VEG_UNUSED_VARIABLE(var)
#else
#define VEG_ONLY_USED_FOR_DEBUG(var)
#endif

////////////////////////////////////////////////////////////////////////////////

#if defined(__clang__)
#define VEG_WRAP_SILENCE_WARNING(...)                                          \
  HEDLEY_DIAGNOSTIC_PUSH _Pragma(                                              \
    "clang diagnostic ignored \"-Wc++17-extensions\"")                         \
    __VA_ARGS__ HEDLEY_DIAGNOSTIC_POP
#else
#define VEG_WRAP_SILENCE_WARNING(...) __VA_ARGS__
#endif

#ifndef VEG_HAS_NO_UNIQUE_ADDRESS
#define VEG_HAS_NO_UNIQUE_ADDRESS 0
#endif

#if VEG_HAS_NO_UNIQUE_ADDRESS
#ifdef _MSC_VER
#define VEG_NO_UNIQUE_ADDRESS [[msvc::no_unique_address]]
#else
#define VEG_NO_UNIQUE_ADDRESS [[no_unique_address]]
#endif
#else
#define VEG_NO_UNIQUE_ADDRESS
#endif

#ifndef VEG_INLINE

#if defined(NDEBUG) || defined(__OPTIMIZE__)
#define VEG_INLINE HEDLEY_ALWAYS_INLINE
#else
#define VEG_INLINE inline
#endif

#endif

#define VEG_NO_INLINE HEDLEY_NEVER_INLINE

#if defined(__cpp_concepts) && __cpp_concepts >= 201907L
#define VEG_HAS_CONCEPTS 1
#else
#define VEG_HAS_CONCEPTS 0
#endif

#if defined(VEG_WITH_CXX17_SUPPORT)
#define VEG_DECLVAL(...) (static_cast<__VA_ARGS__ (*)() noexcept>(nullptr)())
#else
#define VEG_DECLVAL(...)                                                       \
  (::proxsuite::linalg::veg::_detail::_meta::declval<__VA_ARGS__>())
#endif

#if defined(__clang__)
#define VEG_ARROW(...)                                                         \
  __attribute__((always_inline)) noexcept(noexcept((__VA_ARGS__)))             \
    ->decltype((__VA_ARGS__))                                                  \
  {                                                                            \
    return __VA_ARGS__;                                                        \
  }
#elif defined(__GNUC__) && (__GNUC__ >= 9)
#define VEG_ARROW(...)                                                         \
  noexcept(noexcept((__VA_ARGS__))) __attribute__((always_inline))             \
    ->decltype((__VA_ARGS__))                                                  \
  {                                                                            \
    return __VA_ARGS__;                                                        \
  }
#else
#define VEG_ARROW(...)                                                         \
  noexcept(noexcept((__VA_ARGS__)))->decltype((__VA_ARGS__))                   \
  {                                                                            \
    return __VA_ARGS__;                                                        \
  }
#endif

#define VEG_LAZY_BY_REF(...) [&]() VEG_ARROW(__VA_ARGS__)

#define VEG_LIFT(...)                                                          \
  [&](auto&&... args) VEG_ARROW((__VA_ARGS__)(VEG_FWD(args)...))

#define VEG_DEDUCE_RET(...)                                                    \
  noexcept(noexcept(__VA_ARGS__))->decltype(__VA_ARGS__)                       \
  {                                                                            \
    return __VA_ARGS__;                                                        \
  }                                                                            \
  VEG_NOM_SEMICOLON

#if defined(VEG_WITH_CXX17_SUPPORT)
#define VEG_HAS_FOLD_EXPR 1
#define VEG_ALL_OF(...) (__VA_ARGS__ && ... && true)
#define VEG_ANY_OF(...) (__VA_ARGS__ || ... || false)
#elif defined(__clang__)
#define VEG_HAS_FOLD_EXPR 1
#define VEG_ALL_OF(...) VEG_WRAP_SILENCE_WARNING((__VA_ARGS__ && ... && true))
#define VEG_ANY_OF(...) VEG_WRAP_SILENCE_WARNING((__VA_ARGS__ || ... || false))
#else
#define VEG_HAS_FOLD_EXPR 0
#define VEG_ALL_OF(...)                                                        \
  ::proxsuite::linalg::veg::meta::and_test<                                    \
    ::proxsuite::linalg::veg::meta::make_index_sequence<                       \
      ::proxsuite::linalg::veg::meta::pack_size<decltype((                     \
        void)(__VA_ARGS__))...>::value>,                                       \
    ::proxsuite::linalg::veg::meta::bool_constant<(__VA_ARGS__)>...>::value
#define VEG_ANY_OF(...)                                                        \
  ::proxsuite::linalg::veg::meta::or_test<                                     \
    ::proxsuite::linalg::veg::meta::make_index_sequence<                       \
      ::proxsuite::linalg::veg::meta::pack_size<decltype((                     \
        void)(__VA_ARGS__))...>::value>,                                       \
    ::proxsuite::linalg::veg::meta::bool_constant<(__VA_ARGS__)>...>::value
#endif

#define VEG_EVAL_ALL(...)                                                      \
  ((void)(::proxsuite::linalg::veg::_detail::EmptyArr{                         \
    ::proxsuite::linalg::veg::_detail::Empty{},                                \
    ((void)(__VA_ARGS__), ::proxsuite::linalg::veg::_detail::Empty{})... }))

////////////////////////////////////////////////////////////////////////////////

#if VEG_HAS_CONCEPTS

#define __VEG_IMPL_AND(_, Param) &&__VEG_PP_UNWRAP(Param)
#define __VEG_IMPL_OR(_, Param) || __VEG_PP_UNWRAP(Param)

#define __VEG_IMPL_CONJUNCTION(First, ...)                                     \
  (__VEG_PP_UNWRAP(First)                                                      \
     __VEG_PP_TUPLE_FOR_EACH(__VEG_IMPL_AND, _, (__VA_ARGS__)))

#define __VEG_IMPL_DISJUNCTION(First, ...)                                     \
  (__VEG_PP_UNWRAP(First)                                                      \
     __VEG_PP_TUPLE_FOR_EACH(__VEG_IMPL_OR, _, (__VA_ARGS__)))

#define VEG_DEF_CONCEPT(Tpl, Name, ...)                                        \
  template<__VEG_PP_REMOVE_PAREN1(Tpl)>                                        \
  concept Name = __VA_ARGS__;                                                  \
  VEG_NOM_SEMICOLON

#define VEG_CHECK_CONCEPT_MACRO(Namespace, ...)                                \
  static_assert(Namespace::__VA_ARGS__,                                        \
                __VEG_PP_STRINGIZE(__VA_ARGS__) " failed")
#define VEG_CONCEPT_MACRO(Namespace, ...) Namespace::__VA_ARGS__

#define VEG_DEF_CONCEPT_CONJUNCTION(Tpl, Name, Terms)                          \
  VEG_DEF_CONCEPT(Tpl, Name, __VEG_IMPL_CONJUNCTION Terms)
#define VEG_DEF_CONCEPT_DISJUNCTION(Tpl, Name, Terms)                          \
  VEG_DEF_CONCEPT(Tpl, Name, __VEG_IMPL_DISJUNCTION Terms)

#define VEG_CONCEPT_EXPR(Tpl, TplNames, Name, Expr, ...)                       \
  namespace _veg_detail {                                                      \
  template<typename ExprType, __VEG_PP_REMOVE_PAREN1(Tpl)>                     \
  concept test_return_##Name = __VA_ARGS__;                                    \
  }                                                                            \
  template<__VEG_PP_REMOVE_PAREN1(Tpl)>                                        \
  concept Name = _veg_detail::test_return_                                     \
  ##Name<decltype((Expr)), __VEG_PP_REMOVE_PAREN1(TplNames)>;                  \
  template<__VEG_PP_REMOVE_PAREN1(Tpl)>                                        \
  concept xnothrow_##Name = noexcept(Expr);                                    \
  template<__VEG_PP_REMOVE_PAREN1(Tpl)>                                        \
  concept nothrow_##Name = noexcept(Expr);                                     \
  VEG_NOM_SEMICOLON

#else

#if defined(VEG_WITH_CXX14_SUPPORT)
#define __VEG_IMPL_DEF_CONCEPT(Tpl, Name, Value, ...)                          \
  namespace _ {                                                                \
  template<__VEG_PP_REMOVE_PAREN1(Tpl)>                                        \
  struct Name : __VA_ARGS__                                                    \
  {};                                                                          \
  }                                                                            \
  namespace {                                                                  \
  template<__VEG_PP_REMOVE_PAREN1(Tpl)>                                        \
  constexpr bool const& Name =                                                 \
    ::proxsuite::linalg::veg::meta::bool_constant<Value>::value;               \
  }                                                                            \
  VEG_NOM_SEMICOLON
#else
#define __VEG_IMPL_DEF_CONCEPT(Tpl, Name, Value, ...)                          \
  template<__VEG_PP_REMOVE_PAREN1(Tpl)>                                        \
  struct Name : __VA_ARGS__                                                    \
  {}
#endif

#ifdef __clang__
#define __VEG_NO_WARNING_PRAGMA_PUSH                                           \
  HEDLEY_DIAGNOSTIC_PUSH _Pragma("clang diagnostic ignored \"-Wconversion\"")
#define __VEG_NO_WARNING_PRAGMA_POP HEDLEY_DIAGNOSTIC_POP
#else
#define __VEG_NO_WARNING_PRAGMA_PUSH
#define __VEG_NO_WARNING_PRAGMA_POP
#endif

#define VEG_CONCEPT_EXPR(Tpl, TplNames, Name, Expr, ...)                       \
  namespace _veg_detail {                                                      \
  template<typename _veg_Enable, __VEG_PP_REMOVE_PAREN1(Tpl)>                  \
  struct test_sfinae_##Name                                                    \
  {                                                                            \
    using TestExpr = ::proxsuite::linalg::veg::meta::false_type;               \
    using NothrowTestExpr = ::proxsuite::linalg::veg::meta::false_type;        \
  };                                                                           \
  template<__VEG_PP_REMOVE_PAREN1(Tpl)>                                        \
  struct test_sfinae_##Name<                                                   \
    ::proxsuite::linalg::veg::meta::void_t<decltype((Expr))>,                  \
    __VEG_PP_REMOVE_PAREN1(TplNames)>                                          \
  {                                                                            \
    using ExprType = decltype((Expr));                                         \
    using TestExpr =                                                           \
      ::proxsuite::linalg::veg::meta::bool_constant<__VA_ARGS__>;              \
    using NothrowTestExpr = ::proxsuite::linalg::veg::meta::bool_constant<     \
      (TestExpr::value) && __VEG_NO_WARNING_PRAGMA_PUSH noexcept(Expr)         \
                             __VEG_NO_WARNING_PRAGMA_POP>;                     \
  };                                                                           \
  }                                                                            \
  VEG_DEF_CONCEPT(                                                             \
    Tpl,                                                                       \
    Name,                                                                      \
    _veg_detail::test_sfinae_##Name<void, __VEG_PP_REMOVE_PAREN1(TplNames)>::  \
      TestExpr::value);                                                        \
  VEG_DEF_CONCEPT(Tpl, nothrow_##Name, noexcept(Expr));                        \
  VEG_DEF_CONCEPT(                                                             \
    Tpl,                                                                       \
    xnothrow_##Name,                                                           \
    _veg_detail::test_sfinae_##Name<void, __VEG_PP_REMOVE_PAREN1(TplNames)>::  \
      NothrowTestExpr::value);                                                 \
  VEG_NOM_SEMICOLON

#if defined(VEG_WITH_CXX14_SUPPORT)
#define VEG_CONCEPT_MACRO(Namespace, ...) Namespace::__VA_ARGS__
#define __VEG_IMPL_ADD_VALUE(I, _, Param) (Param)
#define __VEG_IMPL_TRAIT(Param) __VEG_PP_HEAD Param _::__VEG_PP_TAIL Param
#else
#define VEG_CONCEPT_MACRO(Namespace, ...) Namespace::__VA_ARGS__::value
#define __VEG_IMPL_ADD_VALUE(I, _, Param)                                      \
  ((__VEG_PP_REMOVE_PAREN(Param)::value))
#define __VEG_IMPL_TRAIT(Param) __VEG_PP_UNWRAP(Param)
#endif
#define __VEG_IMPL_PUT_TRAIT(I, _, Param) __VEG_IMPL_TRAIT(Param)

#define VEG_CHECK_CONCEPT_MACRO(Namespace, ...)                                \
  static_assert(decltype(Namespace::check_##__VA_ARGS__())::value,             \
                __VEG_PP_STRINGIZE(__VA_ARGS__) " failed")
#define VEG_DEF_CONCEPT(Tpl, Name, ...)                                        \
  __VEG_IMPL_DEF_CONCEPT(                                                      \
    Tpl,                                                                       \
    Name,                                                                      \
    (__VA_ARGS__),                                                             \
    ::proxsuite::linalg::veg::meta::bool_constant<__VA_ARGS__>);               \
  VEG_TEMPLATE(                                                                \
    Tpl, requires(__VA_ARGS__), constexpr auto check_##Name, (_ = 0, int))     \
  noexcept -> ::proxsuite::linalg::veg::meta::true_type

#define __VEG_IMPL_SFINAE(_, Param)                                            \
  , ::proxsuite::linalg::veg::meta::                                           \
      enable_if_t<__VEG_PP_ID(__VEG_PP_UNWRAP Param), int> = 0

#define __VEG_IMPL_OVERLOAD(Name_Tpl, Param)                                   \
  template<__VEG_PP_REMOVE_PAREN(__VEG_PP_TAIL Name_Tpl),                      \
           typename ::proxsuite::linalg::veg::_detail::_meta::                 \
             enable_if<__VEG_PP_ID(__VEG_PP_UNWRAP Param), int>::type = 0>     \
  auto __VEG_PP_CAT(check_, __VEG_PP_HEAD Name_Tpl)() noexcept                 \
    -> ::proxsuite::linalg::veg::meta::true_type;

#define VEG_DEF_CONCEPT_BOOL_CONJUNCTION_IMPL(Tpl, Name, Base, Seq)            \
  __VEG_IMPL_DEF_CONCEPT(Tpl,                                                  \
                         Name,                                                 \
                         (__VEG_PP_REMOVE_PAREN1(Base)::value),                \
                         __VEG_PP_REMOVE_PAREN1(Base));                        \
  template<__VEG_PP_REMOVE_PAREN(Tpl)                                          \
             __VEG_PP_TUPLE_FOR_EACH(__VEG_IMPL_SFINAE, _, Seq)>               \
  auto check_##Name() noexcept -> ::proxsuite::linalg::veg::meta::true_type
#define VEG_DEF_CONCEPT_BOOL_DISJUNCTION_IMPL(Tpl, Name, Base, Seq)            \
  __VEG_IMPL_DEF_CONCEPT(Tpl,                                                  \
                         Name,                                                 \
                         (__VEG_PP_REMOVE_PAREN1(Base)::value),                \
                         __VEG_PP_REMOVE_PAREN1(Base));                        \
  __VEG_PP_TUPLE_FOR_EACH(__VEG_IMPL_OVERLOAD, (Name, Tpl), Seq)               \
  VEG_NOM_SEMICOLON

#define VEG_DEF_CONCEPT_CONJUNCTION(Tpl, Name, Terms)                          \
  VEG_DEF_CONCEPT_BOOL_CONJUNCTION_IMPL(                                       \
    Tpl,                                                                       \
    Name,                                                                      \
    (__VEG_IMPL_CONJUNCTION(Terms)),                                           \
    __VEG_PP_TUPLE_TRANSFORM_I(__VEG_IMPL_ADD_VALUE, _, Terms))

#define VEG_DEF_CONCEPT_DISJUNCTION(Tpl, Name, Terms)                          \
  VEG_DEF_CONCEPT_BOOL_DISJUNCTION_IMPL(                                       \
    Tpl,                                                                       \
    Name,                                                                      \
    (__VEG_IMPL_DISJUNCTION(Terms)),                                           \
    __VEG_PP_TUPLE_TRANSFORM_I(__VEG_IMPL_ADD_VALUE, _, Terms))

#define __VEG_IMPL_CONJUNCTION(Tuple)                                          \
  ::proxsuite::linalg::veg::meta::conjunction<__VEG_PP_REMOVE_PAREN(           \
    __VEG_PP_TUPLE_TRANSFORM_I(__VEG_IMPL_PUT_TRAIT, _, Tuple))>
#define __VEG_IMPL_DISJUNCTION(Tuple)                                          \
  ::proxsuite::linalg::veg::meta::disjunction<__VEG_PP_REMOVE_PAREN(           \
    __VEG_PP_TUPLE_TRANSFORM_I(__VEG_IMPL_PUT_TRAIT, _, Tuple))>

#endif

////////////////////////////////////////////////////////////////////////////////

#define VEG_TEMPLATE(TParams, Constraint, Attr_Name, ...)                      \
  __VEG_IMPL_TEMPLATE(Attr_Name,                                               \
                      TParams,                                                 \
                      __VEG_PP_CAT2(__VEG_IMPL_PREFIX_, Constraint),           \
                      __VA_ARGS__)

#if VEG_HAS_CONCEPTS
#define VEG_CONSTRAINED_MEMBER_FN(Constraint, Attr_Name, Params, ...)          \
  Attr_Name __VEG_PP_TUPLE_TRANSFORM_I(__VEG_IMPL_PARAM_EXPAND, _, Params)     \
  __VA_ARGS__                                                                  \
  requires __VEG_PP_CAT2(__VEG_IMPL_PREFIX_, Constraint)

#define VEG_TEMPLATE_CVT(TParams, Constraint, Attr, ...)                       \
  template<__VEG_PP_REMOVE_PAREN(TParams)>                                     \
  Constraint Attr operator __VA_ARGS__()
#else
#define VEG_CONSTRAINED_MEMBER_FN(Constraint, Attr_Name, Params, ...)          \
  VEG_TEMPLATE(                                                                \
    (int _ = 0),                                                               \
    requires(__VEG_PP_CAT2(__VEG_IMPL_PREFIX_, Constraint) &&                  \
             ::proxsuite::linalg::veg::meta::bool_constant<(_ == 0)>::value),  \
    Attr_Name,                                                                 \
    __VEG_PP_REMOVE_PAREN(Params))                                             \
  __VA_ARGS__

#define VEG_TEMPLATE_CVT(TParams, Constraint, Attr, ...)                       \
  template<__VEG_PP_REMOVE_PAREN(TParams)>                                     \
  Attr operator ::proxsuite::linalg::veg::meta::enable_if_t<                   \
    (__VEG_PP_CAT2(__VEG_IMPL_PREFIX_, Constraint)),                           \
    __VA_ARGS__>()
#endif

#if VEG_HAS_CONCEPTS && defined(__cpp_conditional_explicit) &&                 \
  (__cpp_conditional_explicit >= 201806L)
#define VEG_TEMPLATE_EXPLICIT(                                                 \
  TParams, Constraint, Explicit_Cond, Attr_Name, Params, ...)                  \
  VEG_TEMPLATE(TParams,                                                        \
               Constraint,                                                     \
               Explicit_Cond Attr_Name,                                        \
               __VEG_PP_REMOVE_PAREN(Params))                                  \
  __VA_ARGS__

#define VEG_TEMPLATE_CVT_EXPLICIT(                                             \
  TParams, Constraint, Explicit_Cond, Attr, Type, ...)                         \
  template<__VEG_PP_REMOVE_PAREN(TParams)>                                     \
  Constraint Explicit_Cond Attr operator __VEG_PP_REMOVE_PAREN(Type)()         \
    __VA_ARGS__

#else
#define VEG_TEMPLATE_EXPLICIT(                                                 \
  TParams, Constraint, Explicit_Cond, Attr_Name, Params, ...)                  \
  VEG_TEMPLATE(                                                                \
    (__VEG_PP_REMOVE_PAREN TParams,                                            \
     ::proxsuite::linalg::veg::meta::                                          \
       enable_if_t<(__VEG_PP_CAT2(__VEG_IMPL_PREFIX_, Explicit_Cond)), int> =  \
         0),                                                                   \
    Constraint,                                                                \
    explicit Attr_Name,                                                        \
    __VEG_PP_REMOVE_PAREN(Params))                                             \
  __VA_ARGS__                                                                  \
  VEG_TEMPLATE((__VEG_PP_REMOVE_PAREN TParams,                                 \
                ::proxsuite::linalg::veg::meta::enable_if_t<                   \
                  !(__VEG_PP_CAT2(__VEG_IMPL_PREFIX_, Explicit_Cond)),         \
                  unsigned> = 0),                                              \
               Constraint,                                                     \
               Attr_Name,                                                      \
               __VEG_PP_REMOVE_PAREN(Params))                                  \
  __VA_ARGS__

#define VEG_TEMPLATE_CVT_EXPLICIT(                                             \
  TParams, Constraint, Explicit_Cond, Attr, Type, ...)                         \
  VEG_TEMPLATE_CVT(                                                            \
    (__VEG_PP_REMOVE_PAREN TParams,                                            \
     ::proxsuite::linalg::veg::meta::                                          \
       enable_if_t<(__VEG_PP_CAT2(__VEG_IMPL_PREFIX_, Explicit_Cond)), int> =  \
         0),                                                                   \
    Constraint,                                                                \
    explicit Attr,                                                             \
    __VEG_PP_REMOVE_PAREN(Type))                                               \
  __VA_ARGS__                                                                  \
                                                                               \
  VEG_TEMPLATE_CVT((__VEG_PP_REMOVE_PAREN TParams,                             \
                    ::proxsuite::linalg::veg::meta::enable_if_t<               \
                      !(__VEG_PP_CAT2(__VEG_IMPL_PREFIX_, Explicit_Cond)),     \
                      unsigned> = 0),                                          \
                   Constraint,                                                 \
                   Attr,                                                       \
                   __VEG_PP_REMOVE_PAREN(Type))                                \
  __VA_ARGS__
#endif

#define __VEG_IMPL_PREFIX_requires
#define __VEG_IMPL_PREFIX_explicit

#define __VEG_IMPL_PARAM_EXPAND(I, _, Param)                                   \
  __VEG_PP_TAIL Param __VEG_PP_HEAD Param
#if VEG_HAS_CONCEPTS
#define __VEG_IMPL_TEMPLATE(Attr_Name, TParams, Constraint, ...)               \
  template<__VEG_PP_REMOVE_PAREN(TParams)>                                     \
    requires Constraint                                                        \
  Attr_Name __VEG_PP_TUPLE_TRANSFORM_I(                                        \
    __VEG_IMPL_PARAM_EXPAND, _, (__VA_ARGS__))
#else

#define __VEG_IMPL_TEMPLATE2_HELPER_0(Constraint, Param)                       \
  __VEG_PP_TAIL Param __VEG_PP_HEAD Param

#define __VEG_IMPL_TEMPLATE2_HELPER_1(Constraint, Param)                       \
  ::proxsuite::linalg::veg::meta::enable_if_t<(Constraint),                    \
                                              __VEG_PP_TAIL Param>             \
    __VEG_PP_HEAD Param

#define __VEG_IMPL_TEMPLATE2_HELPER(I, Constraint, Param)                      \
  __VEG_PP_CAT2(__VEG_IMPL_TEMPLATE2_HELPER_,                                  \
                __VEG_IMPL_PP_IS_1(__VEG_IMPL_PP_INC(I)))                      \
  (Constraint, Param)

#define __VEG_IMPL_TEMPLATE(Attr_Name, TParams, Constraint, ...)               \
  template<__VEG_PP_REMOVE_PAREN(TParams)>                                     \
  Attr_Name __VEG_PP_TUPLE_TRANSFORM_I(                                        \
    __VEG_IMPL_TEMPLATE2_HELPER, Constraint, (__VA_ARGS__))

#endif

////////////////////////////////////////////////////////////////////////////////

#if defined(VEG_WITH_CXX17_SUPPORT)
#define VEG_INLINE_VAR(Name, Obj)                                              \
  inline constexpr auto const& Name =                                          \
    ::proxsuite::linalg::veg::meta::static_const<Obj>::value;                  \
  static_assert((void(Name), true), ".")

#define VEG_INLINE_VAR_TEMPLATE(Tpl, Name, ...) /* NOLINT */                   \
  template<__VEG_PP_REMOVE_PAREN(Tpl)>                                         \
  inline constexpr auto const& Name =                                          \
    ::proxsuite::linalg::veg::meta::static_const<__VA_ARGS__>::value;          \
  VEG_NOM_SEMICOLON /* NOLINT */
#else
#define VEG_INLINE_VAR(Name, Obj)                                              \
  namespace /* NOLINT */ {                                                     \
  constexpr auto const& Name =                                                 \
    ::proxsuite::linalg::veg::meta::static_const<Obj>::value;                  \
  }                                                                            \
  static_assert((void(Name), true), ".")

#if defined(VEG_WITH_CXX14_SUPPORT)
#define VEG_INLINE_VAR_TEMPLATE(Tpl, Name, ...) /* NOLINT */                   \
  namespace /* NOLINT */ {                      /* NOLINT */                   \
  template<__VEG_PP_REMOVE_PAREN(Tpl)>                                         \
  constexpr auto const& Name = /* NOLINT */                                    \
    ::proxsuite::linalg::veg::meta::static_const<                              \
      __VA_ARGS__>::value; /* NOLINT                                           \
                            */                                                 \
  }                                                                            \
  VEG_NOM_SEMICOLON /* NOLINT */
#else
#define VEG_INLINE_VAR_TEMPLATE(Tpl, Name, ...) VEG_NOM_SEMICOLON
#endif
#endif

#define VEG_NIEBLOID(Name) VEG_INLINE_VAR(Name, nb::Name) // NOLINT

#define VEG_NIEBLOID_TEMPLATE(Tpl, Name, ...)                                  \
  VEG_INLINE_VAR_TEMPLATE(Tpl, Name, nb::Name<__VA_ARGS__>) // NOLINT

#define VEG_TAG(Name, Type)                                                    \
  namespace _ {                                                                \
  template<int I>                                                              \
  struct Type                                                                  \
  {                                                                            \
    explicit Type() = default;                                                 \
  };                                                                           \
  }                                                                            \
  using Type = _::Type<0>;                                                     \
  VEG_INLINE_VAR(Name, Type)

#define VEG_TAG_TEMPLATE(Tpl, Name, Type, ...)                                 \
  template<__VEG_PP_REMOVE_PAREN(Tpl)>                                         \
  struct Type                                                                  \
  {                                                                            \
    explicit Type() = default;                                                 \
  };                                                                           \
  VEG_INLINE_VAR_TEMPLATE(Tpl, Name, Type<__VA_ARGS__>)

#define VEG_FWD(X) static_cast<decltype(X)&&>(X)
#define VEG_FWD2(X) static_cast<decltype(X)>(static_cast<decltype(X)&&>(X))

// disallows moving const rvalues
#define VEG_MOV(X)                                                             \
  static_cast<typename ::proxsuite::linalg::veg::uncvref_t<decltype(X)>&&>(X)

#ifdef VEG_NO_INSTANTIATE
#define VEG_INSTANTIATE(Fn, ...) VEG_NOM_SEMICOLON
#define VEG_INSTANTIATE_CLASS(Class, ...) VEG_NOM_SEMICOLON
#else
#define VEG_INSTANTIATE(Fn, ...)                                               \
  __VEG_IMPL_INSTANTIATE(                                                      \
    Fn,                                                                        \
    __VEG_PP_CAT(__VEG_PP_CAT(_dummy_explicit_instantiation, __LINE__),        \
                 __VEG_PP_CAT(_, __COUNTER__)),                                \
    __VA_ARGS__)
#define __VEG_IMPL_INSTANTIATE(Fn, Name, ...)                                  \
  template<typename... Ts>                                                     \
  struct Name                                                                  \
  {                                                                            \
    void apply(Ts&&... args)                                                   \
    {                                                                          \
      Fn(VEG_FWD(args)...);                                                    \
    }                                                                          \
  };                                                                           \
  template struct Name<__VA_ARGS__>

#define VEG_INSTANTIATE_CLASS(Class, ...) template struct Class<__VA_ARGS__>
#endif

#define VEG_NOM_SEMICOLON static_assert(true, ".")
#define VEG_ID(id) __VEG_PP_CAT(id, __COUNTER__)

namespace proxsuite {
namespace linalg {
namespace veg {
template<typename T>
struct Slice;

namespace meta {
template<typename...>
using void_t = void;
} // namespace meta
namespace _detail {
template<typename T>
struct Wrapper
{
  T inner;
};
namespace _meta {

template<bool B, typename T = void>
struct enable_if
{
  using type = T;
};
template<typename T>
struct enable_if<false, T>
{};

template<typename U, typename V>
using discard_1st = V;

template<typename T>
struct uncvlref;
template<typename T>
struct uncvlref<T&>
{
  using type = T;
};
template<typename T>
struct uncvlref<T const&>
{
  using type = T;
};
template<typename T>
struct uncvlref<T volatile&>
{
  using type = T;
};
template<typename T>
struct uncvlref<T volatile const&>
{
  using type = T;
};

template<typename T>
struct unref;
template<typename T>
struct unref<T&>
{
  using type = T;
};

template<typename T>
auto
declval() VEG_ALWAYS_NOEXCEPT->T;
} // namespace _meta
} // namespace _detail

namespace meta {
template<typename T>
struct static_const
{
  static constexpr T value{};
};

template<typename T>
constexpr T static_const<T>::value; // NOLINT(readability-redundant-declaration)
} // namespace meta

template<typename... Ts>
struct incomplete_t;

template<typename... Types, typename... Args>
auto
print_types_halt(Args&&...) -> incomplete_t<Types..., Args...>;
template<typename... Types, typename... Args>
VEG_DEPRECATED("")
VEG_CPP14(constexpr) void print_types(Args&&... /*unused*/)
{
}

namespace nb {
struct unused
{
  template<typename... Ts>
  VEG_CPP14(constexpr)
  void operator()(Ts const&... /*unused*/) const VEG_NOEXCEPT
  {
  }
};
} // namespace nb
VEG_NIEBLOID(unused);

using usize = decltype(sizeof(0));
namespace _detail {

template<isize I>
struct EmptyI
{};

using Empty = EmptyI<0>;
using EmptyArr = Empty[];
namespace _meta {

template<typename T, T... Nums>
struct integer_sequence;

#if VEG_HAS_BUILTIN(__make_integer_seq)

template<typename T, T N>
using make_integer_sequence = __make_integer_seq<integer_sequence, T, N>;

#elif __GNUC__ >= 8

template<typename T, T N>
using make_integer_sequence = integer_sequence<T, __integer_pack(N)...>;

#else

namespace _detail {

template<typename Seq1, typename Seq2>
struct _merge;

template<typename Seq1, typename Seq2>
struct _merge_p1;

template<typename T, T... Nums1, T... Nums2>
struct _merge<integer_sequence<T, Nums1...>, integer_sequence<T, Nums2...>>
{
  using type = integer_sequence<T, Nums1..., (sizeof...(Nums1) + Nums2)...>;
};

template<typename T, T... Nums1, T... Nums2>
struct _merge_p1<integer_sequence<T, Nums1...>, integer_sequence<T, Nums2...>>
{
  using type = integer_sequence<T,
                                Nums1...,
                                (sizeof...(Nums1) + Nums2)...,
                                sizeof...(Nums1) + sizeof...(Nums2)>;
};

template<typename T, usize N, bool Even = (N % 2) == 0>
struct _make_integer_sequence
{
  using type =
    typename _merge<typename _make_integer_sequence<T, N / 2>::type,
                    typename _make_integer_sequence<T, N / 2>::type>::type;
};

template<typename T, usize N>
struct _make_integer_sequence<T, N, false>
{
  using type =
    typename _merge_p1<typename _make_integer_sequence<T, N / 2>::type,
                       typename _make_integer_sequence<T, N / 2>::type>::type;
};

template<typename T>
struct _make_integer_sequence<T, 0>
{
  using type = integer_sequence<T>;
};
template<typename T>
struct _make_integer_sequence<T, 1>
{
  using type = integer_sequence<T, 0>;
};

} // namespace _detail

template<typename T, T N>
using make_integer_sequence =
  typename _detail::_make_integer_sequence<T, N>::type;

#endif

#define VEG_DEF_CONCEPT_BUILTIN_OR_INTERNAL(Tpl, Name, ...)                    \
  VEG_DEF_CONCEPT(                                                             \
    Tpl,                                                                       \
    Name,                                                                      \
    VEG_HAS_BUILTIN_OR(__is_##Name,                                            \
                       __is_##Name(__VA_ARGS__),                               \
                       (_detail::_meta::is_##Name<__VA_ARGS__>::value)))

template<usize N>
using make_index_sequence = make_integer_sequence<usize, N>;

template<typename... Ts>
struct type_sequence;

} // namespace _meta

template<usize I, typename T>
struct SimpleLeaf
{
  T inner;
};

template<typename Seq, typename... Ts>
struct SimpleITuple;

template<usize... Is, typename... Ts>
struct SimpleITuple<_meta::integer_sequence<usize, Is...>, Ts...>
  : SimpleLeaf<Is, Ts>...
{
#if !defined(VEG_WITH_CXX17_SUPPORT)
  constexpr SimpleITuple(Ts... args) noexcept
    : SimpleLeaf<Is, Ts>{ Ts(VEG_FWD(args)) }...
  {
  }
#endif
};

template<typename... Ts>
using SimpleTuple =
  SimpleITuple<_meta::make_index_sequence<sizeof...(Ts)>, Ts...>;

template<typename... Ts>
constexpr auto
make_simple_tuple(Empty /*dummy*/, Ts... args) noexcept -> SimpleTuple<Ts...>
{
#if !defined(VEG_WITH_CXX17_SUPPORT)
  return { Ts(VEG_FWD(args))... };
#else
  return { { Ts(VEG_FWD(args)) }... };
#endif
}

template<typename T>
struct mem_ptr_type;
template<typename C, typename Mem>
struct mem_ptr_type<Mem C::*>
{
  using Type = Mem;
};

constexpr auto
all_of_slice(bool const* arr, usize size) VEG_NOEXCEPT->bool
{
  return size == 0 ? true
                   : (arr[0] && _detail::all_of_slice(arr + 1, size - 1));
}
template<usize N>
inline constexpr auto
all_of(bool const (&lst)[N]) VEG_NOEXCEPT->bool
{
  return _detail::all_of_slice(lst, N);
}

template<typename T>
struct member_extract_access
{
  template<typename U, typename = U>
  struct DetectImpl
  {
    static constexpr bool value = false;
    using Type = void;
  };
  template<typename U>
  struct DetectImpl<U, typename U::_veglib_impl_member_extract::Type>
  {
    static constexpr bool value = true;
    using Type = typename U::_veglib_impl_member_extract;
  };

  using Detect = DetectImpl<T>;
  static constexpr bool value = Detect::value;
  using Type = typename Detect::Type;
};
} // namespace _detail
namespace meta {
template<bool B, typename T = void>
using enable_if_t =
  _detail::_meta::discard_1st<typename _detail::_meta::enable_if<B, void>::type,
                              T>;

template<typename T>
using uncvref_t = typename _detail::_meta::uncvlref<T&>::type;
} // namespace meta
using meta::uncvref_t;

namespace meta {
template<typename T, T Value>
struct constant
{
  static constexpr T value = Value;
};
template<typename T, T Value>
constexpr T constant<T, Value>::value;
template<bool B>
using bool_constant = constant<bool, B>;

using true_type = bool_constant<true>;
using false_type = bool_constant<false>;
} // namespace meta
namespace _detail {
namespace _meta {

struct wrapper_base
{
  static auto test(...) -> meta::false_type;
};
template<typename T>
struct wrapper : wrapper_base
{
  using wrapper_base::test;
  static auto test(wrapper<T>*) -> meta::true_type;
};
template<typename T, typename U>
using is_same = decltype(wrapper<T>::test(static_cast<wrapper<U>*>(nullptr)));
} // namespace _meta
} // namespace _detail
namespace concepts {
VEG_DEF_CONCEPT_BUILTIN_OR_INTERNAL((typename T, typename U), same, T, U);
} // namespace concepts

enum struct CharUnit : unsigned char
{
  SPACE = 0x20,
  EXCLAMATION_MARK = 0x21,
  DOUBLE_QUOTE = 0x22,
  NUMBER = 0x23,
  DOLLAR = 0x24,
  PERCENT = 0x25,
  AMPERSAND = 0x26,
  SINGLE_QUOTE = 0x27,
  LEFT_PARENTHESIS = 0x28,
  RIGHT_PARENTHESIS = 0x29,
  ASTERISK = 0x2A,
  PLUS = 0x2B,
  COMMA = 0x2C,
  MINUS = 0x2D,
  PERIOD = 0x2E,
  SLASH = 0x2F,
  ZERO = 0x30,
  ONE = 0x31,
  TWO = 0x32,
  THREE = 0x33,
  FOUR = 0x34,
  FIVE = 0x35,
  SIX = 0x36,
  SEVEN = 0x37,
  EIGHT = 0x38,
  NINE = 0x39,
  COLON = 0x3A,
  SEMICOLON = 0x3B,
  LESS_THAN = 0x3C,
  EQUALITY_SIGN = 0x3D,
  GREATER_THAN = 0x3E,
  QUESTION_MARK = 0x3F,
  AT_SIGN = 0x40,
  UPPERCASE_A = 0x41,
  UPPERCASE_B = 0x42,
  UPPERCASE_C = 0x43,
  UPPERCASE_D = 0x44,
  UPPERCASE_E = 0x45,
  UPPERCASE_F = 0x46,
  UPPERCASE_G = 0x47,
  UPPERCASE_H = 0x48,
  UPPERCASE_I = 0x49,
  UPPERCASE_J = 0x4A,
  UPPERCASE_K = 0x4B,
  UPPERCASE_L = 0x4C,
  UPPERCASE_M = 0x4D,
  UPPERCASE_N = 0x4E,
  UPPERCASE_O = 0x4F,
  UPPERCASE_P = 0x50,
  UPPERCASE_Q = 0x51,
  UPPERCASE_R = 0x52,
  UPPERCASE_S = 0x53,
  UPPERCASE_T = 0x54,
  UPPERCASE_U = 0x55,
  UPPERCASE_V = 0x56,
  UPPERCASE_W = 0x57,
  UPPERCASE_X = 0x58,
  UPPERCASE_Y = 0x59,
  UPPERCASE_Z = 0x5A,
  LEFT_SQUARE_BRACKET = 0x5B,
  BACKSLASH = 0x5C,
  RIGHT_SQUARE_BRACKET = 0x5D,
  CARET = 0x5F,
  GRAVE = 0x60,
  LOWERCASE_A = 0x61,
  LOWERCASE_B = 0x62,
  LOWERCASE_C = 0x63,
  LOWERCASE_D = 0x64,
  LOWERCASE_E = 0x65,
  LOWERCASE_F = 0x66,
  LOWERCASE_G = 0x67,
  LOWERCASE_H = 0x68,
  LOWERCASE_I = 0x69,
  LOWERCASE_J = 0x6A,
  LOWERCASE_K = 0x6B,
  LOWERCASE_L = 0x6C,
  LOWERCASE_M = 0x6D,
  LOWERCASE_N = 0x6E,
  LOWERCASE_O = 0x6F,
  LOWERCASE_P = 0x70,
  LOWERCASE_Q = 0x71,
  LOWERCASE_R = 0x72,
  LOWERCASE_S = 0x73,
  LOWERCASE_T = 0x74,
  LOWERCASE_U = 0x75,
  LOWERCASE_V = 0x76,
  LOWERCASE_W = 0x77,
  LOWERCASE_X = 0x78,
  LOWERCASE_Y = 0x79,
  LOWERCASE_Z = 0x7A,
  LEFT_CURLY_BRACKET = 0x7B,
  VERTICAL_BAR = 0x7C,
  RIGHT_CURLY_BRACKET = 0x7D,
  TILDE = 0x7E,
};

inline namespace tags {
VEG_TAG(from_raw_parts, FromRawParts);

VEG_TAG(safe, Safe);
VEG_TAG(unsafe, Unsafe);
} // namespace tags

struct Str
{
private:
  struct
  {
    CharUnit const* ptr;
    isize len;
  } _ = {};

public:
  VEG_INLINE constexpr Str(Unsafe /*unsafe*/,
                           FromRawParts /*from_raw_parts*/,
                           CharUnit const* ptr,
                           isize len) noexcept
    : _{ ptr, len } {};
  VEG_INLINE constexpr auto as_slice() const noexcept -> Slice<CharUnit>;
};

template<CharUnit... Cs>
struct StrLiteralConstant
{
private:
  static constexpr CharUnit literal[sizeof...(Cs)] = { Cs... };

public:
  VEG_INLINE constexpr auto as_slice() const noexcept -> Slice<CharUnit>;
  VEG_INLINE constexpr auto as_str() const noexcept -> Str
  {
    return { unsafe, from_raw_parts, literal, isize{ sizeof...(Cs) } };
  }
};

template<CharUnit... Cs>
constexpr CharUnit StrLiteralConstant<Cs...>::literal[sizeof...(Cs)];

namespace _detail {
using NativeChar8 = meta::uncvref_t<decltype(u8""[0])>;

template<typename T, usize N>
struct Array_
{
  T _[N];
};
template<typename T>
constexpr auto
assert_complete() noexcept -> bool
{
  static_assert(sizeof(T) != 0, ".");
  return true;
}
} // namespace _detail
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#if defined(__clang__) || (defined(VEG_WITH_CXX14_SUPPORT) && defined(__GNUC__))
HEDLEY_DIAGNOSTIC_PUSH
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef __clang__
#pragma clang diagnostic ignored "-Wgnu-string-literal-operator-template"
#endif

template<typename Char,
         Char... Cs>
constexpr auto operator""__veglib_const_literal_gnuc() noexcept // NOLINT
  -> proxsuite::linalg::veg::StrLiteralConstant<
    proxsuite::linalg::veg::CharUnit(Cs)...>
{
  return {};
}

HEDLEY_DIAGNOSTIC_POP

#define __VEG_IMPL_UTF8_CONST(Literal) /* NOLINT */                            \
  (u8##Literal##__veglib_const_literal_gnuc)

#elif (defined(__clang__) && defined(VEG_WITH_CXX20_SUPPORT)) ||               \
  (defined(__cpp_nontype_template_args) &&                                     \
   (__cpp_nontype_template_args >= 201911L))
namespace proxsuite {
namespace linalg {
namespace veg {
namespace _detail {
template<isize N>
struct StrLiteralImpl
{
  proxsuite::linalg::veg::CharUnit _[N];

  template<typename Char>
  constexpr StrLiteralImpl(Char const* s)
  {
    for (isize i = 0; i < N; ++i) {
      _[i] = proxsuite::linalg::veg::CharUnit(s[i]);
    }
  }
};
template<typename Char, isize N>
StrLiteralImpl(Char const (&)[N]) -> StrLiteralImpl<N - 1>;

template<typename T>
struct StrLiteralLen;

template<isize N>
struct StrLiteralLen<StrLiteralImpl<N>>
{
  static constexpr usize value{ N };
};
template<isize N>
struct StrLiteralLen<StrLiteralImpl<N> const>
{
  static constexpr usize value{ N };
};

template<typename Seq, auto Literal>
struct StrLiteralExpand;

template<usize... Is, StrLiteralImpl<static_cast<isize>(sizeof...(Is))> L>
struct StrLiteralExpand<_meta::integer_sequence<usize, Is...>, L>
{
  using Type = StrLiteralConstant<L._[Is]...>;
};
} // namespace _detail
} // namespace veg
} // namespace linalg
} // namespace proxsuite

template<proxsuite::linalg::veg::_detail::StrLiteralImpl S>
constexpr auto operator""__veglib_const_literal_cpp20() noexcept ->
  typename proxsuite::linalg::veg::_detail::StrLiteralExpand< //
    proxsuite::linalg::veg::_detail::_meta::make_index_sequence<
      proxsuite::linalg::veg::_detail::StrLiteralLen<decltype(S)>::value>,
    S>::Type
{
  return {};
}
#define __VEG_IMPL_UTF8_CONST(Literal)                                         \
  (u8##Literal##__veglib_const_literal_cpp20)

#else

namespace proxsuite {
namespace linalg {
namespace veg {
namespace _detail {

template<typename LiteralType, typename Seq>
struct ExtractCharsImpl;

template<typename LiteralType, usize... Is>
struct ExtractCharsImpl<LiteralType, _meta::integer_sequence<usize, Is...>>
{
  using Type = StrLiteralConstant<CharUnit(LiteralType::value[Is])...>;
};

template<typename LiteralType, typename Seq>
struct ExtractCharsImplExpr;

template<typename LiteralType, usize... Is>
struct ExtractCharsImplExpr<LiteralType, _meta::integer_sequence<usize, Is...>>
{
  using Type = StrLiteralConstant<CharUnit(LiteralType::value()[Is])...>;
};

template<typename LiteralType>
auto
extract_chars(LiteralType /*unused*/) -> typename ExtractCharsImpl<
  LiteralType,
  _meta::make_index_sequence<LiteralType::Size::value>>::Type
{
  return {};
}

template<typename LiteralType>
auto
extract_chars_expr(LiteralType /*unused*/) -> typename ExtractCharsImplExpr<
  LiteralType,
  _meta::make_index_sequence<LiteralType::Size::value>>::Type
{
  return {};
}
} // namespace _detail
} // namespace veg
} // namespace linalg
} // namespace proxsuite

#define __VEG_IMPL_UTF8_CONST(Literal)                                         \
  (::proxsuite::linalg::veg::_detail::extract_chars_expr(                      \
    []() /* NOLINT */ noexcept {                                               \
      struct __VEG_PP_CAT(_veglib_type, __LINE__)                              \
      {                                                                        \
        static constexpr auto value() noexcept -> decltype(Literal)            \
        {                                                                      \
          return Literal;                                                      \
        }                                                                      \
        using Size = ::proxsuite::linalg::veg::meta::constant<                 \
          ::proxsuite::linalg::veg::usize,                                     \
          sizeof(value()) / sizeof(value()[0]) - 1>;                           \
      };                                                                       \
      return __VEG_PP_CAT(_veglib_type, __LINE__){};                           \
    }()))
#endif

#define VEG_DECLTYPE_VOID(...) decltype(void(__VA_ARGS__))
#define VEG_BOOL_NOEXCEPT(...)                                                 \
  ::proxsuite::linalg::veg::meta::bool_constant<noexcept(__VA_ARGS__)>

#define VEG_CHECK_CONCEPT(...)                                                 \
  VEG_CHECK_CONCEPT_MACRO(::proxsuite::linalg::veg::concepts, __VA_ARGS__)
#define VEG_CONCEPT(...)                                                       \
  VEG_CONCEPT_MACRO(::proxsuite::linalg::veg::concepts, __VA_ARGS__)

#define __VEG_IMPL_GET_MEMBER_PTR(_, MemberPtr) /* NOLINT */ , &Type::MemberPtr
#define __VEG_IMPL_GET_MEMBER_NAME_PTR(_, MemberPtr) /* NOLINT */              \
  static_cast<::proxsuite::linalg::veg::_detail::NativeChar8 const*>(          \
    __VEG_PP_CAT(u8, __VEG_PP_STRINGIZE(MemberPtr))),
#define __VEG_IMPL_GET_MEMBER_NAME_LEN(_, MemberPtr) /* NOLINT */              \
  (sizeof(__VEG_PP_CAT(u8, __VEG_PP_STRINGIZE(MemberPtr))) - 1),

#define __VEG_IMPL_STRUCT_SETUP(PClass, ...) /* NOLINT */                      \
  void _veg_lib_name_test()&& noexcept                                         \
  {                                                                            \
    static_assert(                                                             \
      VEG_CONCEPT(same<decltype(this), __VEG_PP_REMOVE_PAREN(PClass)*>),       \
      "struct mismatch in VEG_REFLECT");                                       \
  }                                                                            \
  struct _veglib_impl_member_extract                                           \
  {                                                                            \
    using Type = __VEG_PP_REMOVE_PAREN(PClass);                                \
    using MemberTuple =                                                        \
      decltype(::proxsuite::linalg::veg::_detail::make_simple_tuple(           \
        ::proxsuite::linalg::veg::_detail::Empty {                             \
        } __VEG_PP_TUPLE_FOR_EACH(__VEG_IMPL_GET_MEMBER_PTR,                   \
                                  _,                                           \
                                  (__VA_ARGS__))));                            \
    static constexpr auto member_pointers() noexcept -> MemberTuple            \
    {                                                                          \
      return ::proxsuite::linalg::veg::_detail::make_simple_tuple(             \
        ::proxsuite::linalg::veg::_detail::Empty {} __VEG_PP_TUPLE_FOR_EACH(   \
          __VEG_IMPL_GET_MEMBER_PTR, _, (__VA_ARGS__)));                       \
    }                                                                          \
    static constexpr auto class_name_ptr() noexcept                            \
      -> ::proxsuite::linalg::veg::_detail::NativeChar8 const*                 \
    {                                                                          \
      return __VEG_PP_CAT(u8,                                                  \
                          __VEG_PP_STRINGIZE(__VEG_PP_REMOVE_PAREN(PClass)));  \
    }                                                                          \
    static constexpr auto class_name_len() noexcept                            \
      -> ::proxsuite::linalg::veg::usize                                       \
    {                                                                          \
      return sizeof(__VEG_PP_CAT(                                              \
               u8, __VEG_PP_STRINGIZE(__VEG_PP_REMOVE_PAREN(PClass)))) -       \
             1;                                                                \
    }                                                                          \
    static constexpr auto member_name_ptrs() noexcept                          \
      -> ::proxsuite::linalg::veg::_detail::Array_<                            \
        ::proxsuite::linalg::veg::_detail::NativeChar8 const*,                 \
        __VEG_PP_TUPLE_SIZE((__VA_ARGS__))>                                    \
    {                                                                          \
      return { { __VEG_PP_TUPLE_FOR_EACH(                                      \
        __VEG_IMPL_GET_MEMBER_NAME_PTR, _, (__VA_ARGS__)) } };                 \
    }                                                                          \
    static constexpr auto member_name_lens() noexcept                          \
      -> ::proxsuite::linalg::veg::_detail::Array_<                            \
        ::proxsuite::linalg::veg::usize,                                       \
        __VEG_PP_TUPLE_SIZE((__VA_ARGS__))>                                    \
    {                                                                          \
      return { { __VEG_PP_TUPLE_FOR_EACH(                                      \
        __VEG_IMPL_GET_MEMBER_NAME_LEN, _, (__VA_ARGS__)) } };                 \
    }                                                                          \
  };                                                                           \
  friend struct ::proxsuite::linalg::veg::_detail::member_extract_access<      \
    __VEG_PP_REMOVE_PAREN(PClass)>;                                            \
  VEG_NOM_SEMICOLON

#define VEG_REFLECT(PClass, ...) __VEG_IMPL_STRUCT_SETUP(PClass, __VA_ARGS__)

#include "proxsuite/linalg/veg/internal/epilogue.hpp"
#endif /* end of include guard VEG_MACROS_HPP_HSTLSKZXS */
