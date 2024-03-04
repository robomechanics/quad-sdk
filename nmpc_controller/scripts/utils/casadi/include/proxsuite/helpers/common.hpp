//
// Copyright (c) 2022 INRIA
//
/**
 * @file common.hpp
 */

#ifndef PROXSUITE_HELPERS_COMMON_HPP
#define PROXSUITE_HELPERS_COMMON_HPP

#include "proxsuite/config.hpp"
#include <limits>

namespace proxsuite {
namespace helpers {

template<typename Scalar>
struct infinite_bound
{
  static Scalar value()
  {
    using namespace std;
    return sqrt(std::numeric_limits<Scalar>::max());
  }
};

#define PROXSUITE_DEDUCE_RET(...)                                              \
  noexcept(noexcept(__VA_ARGS__))                                              \
    ->typename std::remove_const<decltype(__VA_ARGS__)>::type                  \
  {                                                                            \
    return __VA_ARGS__;                                                        \
  }                                                                            \
  static_assert(true, ".")

/// @brief \brief Returns the part of the expression which is lower than value
template<typename T, typename Scalar>
auto
at_most(T const& expr, const Scalar value) PROXSUITE_DEDUCE_RET(
  (expr.array() < value).select(expr, T::Constant(expr.rows(), value)));

/// @brief \brief Returns the part of the expression which is greater than value
template<typename T, typename Scalar>
auto
at_least(T const& expr, const Scalar value) PROXSUITE_DEDUCE_RET(
  (expr.array() > value).select(expr, T::Constant(expr.rows(), value)));

/// @brief \brief Returns the positive part of an expression
template<typename T>
auto
positive_part(T const& expr)
  PROXSUITE_DEDUCE_RET((expr.array() > 0).select(expr, T::Zero(expr.rows())));

/// @brief \brief Returns the negative part of an expression
template<typename T>
auto
negative_part(T const& expr)
  PROXSUITE_DEDUCE_RET((expr.array() < 0).select(expr, T::Zero(expr.rows())));

/// @brief \brief Select the components of the expression if the condition is
/// fullfiled. Otherwise, set the component to value
template<typename Condition, typename T, typename Scalar>
auto
select(Condition const& condition, T const& expr, const Scalar value)
  PROXSUITE_DEDUCE_RET((condition).select(expr,
                                          T::Constant(expr.rows(), value)));

} // helpers
} // proxsuite

#endif // ifndef PROXSUITE_HELPERS_COMMON_HPP
