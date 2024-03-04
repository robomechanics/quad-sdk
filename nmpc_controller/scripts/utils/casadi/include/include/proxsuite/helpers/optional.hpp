//
// Copyright (c) 2022 INRIA
//
/**
 * @file optional.hpp
 */

#ifndef PROXSUITE_HELPERS_OPTIONAL_HPP
#define PROXSUITE_HELPERS_OPTIONAL_HPP

#include <proxsuite/fwd.hpp>

#ifdef PROXSUITE_WITH_CPP_17
#include <optional>
#else
#include <proxsuite/helpers/tl-optional.hpp>
#endif

namespace proxsuite {
#ifdef PROXSUITE_WITH_CPP_17
template<class T>
using optional = std::optional<T>;
using nullopt_t = std::nullopt_t;
inline constexpr nullopt_t nullopt = std::nullopt;
#else
namespace detail {
// Source boost: https://www.boost.org/doc/libs/1_74_0/boost/none.hpp
// the trick here is to make instance defined once as a global but in a header
// file
template<typename T>
struct nullopt_instance
{
  static const T instance;
};
template<typename T>
const T nullopt_instance<T>::instance =
  T(tl::nullopt); // global, but because 'tis a template, no cpp file required
} // namespace detail
template<class T>
using optional = tl::optional<T>;
using nullopt_t = tl::nullopt_t;
constexpr nullopt_t nullopt = detail::nullopt_instance<tl::nullopt_t>::instance;
#endif
} // namespace proxsuite

#endif /* end of include guard PROXSUITE_HELPERS_OPTIONAL_HPP */
