#ifndef RBDL_RBDL_TESTS_H
#define RBDL_RBDL_TESTS_H

#include <cmath>

#include "catch2/catch.hpp"
#include "rbdl/rbdl_math.h"

template <typename T>
struct IsCloseMatcher : Catch::MatcherBase<T> {
  IsCloseMatcher(
      T const& comparator,
      double atol = 1.0e-8,
      double rtol = 1.0e-5)
      : m_comparator(comparator), m_atol(atol), m_rtol(rtol) {}

  bool match(T const& v) const override {
    using namespace Catch::Matchers::Floating;
    if (std::abs(v - m_comparator)
        > (m_atol + m_rtol * std::abs(m_comparator))) {
      return false;
    }
    return true;
  }
  std::string describe() const override {
    return "is approx: " + ::Catch::Detail::stringify(m_comparator);
  }

  T m_comparator;
  double m_atol;
  double m_rtol;
};

template <typename T>
IsCloseMatcher<T>
IsClose(T const& comparator, double atol = 1.0e-8, double rtol = 1.0e-5) {
  return IsCloseMatcher<T>(comparator, atol, rtol);
}

template <typename T>
struct AllCloseVectorMatcher : Catch::MatcherBase<T> {
  AllCloseVectorMatcher(
      T const& comparator,
      double atol = 1.0e-8,
      double rtol = 1.0e-5)
      : m_comparator(comparator), m_atol(atol), m_rtol(rtol) {}

  bool match(T const& v) const override {
    using namespace Catch::Matchers::Floating;
    if (m_comparator.size() != v.size()) {
      return false;
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
      if (!IsClose(m_comparator[i], m_atol, m_rtol).match(v[i])) {
        return false;
      }
    }
    return true;
  }
  std::string describe() const override {
    return "is approx: " + ::Catch::Detail::stringify(m_comparator);
  }

  T const& m_comparator;
  double m_atol;
  double m_rtol;
};

template <typename T>
AllCloseVectorMatcher<T> AllCloseVector(
    T const& comparator,
    double atol = 1.0e-8,
    double rtol = 1.0e-5) {
  return AllCloseVectorMatcher<T>(comparator, atol, rtol);
}

template <typename T>
struct AllCloseMatrixMatcher : Catch::MatcherBase<T> {
  AllCloseMatrixMatcher(
      T const& comparator,
      double atol = 1.0e-8,
      double rtol = 1.0e-5)
      : m_comparator(comparator), m_atol(atol), m_rtol(rtol) {}

  bool match(T const& v) const override {
    if ((m_comparator.rows() != v.rows()) && (m_comparator.cols() != v.cols()))
      return false;
    for (int i = 0; i < v.rows(); ++i) {
      for (int j = 0; j < v.cols(); ++j) {
        if (!IsClose(m_comparator(i, j), m_atol, m_rtol).match(v(i, j))) {
          return false;
        }
      }
    }
    return true;
  }
  std::string describe() const override {
    return "is approx: " + ::Catch::Detail::stringify(m_comparator);
  }

  T const& m_comparator;
  double m_atol;
  double m_rtol;
};

template <typename T>
AllCloseMatrixMatcher<T> AllCloseMatrix(
    T const& comparator,
    double atol = 1.0e-8,
    double rtol = 1.0e-5) {
  return AllCloseMatrixMatcher<T>(comparator, atol, rtol);
}

#endif  // RBDL_RBDL_TESTS_H
