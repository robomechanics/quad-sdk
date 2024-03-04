//
// Copyright (c) 2022 INRIA
//
/**
 * @file version.hpp
 */

#ifndef PROXSUITE_HELPERS_VERSION_HPP
#define PROXSUITE_HELPERS_VERSION_HPP

#include "proxsuite/config.hpp"
#include <string>
#include <sstream>

namespace proxsuite {
namespace helpers {

inline std::string
printVersion(const std::string& delimiter = ".")
{
  std::ostringstream oss;
  oss << PROXSUITE_MAJOR_VERSION << delimiter << PROXSUITE_MINOR_VERSION
      << delimiter << PROXSUITE_PATCH_VERSION;
  return oss.str();
}

inline bool
checkVersionAtLeast(signed int major_version,
                    signed int minor_version,
                    signed int patch_version)
{
  return PROXSUITE_VERSION_AT_LEAST(
    major_version, minor_version, patch_version);
}

} // helpers
} // proxsuite

#endif // ifndef PROXSUITE_HELPERS_VERSION_HPP
