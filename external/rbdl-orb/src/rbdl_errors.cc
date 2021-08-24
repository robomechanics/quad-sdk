/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Felix Richter <judge@felixrichter.tech>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include "rbdl/rbdl_errors.h"

namespace RigidBodyDynamics
{
namespace Errors
{

RBDLError::RBDLError(std::string text): text(text) {}

const char* RBDLError::what() const noexcept
{
  return text.c_str();
}

RBDLFileParseError::RBDLFileParseError(std::string text): RBDLError(text) {}

RBDLDofMismatchError::RBDLDofMismatchError(std::string text): RBDLError(text) {}

RBDLSizeMismatchError::RBDLSizeMismatchError(std::string text): RBDLError(
    text) {}

RBDLMissingImplementationError::RBDLMissingImplementationError(
  std::string text): RBDLError(text) {}

RBDLInvalidFileError::RBDLInvalidFileError(std::string text): RBDLError(text) {}

RBDLInvalidParameterError::RBDLInvalidParameterError(std::string text):
  RBDLError(text) {}

}
}
