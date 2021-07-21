#include "rbdl/Logging.h"
#include "rbdl/rbdl_errors.h"
#include <iostream>
#include <exception>

#include "rbdl_tests.h"

using namespace RigidBodyDynamics::Errors;

// Test if each error type can be catched by Base Class
TEST_CASE ( __FILE__"_BaseClassCatch", "" )
{
  //RBDLError
  try {
    throw RBDLError("Test RBDLError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLInvalidParameterError
  try {
    throw RBDLInvalidParameterError("Test RBDLInvalidParameterError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLSizeMismatchError
  try {
    throw RBDLSizeMismatchError("Test RBDLSizeMismatchError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLDofMismatchError
  try {
    throw RBDLDofMismatchError("Test RBDLDofMismatchError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLMissingImplementationError
  try {
    throw RBDLMissingImplementationError("Test RBDLMissingImplementationError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLInvalidFileError
  try {
    throw RBDLInvalidFileError("Test RBDLInvalidFileError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLFileParseError
  try {
    throw RBDLFileParseError("Test RBDLFileParseError");
  } catch (RBDLFileParseError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

// Tests to check catching each error type

TEST_CASE (__FILE__"_RBDLInvalidParameterErrorTest", "" )
{
  try {
    throw RBDLInvalidParameterError("Test RBDLInvalidParameterError");
  } catch (RBDLInvalidParameterError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

TEST_CASE (__FILE__"_RBDLSizeMismatchErrorTest", "" )
{
  try {
    throw RBDLSizeMismatchError("Test RBDLSizeMismatchError");
  } catch (RBDLSizeMismatchError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

TEST_CASE (__FILE__"_RBDLDofMismatchErrorTest", "" )
{
  try {
    throw RBDLDofMismatchError("Test RBDLDofMismatchError");
  } catch (RBDLDofMismatchError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

TEST_CASE (__FILE__"_RBDLMissingImplementationErrorTest", "")
{
  try {
    throw RBDLMissingImplementationError("Test RBDLMissingImplementationError");
  } catch (RBDLMissingImplementationError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

TEST_CASE (__FILE__"_RBDLInvalidFileErrorTest", "")
{
  try {
    throw RBDLInvalidFileError("Test RBDLInvalidFileError");
  } catch (RBDLInvalidFileError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

TEST_CASE (__FILE__"_RBDLFileParseErrorTest", "" )
{
  try {
    throw RBDLFileParseError("Test RBDLFileParseError");
  } catch (RBDLFileParseError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}
