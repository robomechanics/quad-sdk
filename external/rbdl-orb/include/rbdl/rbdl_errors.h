/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Felix Richter <judge@felixrichter.tech>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */
#ifndef RBDL_ERRORS_H
#define RBDL_ERRORS_H

#include <string>
#include <exception>
#include "rbdl/rbdl_config.h"

namespace RigidBodyDynamics
{

/** Namespace that contains error classes that may be thrown by RBDL */
namespace Errors
{

/** \brief Base class for all RBDL exceptions 
 *
 * The base class for all errors thrown in RBDL.
 * It saves an error message that then can be read when catching the 
 * error in the calling code, by calling the what() function. 
 * 
 * When catching for this class you will also be catching any other error
 * types derived from this class.
 * 
 * To add another error type just subclass this class, you only need to implement
 * the constructor and call the parent constructor with your error message.
 * 
 */
class RBDL_DLLAPI RBDLError : public std::exception
{
protected:
  std::string text;
public:
  RBDLError(std::string text);
  virtual const char* what() const noexcept;
};


/** \brief Thrown if parameter of function was faulty
 * 
 * This error class is thrown if a parameter supplied to a function can not be used. This may have many reasons,
 * details should be read from the error message.
 * 
 * Example of error thrown in addons/geometry/SmoothSegmentedFunction.cc: 
 * \snippet{lineno} addons/geometry/SmoothSegmentedFunction.cc Invalid Parameter
 */
class RBDL_DLLAPI RBDLInvalidParameterError : public RBDLError
{
public:
  RBDLInvalidParameterError(std::string text);
};


/** \brief Thrown if there is a size mismatch that prevents further calculations
 * 
 * This error class is thrown if the operads of a calculation have incompatible sizes. This mostly
 * happens when matrices are of different sizes than expected.
 * 
 * Example of error thrown in addons/geometry/SmoothSegmentedFunction.cc: 
 * \snippet{lineno} addons/geometry/SmoothSegmentedFunction.cc Size Mismatch
 */
class RBDL_DLLAPI RBDLSizeMismatchError : public RBDLError
{
public:
  RBDLSizeMismatchError(std::string text);
};


/** \brief Thrown if there is a dof mismatch
 * 
 * This error class is thrown if the parameter supplied does not match the supported dof of the used model or
 * the supplied dof is not supported by the function used!
 * 
 * Example of error thrown in addons/geometry/SmoothSegmentedFunction.cc: 
 * \snippet{lineno} addons/geometry/SmoothSegmentedFunction.cc Dof Mismatch
 */
class RBDL_DLLAPI RBDLDofMismatchError : public RBDLError
{
public:
  RBDLDofMismatchError(std::string text);
};


/** \brief Thrown if code path reaches a point that is not implmented yet
 * 
 * This error is thrown if RBDL is missing an implementation for your specific use case!
 * 
 * Example of error thrown in addons/muscle/Millard2016TorqueMuscle.cc: 
 * \snippet{lineno} addons/muscle/Millard2016TorqueMuscle.cc Missing Implementation
 */
class RBDL_DLLAPI RBDLMissingImplementationError: public RBDLError
{
public:
  RBDLMissingImplementationError(std::string text);
};

/** \brief Thrown if the specified file could not be found/read.
 * 
 * This error is thrown if the file could not be found or read.
 * 
 * Example of error thrown in addons/urdfreader/urdfreader.cc: 
 * \snippet{lineno} addons/urdfreader/urdfreader.cc Invalid File
 */
class RBDL_DLLAPI RBDLInvalidFileError : public RBDLError
{
public:
  RBDLInvalidFileError(std::string text);
};


/** \brief Thrown if the file beeing read contains errors.
 * 
 * This error is thrown if the file that is beeing read contains formatting or other errors, that cause 
 * the program to not beeing able to use the file as expected.
 * 
 * Example of error thrown in addons/luamodel/luamodel.cc: 
 * \snippet{lineno} addons/luamodel/luamodel.cc Parse Failed
 */
class RBDL_DLLAPI RBDLFileParseError : public RBDLError
{
public:
  RBDLFileParseError(std::string text);
};

}
}

#endif

