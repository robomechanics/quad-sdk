/** \file example.h 
 * \page SimpleExample A simple example
 *
 * Here is a simple example how one can create a meaningless model and
 * compute the forward dynamics for it:
 *
 * \include example.cc
 * 
 * If the library itself is already created, one can compile this example
 * with CMake. In the example folder is a CMakeLists.txt file, that can be
 * used to automatically create the makefiles by using <a
 * href="http://www.cmake.org">CMake</a>.  It uses the script
 * FindRBDL.cmake which can be used to find the library and include
 * directory of the headers.
 *
 * The FindRBDL.cmake script can use the environment variables RBDL_PATH,
 * RBDL_INCLUDE_PATH, and RBDL_LIBRARY_PATH to find the required files.
 *
 * To build it manually you have to specify the following compiler and
 * linking switches:
 * \code
 * 	g++ example.cc -I<path to build folder>/src -I<path to src folder> -lrbdl -L<path to librbdl.a> -o example
 * \endcode
 *
 */
