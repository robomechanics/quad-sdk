/** \file example.h 
 * \page LuaModelExample LuaModel example
 *
 * Here is an example on how to use the \ref luamodel_introduction Addon both
 * for loading the models from C++ code and also how the model file looks
 * like.
 *
 * Using Lua as a description format for models instead of C++ code has
 * numerous advantages such as simplified modeling, less C++ compilations,
 * and easier exchange of models between multiple users. As the file format
 * uses Lua code it is easy to create parameterized models.
 *
 * To be able to use the addon one has to enable the addon in CMake by
 * setting BUILD_ADDON_LUAMODEL to true. 
 *
 * Here is an example how \ref luamodel_introduction can be loaded by the RBDL:
 *
 * \include example_luamodel.cc
 * 
 * If the library itself is already created, one can compile this example
 * with CMake. In the example folder is a CMakeLists.txt file, that can be
 * used to automatically create the makefiles by using <a
 * href="http://www.cmake.org">CMake</a>.  It uses the script
 * FindRBDL.cmake which can be used to find the library and include
 * directory of the headers.
 *
 * The documentation about how to write the lua models can be found
 * \ref luamodel_introduction "here".
 *
 * Here is an example:
 *
 * \include samplemodel.lua
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
