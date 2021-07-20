/** \file python_example.h 
 * \page PythonExample Python API example
 *
 * Here is a simple example of the Python API that showcases a subset of
 * the wrapped functions and how to access them from python:
 *
 * \include example.py
 * 
 * To build the wrapper one needs both <a
 * href="http://cython.org/">Cython</a> and the development libraries for
 * <a href="http://www.numpy.org/">NumPy</a> installed on the system. If
 * this is the case you can build the wrapper by enabling the CMake option
 * RBDL_BUILD_PYTHON_WRAPPER when configuring RBDL. You can find the
 * wrapper in the subdirectory <tt>python/</tt> of your build directory. 
 * By running the python interpreter in this directory you can load it
 * within the python shell using
 *
 * \code
 * Python 2.7.11+ (default, Apr 17 2016, 14:00:29) 
 * [GCC 5.3.1 20160413] on linux2
 * Type "help", "copyright", "credits" or "license" for more information.
 * >>> import rbdl
 * \endcode
 *
 * To install the wrapper you can use the <tt>python/setup.py</tt> script:
 * \code
 * sudo ./setup.py install
 * \endcode
 *
 * This installs the RBDL python module globally and allows you to import
 * it from any python script. 
 */
