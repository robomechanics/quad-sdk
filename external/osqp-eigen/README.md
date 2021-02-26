<p align="center">
<h1 align="center">osqp-eigen</h1>
</p>

<p align="center">
<a href="https://isocpp.org"><img src="https://img.shields.io/badge/standard-C++14-blue.svg?style=flat&logo=c%2B%2B" alt="C++ Standard"/></a>
<a href="./LICENSE"><img src="https://img.shields.io/badge/license-LGPL-19c2d8.svg" alt="Size" /></a>
<a href="https://www.codacy.com/manual/GiulioRomualdi/osqp-eigen?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=robotology/osqp-eigen&amp;utm_campaign=Badge_Grade"><img src="https://api.codacy.com/project/badge/Grade/a73c260e38d949eabeecc424410d859c"/></a>
<a href="https://github.com/robotology/osqp-eigen/workflows/C++%20CI%20Workflow/badge.svg"><img src="https://github.com/robotology/osqp-eigen/workflows/C++%20CI%20Workflow/badge.svg"/></a>
</p>

Simple C++ wrapper for [osqp](http://osqp.readthedocs.io/en/latest/index.html) library.


## Dependeces
- [osqp](http://osqp.readthedocs.io/en/latest/index.html) of course :smile:;
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page);
- [cmake](https://cmake.org/);
- [Catch2](https://github.com/catchorg/Catch2) (only for testing).

## Build the library and the application
### Linux / macOs
```sh
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ../
make
[sudo] make install
```
If you want to enable tests set the `BUILD_TESTING` option to `ON`.

**Notice**: ``sudo`` is not necessary if you specify the ``CMAKE_INSTALL_PREFIX``. In this case it is necessary to add in the ``.bashrc`` the following lines:

### Linux / macOs
```
export OsqpEigen_DIR=/path/where/you/installed/
```

**Notice**:  The choice of `OsqpEigen_DIR` name for the environment variable is not random. Indeed `<package>_DIR` is one of the search paths of [`find_package()`](https://cmake.org/cmake/help/v3.0/command/find_package.html).

## How to use the wrapper
[Here](./example/) you can find a simple example that shows how to use this library.

## API
[Here](https://robotology.github.io/osqp-eigen/) you can find the documentation.

## Bug reports and support
All types of [issues](https://github.com/robotology/osqp-eigen/issues/new) are welcome :smile:.
