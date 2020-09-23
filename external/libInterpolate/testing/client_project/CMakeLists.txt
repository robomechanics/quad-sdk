cmake_minimum_required(VERSION 3.10)
project(libInterpolatePackageTest CXX)

find_package(libInterp REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example libInterp::Interp)
