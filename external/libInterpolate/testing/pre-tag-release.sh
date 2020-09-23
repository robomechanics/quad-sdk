#! /bin/bash

set -e
root=$(git rev-parse --show-toplevel)

cd $root

bindir="$PWD/build-and-test"
function cleanup()
{
  rm -r $bindir
}
trap cleanup EXIT

function copy_bindir()
{
  cp -r $bindir $bindir.error
}
trap copy_bindir ERR

echo "Checking that project can be installed."
mkdir $bindir
cd $bindir
conan install .. -g cmake_paths
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_paths.cmake -DCMAKE_INSTALL_PREFIX=$bindir/install
cmake --build .
cmake --build . --target test

# test install
cmake --build . --target install





echo "Checking that installed project can be detected and used."
mkdir app
cd app

cat << EOF > main.cpp
#include <iostream>
#include <libInterpolate/Interpolate.hpp>
#include <libInterpolate/libInterpolate_version.h>

int main()
{
  std::cout << "VERSION: " << libInterpolate_VERSION << std::endl;
  std::cout << "FULL VERSION: " << libInterpolate_VERSION_FULL << std::endl;

  _1D::LinearInterpolator<double> i1;

  return 0;
}
EOF

cat << EOF > CMakeLists.txt
cmake_minimum_required(VERSION 3.1)
add_executable( main main.cpp )
find_package( libInterpolate REQUIRED )
target_link_libraries(main libInterpolate::Interpolate )
set_target_properties(main PROPERTIES CXX_STANDARD 11)
EOF

mkdir build1
cd build1
conan install ${root} -g cmake_paths
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_paths.cmake -DlibInterpolate_DIR=${bindir}/install/cmake/
cmake --build .
./main

cd ..

cat << EOF > CMakeLists.txt
cmake_minimum_required(VERSION 3.1)
add_executable( main main.cpp )
find_package( libInterpolate REQUIRED PATHS ${bindir}/install )
target_link_libraries(main libInterpolate::Interpolate )
#set_target_properties(main PROPERTIES CXX_STANDARD 11)
EOF

mkdir build2
cd build2
conan install ${root} -g cmake_paths
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_paths.cmake
cmake --build .
./main

cd ..







# LEGACY
# these tests are to make sure that the old cmake targets and header
# includes still work after the name change. they will be removed
# after the next major version
echo "Checking that installed project can be detected and used with libInterp::Interp target."
mkdir app
cd app

cat << EOF > main.cpp
#include <iostream>
#include <Interp.hpp>

int main()
{
  std::cout << "VERSION: " << libInterp_VERSION << std::endl;
  std::cout << "FULL VERSION: " << libInterp_VERSION_FULL << std::endl;

  _1D::LinearInterpolator<double> i1;

  return 0;
}
EOF

cat << EOF > CMakeLists.txt
cmake_minimum_required(VERSION 3.1)
add_executable( main main.cpp )
find_package( libInterp REQUIRED )
target_link_libraries(main libInterp::Interp )
set_target_properties(main PROPERTIES CXX_STANDARD 11)
EOF

mkdir build1
cd build1
conan install ${root} -g cmake_paths
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_paths.cmake -DlibInterp_DIR=${bindir}/install/cmake/
cmake --build .
./main

cd ..

cat << EOF > CMakeLists.txt
cmake_minimum_required(VERSION 3.1)
add_executable( main main.cpp )
find_package( libInterp REQUIRED PATHS ${bindir}/install )
target_link_libraries(main libInterp::Interp )
set_target_properties(main PROPERTIES CXX_STANDARD 11)
EOF

mkdir build2
cd build2
conan install ${root} -g cmake_paths
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_paths.cmake
cmake --build .
./main

cd ..

echo "Checking that project can be included with add_subdirectory libInterp::Interp target."
ln -s $root libInterp
cat << EOF > CMakeLists.txt
cmake_minimum_required(VERSION 3.1)
add_executable( main main.cpp )
add_subdirectory( libInterp )
target_link_libraries(main libInterp::Interp )
set_target_properties(main PROPERTIES CXX_STANDARD 11)
EOF

mkdir build3
cd build3
conan install ${root} -g cmake_paths
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_paths.cmake
cmake --build .
./main

cd ..

echo "PASSED"

exit 0
