#! /bin/bash

root=$(git rev-parse --show-toplevel)
test_dir="${root}/_test-build.d"

function exit_on_error()
{
  echo "There was an error. The test directory (${test_dir}) will not be removed."
}

trap exit_on_error ERR

set -e

cd "$root"
mkdir "${test_dir}"
cd "${test_dir}"


echo "Testing CMake-based Build"
mkdir build
cd build
cmake "${root}"
cmake --build .
cd ..
rm build -r


echo "Testing conan-based Build"
mkdir build
cd build
conan install "${root}"
conan build "${root}"
cd ..
rm build -r

cd "$root"
rm -rf "${test_dir}"



