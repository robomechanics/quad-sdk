#! /bin/bash

root=$(git rev-parse --show-toplevel)
test_dir="${root}/_test-client.d"

function exit_on_error()
{
  echo "There was an error. The test directory (${test_dir}) will not be removed."
}

trap exit_on_error ERR

set -e

cd "$root"
mkdir "${test_dir}"
cd "${test_dir}"
install_dir=$PWD/install
lib_build_dir=$PWD/build_lib
client_build_dir=$PWD/build_client
mkdir "${install_dir}" "${lib_build_dir}" "${client_build_dir}"


echo "Testing CMake-based Install"
cd "${lib_build_dir}"
cmake ${root} -DCMAKE_INSTALL_PREFIX="${install_dir}"
cmake --build .
cmake --build . --target install
cd "${client_build_dir}"
cmake ${root}/testing/client_project/ -DCMAKE_INSTALL_PREFIX="${install_dir}"
cmake --build .
./example
cd "$root"
rm -rf "${test_dir}"



