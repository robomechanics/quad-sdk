#!/usr/bin/env bash

# Exit script if you try to use an uninitialized variable.
set -o nounset

# Exit script if a statement returns a non-true return value.
set -o errexit

# Use the error status of the first failure, rather than that of the last item in a pipeline.
set -o pipefail

cd ../
export PATH=$PATH:~/.local/bin/ 
cpplint --quiet --filter=-legal,-runtime,-build,-readability/casting,-readability/fn_size --extensions=cpp,h,hpp --recursive  --exclude=external/* --exclude=nmpc_controller/src/gen/ --exclude=nmpc_controller/include/nmpc_controller/gen/ ./
