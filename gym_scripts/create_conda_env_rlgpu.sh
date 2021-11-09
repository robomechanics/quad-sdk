#!/bin/bash

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# should match env name from YAML
ENV_NAME=rlgpu

pushd "${ROOT_DIR}/python"

    # setup conda
    CONDA_DIR="$(conda info --base)"
    source "${CONDA_DIR}/etc/profile.d/conda.sh"

    # deactivate the env, if it is active
    ACTIVE_ENV_NAME="$(basename ${CONDA_PREFIX})"
    if [ "${ENV_NAME}" = "${ACTIVE_ENV_NAME}" ]; then
        conda deactivate
    fi

    # !!! this removes existing version of the env
    conda remove -y -n "${ENV_NAME}" --all

    # create the env from YAML
    conda env create -f ./rlgpu_conda_env.yml
    if [ $? -ne 0 ]; then
        echo "*** Failed to create env"
        exit 1
    fi

    # activate env
    conda activate "${ENV_NAME}"
    if [ $? -ne 0 ]; then
        echo "*** Failed to activate env"
        exit 1
    fi

    # double check that the correct env is active
    ACTIVE_ENV_NAME="$(basename ${CONDA_PREFIX})"
    if [ "${ENV_NAME}" != "${ACTIVE_ENV_NAME}" ]; then
        echo "*** Env is not active, aborting"
        exit 1
    fi

    # install isaacgym package
    pip install -e .

popd

echo "SUCCESS"
