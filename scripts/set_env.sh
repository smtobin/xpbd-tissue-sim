#!/bin/bash

THIRDPARTY_FOLDER=$1

# set thirdparty folder
export XPBD_SIM_THIRDPARTY_DIR=$THIRDPARTY_FOLDER

# set append prefix path for Easy3D
export XPBD_SIM_EASY3D_CMAKE_PREFIX_PATH=$(realpath $THIRDPARTY_FOLDER/Easy3D/build)

export XPBD_SIM_SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export XPBD_SIM_BASE_DIR=$(realpath $XPBD_SIM_SCRIPT_DIR/..)

# needed for OpenHaptics SDK
export GTDD_HOME=/root/.3dsystems

# set PETSc location
export PETSC_DIR=$(realpath $THIRDPARY_FOLDER/petsc)
export PETSC_ARCH=arch-linux-c-debug
