#!/bin/bash

export XPBD_SIM_SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export XPBD_SIM_BASE_DIR=$(realpath $XPBD_SIM_SCRIPT_DIR/..)

# set thirdparty folder
export XPBD_SIM_THIRDPARTY_DIR=$XPBD_SIM_BASE_DIR/external

# set append prefix path for Easy3D
export XPBD_SIM_EASY3D_CMAKE_PREFIX_PATH=$(realpath $XPBD_SIM_THIRDPARTY_DIR/Easy3D/build)

# needed so that we can find the Easy3D libraries
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/easy3d-2.6.1/lib/

# needed for OpenHaptics SDK
export GTDD_HOME=/root/.3dsystems

# set PETSc location
export PETSC_DIR=$(realpath $THIRDPARY_FOLDER/petsc)
export PETSC_ARCH=arch-linux-c-debug
