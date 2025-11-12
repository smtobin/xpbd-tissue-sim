#!/bin/bash

THIRDPARTY_FOLDER=$1

# set thirdparty folder
export XPBD_SIM_THIRDPARTY_DIR=$THIRDPARTY_FOLDER

# set append prefix path for Easy3D
export XPBD_SIM_EASY3D_CMAKE_PREFIX_PATH=$(realpath $THIRDPARTY_FOLDER/Easy3D/build)

export XPBD_SIM_SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

export XPBD_SIM_BASE_DIR=$(realpath $XPBD_SIM_SCRIPT_DIR/..)

# needed so that we can find the Easy3D libraries
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/easy3d-2.6.1/lib/

# needed for OpenHaptics SDK
export GTDD_HOME=/root/.3dsystems
