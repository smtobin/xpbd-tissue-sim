#!/bin/bash

# make sure the script has root
# if (( $EUID != 0 )); then
#     echo "This script requires root access."
#     sudo -k
#     SUDO='sudo'
# fi

# install required packages
sudo apt-get update && sudo apt-get install -y \
    g++ \
    git \
    cmake \
    xorg-dev \
    libassimp-dev \
    wget \
    gdb \
    # needed by OpenHaptics API
    libncurses-dev \
    freeglut3-dev \
    build-essential \
    libusb-1.0-0-dev \

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) # scripts directory
BASE_DIR=$(realpath $SCRIPT_DIR/..) # repo base directory

# create the 'dependencies' folder for the external dependencies
cd $BASE_DIR
rm -rf dependencies # remove the 'dependencies' directory and all its contents before reinstalling all the libraries
mkdir dependencies  # create the 'dependencies' directory
mkdir dependencies/src  # create the src directory (where we will clone the repos)
mkdir dependencies/install # create the install directory (where the repos will be installed to)

DEPS_DIR=$BASE_DIR/dependencies
DEPS_SRC_DIR=$DEPS_DIR/src
DEPS_INSTALL_DIR=$DEPS_DIR/install

##########################
# Embree install
##########################
# packages needed for Embree
sudo apt-get update && sudo apt-get -y install cmake-curses-gui \
    libtbb-dev \
    libglfw3-dev

cd $DEPS_SRC_DIR
git clone https://github.com/RenderKit/embree.git
mkdir embree/build
cd embree/build
cmake -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_DIR/embree -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF .. 
make -j12
make install

########################
# YAML-cpp
########################
cd $DEPS_SRC_DIR
git clone https://github.com/jbeder/yaml-cpp.git
mkdir yaml-cpp/build
cd yaml-cpp/build
cmake -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_DIR/yaml-cpp -DCMAKE_BUILD_TYPE=Release .. 
make -j12
make install

#########################
# Eigen install
#########################
cd $DEPS_SRC_DIR
git clone https://gitlab.com/libeigen/eigen.git
mkdir eigen/build
cd eigen/build
cmake -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_DIR/eigen -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF .. 
make -j12
make install

###########################
# Easy3D install
###########################
cd $DEPS_SRC_DIR
git clone https://github.com/LiangliangNan/Easy3D.git
mkdir Easy3D/build
cd Easy3D/build
cmake -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_DIR/Easy3D -DCMAKE_BUILD_TYPE=Release .. 
make -j12
make install

#########################
# Mesh2SDF install
#########################
cd $DEPS_SRC_DIR
git clone https://github.com/smtobin/Mesh2SDF.git
mkdir Mesh2SDF/build
cd Mesh2SDF/build
cmake -DUSE_DOUBLE_PRECISION=1 -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_DIR/Mesh2SDF .. 
make -j12
make install

##########################
# VTK install
##########################
cd $DEPS_SRC_DIR
git clone https://gitlab.kitware.com/vtk/vtk.git
mkdir vtk/build
cd vtk/build
cmake -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_DIR/vtk -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF .. 
make -j12
make install

########################
# GMSH install
########################
cd $DEPS_SRC_DIR
git clone https://gitlab.onelab.info/gmsh/gmsh.git
mkdir gmsh/build
cd gmsh/build
# install from source as a dynamic library for access to C++ API
cmake -DCMAKE_INSTALL_PREFIX=$DEPS_INSTALL_DIR/gmsh -DENABLE_BUILD_DYNAMIC=1 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF ..
make -j12
make install


################################## 
# Geomagic Touch device setup
##################################

# install OpenHaptics drivers
cd $DEPS_SRC_DIR
wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz
tar -xvf openhaptics_3.4-0-developer-edition-amd64.tar.gz
wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver_2023_11_15.tgz
tar -xvf TouchDriver_2023_11_15.tgz

# copied from OpenHaptics ./install script
cd $DEPS_DIR
cd openhaptics_3.4-0-developer-edition-amd64
sudo cp -R opt/* /opt                                                                                                                                             
sudo cp -R usr/lib/* /usr/lib                                                                                                                                     
sudo cp -R usr/include/* /usr/include                                                                                                                                                                        
sudo ln -sfn /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so.3.4                                                                                                                  
sudo ln -sfn /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so                                                                                                                                                                                                                                                                                          
sudo ln -sfn /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so.3.4                                                                                                                  
sudo ln -sfn /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so                                                                                                                                                                                                                                                                                              
sudo ln -sfn /usr/lib/libQH.so.3.4.0 /usr/lib/libQH.so.3.4                                                                                                                  
sudo ln -sfn /usr/lib/libQH.so.3.4.0 /usr/lib/libQH.so                                                                                                                    
sudo ln -sfn /usr/lib/libQHGLUTWrapper.so.3.4.0 /usr/lib/libQHGLUTWrapper.so.3.4                                                                                            
sudo ln -sfn /usr/lib/libQHGLUTWrapper.so.3.4.0 /usr/lib/libQHGLUTWrapper.so                                                                                                                                                                                                                                               
sudo chmod -R 777 /opt/OpenHaptics
export OH_SDK_BASE=/opt/OpenHaptics/Developer/3.4-0

# follow driver installation instructions
cd $DEPS_SRC_DIR/TouchDriver_2023_11_15
sudo bash ./install_haptic_driver

# make symbolic links so API examples build
sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.so /usr/lib/libglut.so.3
sudo ln -s /usr/lib/x86_64-linux-gnu/libncurses.so.6 /usr/lib/libncurses.so.5
sudo ln -s /usr/lib/x86_64-linux-gnu/libtinfo.so.6 /usr/lib/libtinfo.so.5