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

# navigate to the install folder provided by the user
THIRDPARTY_FOLDER=$1
cd $THIRDPARTY_FOLDER

########################
# YAML-cpp
########################
cd $THIRDPARTY_FOLDER
git clone https://github.com/jbeder/yaml-cpp.git
mkdir yaml-cpp/build
cd yaml-cpp/build
cmake ..
make -j12
sudo make install

#########################
# Eigen install
#########################
cd $THIRDPARTY_FOLDER
git clone https://gitlab.com/libeigen/eigen.git
mkdir eigen/build
cd eigen/build
cmake ..
make -j12
sudo make install

###########################
# Easy3D install
###########################
cd $THIRDPARTY_FOLDER
git clone https://github.com/LiangliangNan/Easy3D.git
mkdir Easy3D/build
cd Easy3D/build
cmake ..
make -j12
sudo make install
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/easy3d-2.6.1/lib/

########################
# GMSH install
########################
cd $THIRDPARTY_FOLDER
git clone https://gitlab.onelab.info/gmsh/gmsh.git
mkdir gmsh/build
cd gmsh/build
# install from source as a dynamic library for access to C++ API
cmake -DENABLE_BUILD_DYNAMIC=1 ..
make -j12
sudo make install

#########################
# Mesh2SDF install
#########################
cd $THIRDPARTY_FOLDER
git clone https://github.com/smtobin/Mesh2SDF.git
mkdir Mesh2SDF/build
cd Mesh2SDF/build
cmake .. -DUSE_DOUBLE_PRECISION=1
make -j12
sudo make install

##########################
# Embree install
##########################
# packages needed for Embree
sudo apt-get update && sudo apt-get -y install cmake-curses-gui \
    libtbb-dev \
    libglfw3-dev

cd $THIRDPARTY_FOLDER
git clone https://github.com/RenderKit/embree.git
mkdir embree/build
cd embree/build
cmake ..
make -j12
sudo make install

##########################
# VTK install
##########################
cd $THIRDPARTY_FOLDER
git clone https://gitlab.kitware.com/vtk/vtk.git
mkdir vtk/build
cd vtk/build
cmake ..
make -j12
sudo make install

#########################
# PETSc install
#########################
cd $THIRDPARTY_FOLDER
git clone -b release https://gitlab.com/petsc/petsc.git petsc
cd petsc
./configure --with-cc=gcc --with-cxx=g++ --with-fc=gfortran --download-f2cblaslapack --download-mpich
make PETSC_DIR=$THIRDPARTY_FOLDER/petsc PETSC_ARCH=arch-linux-c-debug all


################################## 
# Geomagic Touch device setup
##################################

# install OpenHaptics drivers
cd $THIRDPARTY_FOLDER
wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz
tar -xvf openhaptics_3.4-0-developer-edition-amd64.tar.gz
wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver_2023_11_15.tgz
tar -xvf TouchDriver_2023_11_15.tgz

# copied from OpenHaptics ./install script
cd $THIRDPARTY_FOLDER
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
cd $THIRDPARTY_FOLDER/TouchDriver_2023_11_15
sudo bash ./install_haptic_driver

# make symbolic links so API examples build
sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.so /usr/lib/libglut.so.3
sudo ln -s /usr/lib/x86_64-linux-gnu/libncurses.so.6 /usr/lib/libncurses.so.5
sudo ln -s /usr/lib/x86_64-linux-gnu/libtinfo.so.6 /usr/lib/libtinfo.so.5