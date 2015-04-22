#!/bin/bash

# written by Maksym Figat

PROGRAMS_DIRECTORY="/home/nao/programs"
MY_WORKSPACE_SRC_DIR="/home/nao/my_workspace/src"
MY_WORKSPACE_INSTALL_ISOLATED="/home/nao/my_workspace/install_isolated"

if [ -d $PROGRAMS_DIRECTORY ]; then #If direcotory exists
	echo "Folder $PROGRAMS_DIRECTORY exists"
else
	mkdir -p PROGRAMS_DIRECTORY
	echo "Creating $PROGRAMS_DIRECTORY folder"
fi

if [ -d $MY_WORKSPACE_INSTALL_ISOLATED ]; then #If direcotory exists
	echo "Folder $MY_WORKSPACE_INSTALL_ISOLATED exists"
else
	mkdir -p MY_WORKSPACE_INSTALL_ISOLATED
	echo "Creating $MY_WORKSPACE_INSTALL_ISOLATED folder"
fi

# Yaml-cpp
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of yaml-cpp"
wget https://launchpad.net/ubuntu/+archive/primary/+files/yaml-cpp_0.5.1.orig.tar.gz
tar zxvf yaml-cpp_0.5.1.orig.tar.gz
cd yaml-cpp-0.5.1
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$MY_WORKSPACE_INSTALL_ISOLATED
make install

# Bigloo
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of Bigloo"
wget ftp://ftp-sop.inria.fr/indes/fp/Bigloo/bigloo4.2a-alpha31Mar15.tar.gz
tar zxvf bigloo4.2a-alpha31Mar15.tar.gz
cd bigloo4.2a
./configure --prefix=/home/nao/my_workspace/install_isolated
make install

# Hop
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of Hop"
wget ftp://ftp-sop.inria.fr/indes/fp/Hop/hop-3.0.0-pre14.tar.gz
tar zxvf hop-3.0.0-pre14.tar.gz
cd hop-3.0.0-pre14
./configure --prefix=/home/nao/my_workspace/install_isolated
make install

# Gsasl
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of Gsasl"
wget ftp://ftp.gnu.org/gnu/gsasl/libgsasl-1.8.0.tar.gz
tar zxvf libgsasl-1.8.0.tar.gz
cd libgsasl-1.8.0
./configure --prefix=/home/nao/my_workspace/install_isolated
make install

# Vmime
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of libvmime library"
wget http://sourceforge.net/projects/vmime/files/vmime/0.9/libvmime-0.9.1.tar.bz2
tar -xjf libvmime-0.9.1.tar.bz2
./configure --prefix=/home/nao/my_workspace/install_isolated
make install

cd $MY_WORKSPACE_SRC_DIR
echo "Downloading source code from image_common repository"
git clone https://github.com/ros-perception/image_common.git
echo "Downloading source code from nodelet_core repository"
git clone https://github.com/ros/nodelet_core.git
echo "Downloading source code from bond_core repository"
git clone https://github.com/ros/bond_core.git
echo "Downloading source code from cmake_modules repository"
git clone https://github.com/ros/cmake_modules.git
echo "Downloading source code from vision_opencv repository"
git clone https://github.com/ros-perception/vision_opencv.git
echo "Downloading source code from rosbridge_suite repository"
git clone https://github.com/RobotWebTools/rosbridge_suite.git

cd ..
echo "Building ros packages"
/home/nao/my_workspace/src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release



