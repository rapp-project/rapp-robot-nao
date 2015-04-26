#!/bin/bash

# written by Maksym Figat

PROGRAMS_DIRECTORY="/home/nao/programs"
ROS_DIR="/home/nao/my_workspace"
MY_WORKSPACE_SRC_DIR=$ROS_DIR"/src"
MY_WORKSPACE_INSTALL_ISOLATED=$ROS_DIR"/install_isolated"
SCRIPTS_DIR="/home/nao/scripts"

cd /home/nao/my_workspace

if [ -d $MY_WORKSPACE_INSTALL_ISOLATED ]; then #If directory exists
	echo "Removing $ROS_DIR/install_isolated"
	rm install_isolated devel_isolated build_isolated -rf
fi

echo "Building ros packages"
src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

cp /usr/lib/liblog4cxx* install_isolated/lib/
cp -r /usr/include/log4cxx install_isolated/include/
cp /usr/lib/libapr* install_isolated/lib/
cp -r /usr/include/apr* install_isolated/include/
cp /usr/lib/libtinyxml* install_isolated/lib/
cp /usr/lib/libPoco* install_isolated/lib/
cp -r /usr/include/Poco* install_isolated/include/
cp -r /usr/lib/liburdfdom* install_isolated/lib/
cp -r /usr/include/urdf* install_isolated/include/
cp -r /usr/lib/libcxsparse* install_isolated/lib/
cp -r /usr/lib/libcholmod* install_isolated/lib/
cp -r /usr/include/cholmod* install_isolated/include/
cp -r /usr/lib/liblz4* install_isolated/lib/
cp -r /usr/include/lz4* install_isolated/include/
cp -r /usr/local/lib/libconsole_bridge* install_isolated/lib/
cp -r /usr/local/include/console_bridge* install_isolated/include/
cp -r /usr/lib/python2.7/site-packages/* install_isolated/lib/python2.7/site-packages/
cp /usr/bin/rosversion install_isolated/bin/
cp /usr/bin/ros* install_isolated/bin/

if [ -d $PROGRAMS_DIRECTORY ]; then #If directory exists
	echo "Folder $PROGRAMS_DIRECTORY exists"
	rm $PROGRAMS_DIRECTORY -rf
fi
mkdir -p $PROGRAMS_DIRECTORY
echo "Creating $PROGRAMS_DIRECTORY folder"

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

cd /home/nao/my_workspace/src
echo "Downloading source code from bond_core repository"
git clone https://github.com/ros/bond_core.git
echo "Downloading source code from cmake_modules repository"
git clone https://github.com/ros/cmake_modules.git
cd ..

echo "Building ros packages"
src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release


cd /home/nao/my_workspace/src
echo "Downloading source code from image_common repository"
git clone https://github.com/ros-perception/image_common.git
echo "Downloading source code from nodelet_core repository"
git clone https://github.com/ros/nodelet_core.git
echo "Downloading source code from vision_opencv repository"
git clone https://github.com/ros-perception/vision_opencv.git
echo "Downloading source code from rosbridge_suite repository"
git clone https://github.com/RobotWebTools/rosbridge_suite.git

cd ..

echo "Building ros packages"
src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

# Bigloo
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of Bigloo"
wget ftp://ftp-sop.inria.fr/indes/fp/Bigloo/bigloo4.2a-alpha31Mar15.tar.gz
tar zxvf bigloo4.2a-alpha31Mar15.tar.gz
cd bigloo4.2a
./configure --prefix=$MY_WORKSPACE_INSTALL_ISOLATED
make install

# Hop
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of Hop"
wget ftp://ftp-sop.inria.fr/indes/fp/Hop/hop-3.0.0-pre14.tar.gz
tar zxvf hop-3.0.0-pre14.tar.gz
cd hop-3.0.0-pre14
./configure --prefix=$MY_WORKSPACE_INSTALL_ISOLATED
make install

# Gsasl
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of Gsasl"
wget ftp://ftp.gnu.org/gnu/gsasl/libgsasl-1.8.0.tar.gz
tar zxvf libgsasl-1.8.0.tar.gz
cd libgsasl-1.8.0
./configure --prefix=$MY_WORKSPACE_INSTALL_ISOLATED
make install

# Vmime
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of libvmime library"
wget http://sourceforge.net/projects/vmime/files/vmime/0.9/libvmime-0.9.1.tar.bz2
tar -xjf libvmime-0.9.1.tar.bz2
cd libvmime-0.9.1
export CPPFLAGS=$CPPFLAGS:'-I$MY_WORKSPACE_INSTALL_ISOLATED/include'
export LDFLAGS=$LDFLAGS:'-I$MY_WORKSPACE_INSTALL_ISOLATED/lib'
./configure --prefix=$MY_WORKSPACE_INSTALL_ISOLATED
make install

