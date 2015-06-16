#!/bin/bash

# written by Maksym Figat

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"

PROGRAMS_DIRECTORY="/home/nao/programs"

# ROS - core ROS packages
ROS_DIR="/home/nao/ws_ros"
ROS_SRC_DIR=$ROS_DIR"/src"
ROS_INSTALL_ISOLATED=$ROS_DIR"/install_isolated"

# ROS - additional ROS packages and libraries
ROS_ADDITIONAL_PACKAGES_DIR="/home/nao/ws_ros_additional_packages"
ROS_ADDITIONAL_PACKAGES_SRC_DIR=$ROS_ADDITIONAL_PACKAGES_DIR"/src"
ROS_ADDITIONAL_PACKAGES_ISOLATED=$ROS_ADDITIONAL_PACKAGES_DIR"/install_isolated"

SCRIPTS_DIR="/home/nao/ws_rapp_nao"

# Creates $ROS_ADDITIONAL_PACKAGES_SRC_DIR
if [ -d $ROS_ADDITIONAL_PACKAGES_SRC_DIR ]; then #If directory exists
	echo "Workspace $ROS_ADDITIONAL_PACKAGES_SRC_DIR exists"
else
	echo -e "$COL_GREEN[OK]$COL_RESET - Creates $ROS_ADDITIONAL_PACKAGES_SRC_DIR"
	mkdir -p $ROS_ADDITIONAL_PACKAGES_SRC_DIR
fi

# Removes $ROS_ADDITIONAL_PACKAGES_ISOLATED
if [ -d $ROS_ADDITIONAL_PACKAGES_ISOLATED ]; then #If directory exists
	echo -e "$COL_GREEN[OK]$COL_RESET - Removing $ROS_ADDITIONAL_PACKAGES_ISOLATED"
	cd $ROS_ADDITIONAL_PACKAGES_DIR
	rm install_isolated devel_isolated build_isolated -rf
fi

# Builds ROS core packages 
if [ -d $ROS_INSTALL_ISOLATED ]; then #If directory ROS_INSTALL_ISOLATED exists
	echo -e "$COL_GREEN[OK]$COL_RESET - Workspace $ROS_INSTALL_ISOLATED already exists"
	echo "No need for compilation of core ROS packages"
else
	cd $ROS_DIR
	echo -e "$COL_GREEN[OK]$COL_RESET - Compiles workspace: $ROS_DIR"
	src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
	
	echo -e "$COL_GREEN[OK]$COL_RESET - Copies ROS core dependencies"
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
fi

source $ROS_INSTALL_ISOLATED/setup.bash
echo -e "$COL_GREEN[OK]$COL_RESET - Building ros additional packages"

if [ -d $PROGRAMS_DIRECTORY ]; then #If directory exists
	echo -e "$COL_GREEN[OK]$COL_RESET - Folder $PROGRAMS_DIRECTORY exists"
	rm $PROGRAMS_DIRECTORY -rf
fi
mkdir -p $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Creating $PROGRAMS_DIRECTORY folder"

# Yaml-cpp
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of yaml-cpp"
wget https://launchpad.net/ubuntu/+archive/primary/+files/yaml-cpp_0.5.1.orig.tar.gz
tar zxvf yaml-cpp_0.5.1.orig.tar.gz
cd yaml-cpp-0.5.1
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make install

cd $ROS_ADDITIONAL_PACKAGES_SRC_DIR
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from bond_core repository"
git clone https://github.com/ros/bond_core.git
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from cmake_modules repository"
git clone -b 0.3-devel https://github.com/ros/cmake_modules.git
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from image_common repository"
git clone https://github.com/ros-perception/image_common.git
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from nodelet_core repository"
git clone -b indigo-devel https://github.com/ros/nodelet_core.git
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from vision_opencv repository"
git clone -b indigo https://github.com/ros-perception/vision_opencv.git
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from rosbridge_suite repository"
git clone https://github.com/RobotWebTools/rosbridge_suite.git
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from rosauth repository"
git clone https://github.com/WPI-RAIL/rosauth.git
cd ..

echo -e "$COL_GREEN[OK]$COL_RESET - Compiles workspace: $ROS_ADDITIONAL_PACKAGES_DIR"
catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

# Gsasl
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of Gsasl"
wget ftp://ftp.gnu.org/gnu/gsasl/libgsasl-1.8.0.tar.gz
tar zxvf libgsasl-1.8.0.tar.gz
cd libgsasl-1.8.0
./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make
make install
make clean

# Vmime
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of libvmime library"
wget http://sourceforge.net/projects/vmime/files/vmime/0.9/libvmime-0.9.1.tar.bz2
tar -xjf libvmime-0.9.1.tar.bz2
cd libvmime-0.9.1
./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED CPPFLAGS='-I/home/nao/ws_ros_additional_packages/install_isolated/include' LDFLAGS='-L/home/nao/ws_ros_additional_packages/install_isolated/lib'
make
make install
make clean

# Openssl
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Openssl"
wget https://www.openssl.org/source/openssl-1.0.2c.tar.gz
tar zxvf openssl-1.0.2c.tar.gz
cd openssl-1.0.2c
./config --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED CPPFLAGS='-I/home/nao/ws_ros_additional_packages/install_isolated/include' LDFLAGS='-L/home/nao/ws_ros_additional_packages/install_isolated/lib'
make
sudo make install
make clean

# Exporting LIBRARY PATH
echo -e "$COL_GREEN[OK]$COL_RESET - Exporting LIBRARY PATH"
export LIBRARY_PATH=/usr/local/lib

# Bigloo
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Bigloo"
wget ftp://ftp-sop.inria.fr/indes/rapp/hop/2015-05-07/bigloo4.2a.tar.gz
tar zxvf bigloo4.2a.tar.gz
cd bigloo4.2a
./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make
sudo make install
make clean

# Hop
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Hop"
wget ftp://ftp-sop.inria.fr/indes/rapp/hop/2015-05-07/hop-3.0.0-pre14.tar.gz
tar zxvf hop-3.0.0-pre14.tar.gz
cd hop-3.0.0-pre14
./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make
sudo make install
make clean
