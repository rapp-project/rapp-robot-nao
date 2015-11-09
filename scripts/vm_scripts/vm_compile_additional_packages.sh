#!/bin/bash

# written by Maksym Figat & Wojciech Dudek

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"

PROGRAMS_DIRECTORY="/home/nao/ws_ros_additional_packages/programs"

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
	sudo rm install_isolated devel_isolated build_isolated -rf
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
# Eigen
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of Eigen"
wget http://bitbucket.org/eigen/eigen/get/3.2.5.tar.gz
tar zxvf 3.2.5.tar.gz
mkdir eigen-eigen-bdd17ee3b1b3/build_dir
cd eigen-eigen-bdd17ee3b1b3/build_dir
cmake .. -DCMAKE_INSTALL_PREFIX=$ROS_ADDITIONAL_PACKAGES_ISOLATED -DCMAKE_BUILD_TYPE=Release
make install
make clean


# # FLANN
# cd $PROGRAMS_DIRECTORY
# echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of PCL"
# wget http://www.cs.ubc.ca/research/flann/uploads/FLANN/flann-1.8.4-src.zip
# unzip flann-1.8.4-src.zip
# cd flann-1.8.4-src && mkdir build && cd build
# cmake -DCMAKE_INSTALL_PREFIX=$ROS_ADDITIONAL_PACKAGES_ISOLATED -DCMAKE_BUILD_TYPE=Release .. 
# make 
# make install
# make clean

# #QHULL
# cd $PROGRAMS_DIRECTORY
# echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of QHULL"
# wget http://www.qhull.org/download/qhull-2012.1-src.tgz
# tar zxvf qhull-2012.1-src.tgz
# cd qhull-2012.1
# mkdir build
# cd build
# cmake -DCMAKE_INSTALL_PREFIX=$ROS_ADDITIONAL_PACKAGES_ISOLATED -DCMAKE_BUILD_TYPE=Release .. 
# make 
# make install
# make clean

# # PCL
# cd $PROGRAMS_DIRECTORY
# echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of PCL"
# git clone https://github.com/PointCloudLibrary/pcl.git
# cd pcl && mkdir build && cd build
# cmake -DCMAKE_INSTALL_PREFIX=$ROS_ADDITIONAL_PACKAGES_ISOLATED -DCMAKE_BUILD_TYPE=Release -DWITH_CUDA:BOOL=OFF -DWITH_DAVIDSDK:BOOL=ON -DWITH_DOCS:BOOL=OFF -DWITH_ENSENSO:BOOL=ON -DWITH_FZAPI:BOOL=OFF -DWITH_LIBUSB:BOOL=OFF -DWITH_OPENGL:BOOL=OFF -DWITH_OPENNI:BOOL=OFF -DWITH_OPENNI2:BOOL=OFF -DWITH_PCAP:BOOL=OFF -DWITH_PNG:BOOL=ON -DWITH_PXCAPI:BOOL=OFF -DWITH_QHULL:BOOL=ON -DWITH_QT:BOOL=OFF -DWITH_VTK:BOOL=OFF -DBUILD_recognition:BOOL=OFF ..
# make 
# make install
# make clean

# SDL2
#cd $PROGRAMS_DIRECTORY
#echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of PCL"
#wget https://www.libsdl.org/release/SDL2-2.0.3.tar.gz
#tar zxvf SDL2-2.0.3.tar.gz
##cd SDL2-2.0.3 && ./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED && mkdir build && cd build
#cmake -DCMAKE_INSTALL_PREFIX=$ROS_ADDITIONAL_PACKAGES_ISOLATED -DCMAKE_BUILD_TYPE=Release .. 
#make
#sudo make install
#make clean

# SDL_image 2.0
#cd $PROGRAMS_DIRECTORY
#echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of PCL"
#wget https://www.libsdl.org/projects/SDL_image/release/SDL2_image-2.0.0.tar.gz
#tar zxvf SDL2_image-2.0.0.tar.gz

#cd SDL2_image-2.0.0 && sh ./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED CPPFLAGS="-I/home/nao/ws_ros_additional_packages/install_isolated/include" LDFLAGS="-L/home/nao/ws_ros_additional_packages/install_isolated/lib"
#make
#make install
#make clean
# # SDL-1.2
# cd $PROGRAMS_DIRECTORY
# echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of SDL-1.2.15"
# wget https://www.libsdl.org/release/SDL-1.2.15.tar.gz
# tar zxvf SDL-1.2.15.tar.gz
# cd SDL-1.2.15 && ./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
# make 
# make install

# # SDL_image-1.2
# cd $PROGRAMS_DIRECTORY
# echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of SDL_image-1.2.12"
# wget https://www.libsdl.org/projects/SDL_image/release/SDL_image-1.2.12.tar.gz
# tar zxvf SDL_image-1.2.12.tar.gz
# cd SDL_image-1.2.12 && ./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED 
# #cmake -DCMAKE_INSTALL_PREFIX=$ROS_ADDITIONAL_PACKAGES_ISOLATED -DCMAKE_BUILD_TYPE=Release .. 
# make
# make install

# link libraries to system path
# sudo ln -s /home/nao/ws_ros_additional_packages/install_isolated/lib/libyaml-cpp.a /usr/lib/libyaml-cpp.a
# sudo ln -s /home/nao/ws_ros_additional_packages/install_isolated/lib/libSDL_image.so /usr/lib/libSDL_image.so
# sudo ln -s /home/nao/ws_ros_additional_packages/install_isolated/lib/libSDL.so /usr/lib/libSDL.so


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

echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from uuid_msgs repository for robot_localization pkg"
git clone https://github.com/ros-geographic-info/unique_identifier.git

# for robot_localization
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from rosbag repository for robot_localization pkg"
git clone https://github.com/ros/ros_comm.git
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from geographic_msgs repository for robot_localization pkg"
git clone https://github.com/ros-geographic-info/geographic_info.git 
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from tf2 repository for robot_localization pkg"
git clone https://github.com/ros/geometry_experimental.git
cd geometry_experimental
rm -R tf2_bullet
cd ..
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from diagnostics repository for robot_localization pkg"
git clone  https://github.com/ros/diagnostics.git 
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code from robot_localization repository"
git clone https://github.com/cra-ros-pkg/robot_localization.git
cd ..
# compilation
echo -e "$COL_GREEN[OK]$COL_RESET - Compiles workspace: $ROS_ADDITIONAL_PACKAGES_DIR"
catkin_make_isolated --install --pkg bond cv_bridge -DCMAKE_BUILD_TYPE=Release -DCMAKE_MODULE_PATH=/home/nao/ws_ros_additional_packages/programs/eigen-eigen-bdd17ee3b1b3/cmake -DCMAKE_CC_COMPILER=/usr/bin/gcc -j1 -l1
catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DCMAKE_MODULE_PATH=/home/nao/ws_ros_additional_packages/programs/eigen-eigen-bdd17ee3b1b3/cmake -DCMAKE_CC_COMPILER=/usr/bin/gcc


# Gsasl
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of Gsasl"
wget ftp://ftp.gnu.org/gnu/gsasl/libgsasl-1.8.0.tar.gz
tar zxvf libgsasl-1.8.0.tar.gz
cd libgsasl-1.8.0
./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make || { echo -e >&2 "$COL_RED[Error]$COL_RESET - gsasl make failed with $?"; exit 1; }
make install
make clean

# Vmime
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of libvmime library"
wget http://sourceforge.net/projects/vmime/files/vmime/0.9/libvmime-0.9.1.tar.bz2
tar -xjf libvmime-0.9.1.tar.bz2
cd libvmime-0.9.1
./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED CPPFLAGS='-I/home/nao/ws_ros_additional_packages/install_isolated/include' LDFLAGS='-L/home/nao/ws_ros_additional_packages/install_isolated/lib'
make || { echo -e >&2 "$COL_RED[Error]$COL_RESET - vmime make failed with $?"; exit 1; }
make install
make clean

# Sourcing ws_ros_additional_packages workspace
source $ROS_ADDITIONAL_PACKAGES_ISOLATED/setup.bash
echo -e "$COL_GREEN[OK]$COL_RESET - Sourcing ws_ros_additional_packages workspace"

# Openssl
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Openssl"
wget ftp://www.openssl.org/source/openssl-1.0.2d.tar.gz
tar zxvf openssl-1.0.2d.tar.gz
cd openssl-1.0.2d
./config --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED --openssldir=$ROS_ADDITIONAL_PACKAGES_ISOLATED/#openssl
make || { echo -e >&2 "$COL_RED[Error]$COL_RESET - openssl make failed with $?"; exit 1; }
sudo make install
make clean

# unistring
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading and building source code of unistring library"
wget http://ftp.gnu.org/gnu/libunistring/libunistring-latest.tar.gz
tar -zxvf libunistring-latest.tar.gz
cd libunistring-0.9.6/
./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make || { echo -e >&2 "$COL_RED[Error]$COL_RESET - unistring make failed with $?"; exit 1; }
sudo make install
make clean

# Bigloo
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Bigloo"
wget ftp://ftp-sop.inria.fr/indes/fp/Bigloo/bigloo4.2c-beta04Nov15.tar.gz
tar zxvf bigloo4.2c-beta04Nov15.tar.gz
cd bigloo4.2c
./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make || { echo -e >&2 "$COL_RED[Error]$COL_RESET - bigloo make failed with $?"; exit 1; }
sudo make install
make clean

# Exporting LIBRARY PATH
echo -e "$COL_GREEN[OK]$COL_RESET - Exporting LIBRARY PATH"
export LIBRARY_PATH=$ROS_ADDITIONAL_PACKAGES_ISOLATED/lib

# Hop
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Hop"
git clone https://github.com/manuel-serrano/hop.git
cd hop
./configure --prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED --bigloo=$ROS_ADDITIONAL_PACKAGES_ISOLATED/bin/bigloo --bigloo-unistring=no
make || { echo -e >&2 "$COL_RED[Error]$COL_RESET - hop make failed with $?"; exit 1; }
sudo make install
make clean


