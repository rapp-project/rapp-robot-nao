#!/bin/bash

# written by Maksym Figat from ROS tutorial
# http://wiki.ros.org/nao/Tutorials/Installation/compileFromVirtualNao

ROS_WORKSPACE="/home/nao/ws_ros"

mkdir -p $ROS_WORKSPACE/src
mkdir -p $ROS_WORKSPACE/external
cd $ROS_WORKSPACE

sudo bash -c 'echo app-portage/gentoolkit >> /etc/portage/package.keywords'
sudo bash -c 'echo dev-python/setuptools >> /etc/portage/package.keywords'
sudo emerge setuptools
sudo emerge --autounmask-write tinyxml cxsparse cholmod poco log4cxx gtest yaml-cpp
sudo etc-update
sudo emerge tinyxml cxsparse cholmod poco log4cxx gtest yaml-cpp

sudo easy_install rosdep rosinstall_generator wstool rosinstall rospkg empy nose catkin_pkg pyyaml netifaces
sudo rosdep init
rosdep update

touch ~/temp && echo '#define TIXML_USE_STL' | cat - /usr/include/tinyxml.h > ~/temp && sudo mv ~/temp /usr/include/tinyxml.h

cd external
hg clone http://www.riverbankcomputing.com/hg/sip
cd sip
python build.py prepare
python configure.py
make
sudo make install
cd ..

svn checkout http://lz4.googlecode.com/svn/trunk/ lz4
cd lz4/
make
sudo make install
cd ..

git clone --depth 5 git://github.com/ros/console_bridge.git
cd console_bridge
cmake .
make
sudo make install
cd ..

git clone --depth 5 https://github.com/ros/urdfdom_headers.git urdfdom-headers
cd urdfdom-headers
cmake . -DCMAKE_INSTALL_PREFIX=/usr
make
sudo make install
cd ..

git clone --depth 5 https://github.com/ros/urdfdom.git urdfdom
cd urdfdom

sed -i "s/--install-layout deb//g" urdf_parser_py/CMakeLists.txt
cmake . -DCMAKE_INSTALL_PREFIX=/usr
make
sudo make install
cd ../..

rosinstall_generator roscpp roscpp_core std_msgs rospy sensor_msgs nao_meshes nao_robot --rosdistro indigo --deps > nao_ros_indigo.rosinstall
wstool init src nao_ros_indigo.rosinstall -j8
rosdep install --from-path src -i -y
cd src
rm -rf diagnostics/self_test diagnostics/test_diagnostic_aggregator
cd ..

rosdep install --from-path src -i -y

bash /home/nao/ws_ros/scripts/vm_ros_install_isolated.sh
