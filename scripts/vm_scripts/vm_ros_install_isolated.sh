#!/bin/bash

# written by Maksym Figat

ROS_DIR="/home/nao/my_workspace"
MY_WORKSPACE_INSTALL_ISOLATED=$ROS_DIR"/install_isolated"

if [ -d $MY_WORKSPACE_INSTALL_ISOLATED ]; then #If directory exists
	echo "Removes folders $ROS_DIR/install_isolated $ROS_DIR/build_isolated $ROS_DIR/devel_isolated"
	rm devel_isolated build_isolated install_isolated -rf
fi

cd ROS_DIR
echo "Building ros packages"
$ROS_DIR/src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

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



