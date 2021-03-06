#!/bin/bash

# written by Maksym Figat

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"


# ROS - core ROS packages
WS_ROS_DIR="/home/nao/ws_ros"
WS_ROS_SRC_DIR=$WS_ROS_DIR"/src"

# ROS - additional ROS packages and libraries
WS_ROS_ADDITIONAL_PACKAGES_DIR="/home/nao/ws_ros_additional_packages"
WS_ROS_ADDITIONAL_PACKAGES_ISOLATED=$WS_ROS_ADDITIONAL_PACKAGES_DIR"/install_isolated"

# Rapp - ws_rapp_nao - core agent packages
WS_RAPP_NAO_DIR="/home/nao/ws_rapp_nao"
WS_RAPP_NAO_ISOLATED=$WS_RAPP_NAO_DIR"/install_isolated"
WS_RAPP_API_DIR="/home/nao/ws_rapp_api"

source $WS_ROS_DIR/install_isolated/setup.bash
cd $WS_RAPP_NAO_DIR/src
if [ ! -f $WS_RAPP_NAO_DIR/src/CMakeLists.txt ]; then
	catkin_init_workspace
fi

cd $WS_RAPP_NAO_DIR
export Rapp_DIR=$WS_RAPP_API_DIR/install/share/Rapp
export RappRobots_DIR=$WS_RAPP_API_DIR/install/share/RappRobots
echo -e "$COL_GREEN[OK]$COL_RESET - Sources with $WS_ROS_ADDITIONAL_PACKAGES_ISOLATED"
source $WS_ROS_ADDITIONAL_PACKAGES_ISOLATED/setup.bash 
echo -e "$COL_GREEN[OK]$COL_RESET - Compiles workspace: $WS_RAPP_NAO_DIR"
#catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --pkg rapp_ros_naoqi_wrappings
catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DCMAKE_MODULE_PATH=/home/nao/ws_ros_additional_packages/programs/eigen-eigen-bdd17ee3b1b3/cmake|| { echo -e >&2 "$COL_RED[Error]$COL_RESET - catkin_make_isolated failed with $?"; exit 1; }
