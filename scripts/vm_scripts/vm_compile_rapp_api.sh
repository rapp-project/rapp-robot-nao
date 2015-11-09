#!/bin/bash

# written by Wojciech Dudek

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"

# Rapp - ws_rapp_nao - core agent packages
WS_RAPP_API_DIR="/home/nao/ws_rapp_api"
cd $WS_RAPP_API_DIR
mkdir -p cpp/build/
WS_RAPP_API_DIR_BUILD="/home/nao/ws_rapp_api/cpp/build"
cd $WS_RAPP_API_DIR_BUILD

cmake ..

make -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/home/nao/ws_rapp_api/install
echo -e "$COL_GREEN[OK]$COL_RESET - Sources with $WS_ROS_ADDITIONAL_PACKAGES_ISOLATED"
source $WS_ROS_ADDITIONAL_PACKAGES_ISOLATED/setup.bash 
echo -e "$COL_GREEN[OK]$COL_RESET - Compiles workspace: $WS_RAPP_NAO_DIR"
#catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --pkg rapp_ros_naoqi_wrappings
catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release || { echo -e >&2 "$COL_RED[Error]$COL_RESET - catkin_make_isolated failed with $?"; exit 1; }

