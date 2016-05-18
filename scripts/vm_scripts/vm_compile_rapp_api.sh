#!/bin/bash

# written by Wojciech Dudek

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"

# Rapp - ws_rapp_nao - core agent packages
WS_RAPP_API_DIR="/home/nao/ws_rapp_api"
cd $WS_RAPP_API_DIR
mkdir -p src/rapp-api/cpp/build/
WS_RAPP_API_DIR_BUILD="/home/nao/ws_rapp_api/src/rapp-api/cpp/build/"
cd $WS_RAPP_API_DIR_BUILD

cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/home/nao/ws_rapp_api/install

make 
make install
echo -e "$COL_GREEN[OK]$COL_RESET - Sources with $WS_ROS_ADDITIONAL_PACKAGES_ISOLATED"

# rapp-robots-api
WS_RAPP_ROBOT_API_DIR="/home/nao/ws_rapp_robots_api"
cd $WS_RAPP_ROBOT_API_DIR
mkdir -p src/rapp-robots-api/cpp/build/
WS_RAPP_ROBOTS_API_DIR_BUILD="/home/nao/ws_rapp_robots_api/src/rapp-robots-api/cpp/build/"
cd $WS_RAPP_ROBOT_API_DIR_BUILD

cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/home/nao/ws_rapp_robot_api/install
make 
make install
