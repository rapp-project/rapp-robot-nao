#!/bin/bash

# written by Maksym Figat

echo "Extracting workspaces..."

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"

WS_ROS="/home/nao/ws_ros"
WS_ROS_ADDITIONAL_PACKAGES="/home/nao/ws_ros_additional_packages"
WS_RAPP_NAO="/home/nao/ws_rapp_nao"
WS_RAPP_APPLICATIONS_NAO_INSTALL_ISOLATED="/home/nao/ws_rapp_applications_nao/install_isolated"
WS_RAPP_APPLICATIONS_NAO_DATA="/home/nao/ws_rapp_applications_nao/data"

echo -e "$COL_GREEN[OK]$COL_RESET - Creates workspaces"
mkdir -p $WS_ROS
mkdir -p $WS_ROS_ADDITIONAL_PACKAGES
mkdir -p $WS_RAPP_NAO
mkdir -p $WS_RAPP_APPLICATIONS_NAO_INSTALL_ISOLATED

echo -e "$COL_GREEN[OK]$COL_RESET - Extracts archived files into correct workspaces"
echo -e "$COL_GREEN[OK]$COL_RESET - $WS_ROS"
cd $WS_ROS
tar zxf /home/nao/ws_ros.tar.gz

echo -e "$COL_GREEN[OK]$COL_RESET - $WS_ROS_ADDITIONAL_PACKAGES"
cd $WS_ROS_ADDITIONAL_PACKAGES
tar zxf /home/nao/ws_ros_additional_packages.tar.gz

echo -e "$COL_GREEN[OK]$COL_RESET - $WS_RAPP_NAO"
cd $WS_RAPP_NAO
tar zxf /home/nao/ws_rapp_nao.tar.gz

echo -e "$COL_GREEN[OK]$COL_RESET - Extracts nao data"
cd $WS_RAPP_APPLICATIONS_NAO_DATA
tar zxf /home/nao/nao_data.tar.gz

echo -e "$COL_GREEN[OK]$COL_RESET - Removes archived files"
rm /home/nao/ws_ros.tar.gz /home/nao/ws_ros_additional_packages.tar.gz /home/nao/ws_rapp_nao.tar.gz

echo -e "$COL_GREEN[OK]$COL_RESET - Removes nao_scripts.tar.gz"
rm /home/nao/nao_scripts.tar.gz

echo -e "$COL_GREEN[OK]$COL_RESET - Removes nao_data.tar.gz"
rm /home/nao/nao_data.tar.gz

exit
