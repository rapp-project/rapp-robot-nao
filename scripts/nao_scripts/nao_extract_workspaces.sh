#!/bin/bash

# written by Maksym Figat

echo "Extracting workspaces"

WS_ROS="/home/nao/ws_ros"
WS_ROS_ADDITIONAL_PACKAGES="/home/nao/ws_ros_additional_packages"
WS_RAPP_NAO="/home/nao/ws_rapp_nao"
WS_RAPP_APPLICATIONS_NAO="/home/nao/ws_rapp_applications_nao/install_isolated"

echo "Creates workspaces"
mkdir -p $WS_ROS
mkdir -p $WS_ROS_ADDITIONAL_PACKAGES
mkdir -p $WS_RAPP_NAO
mkdir -p $WS_RAPP_APPLICATIONS_NAO

echo "Extracts archived files into correct workspaces"
echo "$WS_ROS"
cd $WS_ROS
tar zxvf /home/nao/ws_ros.tar.gz

echo "$WS_ROS_ADDITIONAL_PACKAGES"
cd $WS_ROS_ADDITIONAL_PACKAGES
tar zxvf /home/nao/ws_ros_additional_packages.tar.gz

echo "$WS_RAPP_NAO"
cd $WS_RAPP_NAO
tar zxvf /home/nao/ws_rapp_nao.tar.gz

echo "Removes archived files"
rm /home/nao/ws_ros.tar.gz /home/nao/ws_ros_additional_packages.tar.gz /home/nao/ws_rapp_nao.tar.gz

echo "Removes nao_scripts.tar.gz"
rm /home/nao/nao_scripts.tar.gz
