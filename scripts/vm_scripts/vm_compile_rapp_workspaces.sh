#!/bin/bash

# written by Maksym Figat

PROGRAMS_DIRECTORY="/home/nao/programs"

# ROS - core ROS packages
WS_ROS_DIR="/home/nao/ws_ros"
WS_ROS_SRC_DIR=$WS_ROS_DIR"/src"
WS_ROS_INSTALL_ISOLATED=$WS_ROS_DIR"/install_isolated"

# ROS - additional ROS packages and libraries
WS_ROS_ADDITIONAL_PACKAGES_DIR="/home/nao/ws_ros_additional_packages"
WS_ROS_ADDITIONAL_PACKAGES_SRC_DIR=$WS_ROS_ADDITIONAL_PACKAGES_DIR"/src"
WS_ROS_ADDITIONAL_PACKAGES_ISOLATED=$WS_ROS_ADDITIONAL_PACKAGES_DIR"/install_isolated"

# Rapp - ws_rapp_nao - core agent packages
WS_RAPP_NAO_DIR="/home/nao/ws_rapp_nao"
WS_RAPP_NAO_ISOLATED=$WS_RAPP_NAO_DIR"/install_isolated"

# Rapp - ws_rapp_applications_nao - dynamic agent packages
WS_RAPP_APPLICATIONS_NAO_DIR="/home/nao/ws_rapp_applications_nao"


cd $WS_RAPP_NAO_DIR/src
$WS_ROS_SRC_DIR/catkin/bin/catkin_init_workspace
cd $WS_RAPP_NAO_DIR

echo "Sources with $WS_ROS_ADDITIONAL_PACKAGES_ISOLATED"
source $WS_ROS_ADDITIONAL_PACKAGES_ISOLATED/setup.bash
echo "Compiles workspace: $WS_RAPP_NAO_DIR"
catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release


cd $WS_RAPP_APPLICATIONS_NAO_DIR/src
$WS_ROS_SRC_DIR/catkin/bin/catkin_init_workspace
cd $WS_RAPP_APPLICATIONS_NAO_DIR

echo "Sources with $WS_RAPP_NAO_ISOLATED"
source $WS_RAPP_NAO_ISOLATED/setup.bash
echo "Compiles workspace: $WS_RAPP_APPLICATIONS_NAO_DIR"
catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
