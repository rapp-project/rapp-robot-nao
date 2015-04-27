#!/bin/bash

# written by Maksym Figat

PROGRAMS_DIRECTORY="/home/nao/programs"

# ROS - core ROS packages
ROS_DIR="/home/nao/ws_ros"
ROS_SRC_DIR=$ROS_DIR"/src"
ROS_INSTALL_ISOLATED=$ROS_DIR"/install_isolated"

# ROS - additional ROS packages and libraries
ROS_ADDITIONAL_PACKAGES_DIR="/home/nao/ws_ros_additional_packages"
ROS_ADDITIONAL_PACKAGES_SRC_DIR=$ROS_ADDITIONAL_PACKAGES_DIR"/src"
ROS_ADDITIONAL_PACKAGES_ISOLATED=$ROS_ADDITIONAL_PACKAGES_DIR"/install_isolated"

# Rapp - ws_rapp_nao - core agent packages
WS_RAPP_NAO_DIR="/home/nao/ws_rapp_nao"
WS_RAPP_NAO_ISOLATED_DIR=$WS_RAPP_NAO_DIR"/install_isolated"

# Rapp - ws_rapp_applications_nao - dynamic agent packages
WS_RAPP_APPLICATIONS_NAO_DIR="/home/nao/ws_rapp_applications_nao"

cd $WS_RAPP_NAO_DIR
echo "Sources with $ROS_ADDITIONAL_PACKAGES_ISOLATED"
source $ROS_ADDITIONAL_PACKAGES_ISOLATED/setup.bash
echo "Compiles workspace: $WS_RAPP_NAO_DIR"
$ROS_SRC_DIR/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

cd $WS_RAPP_APPLICATIONS_NAO_DIR
echo "Sources with $WS_RAPP_NAO_DIR"
source $WS_RAPP_NAO_DIR/setup.bash
echo "Compiles workspace: $WS_RAPP_APPLICATIONS_NAO_DIR"
$ROS_SRC_DIR/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
