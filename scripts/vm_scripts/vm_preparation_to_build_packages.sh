#!/bin/bash

# written by Maksym Figat
GIT_WS_RAPP_NAO_DIR="/home/nao/ws_rapp_nao/src"
GIT_WS_RAPP_APPLICATIONS_DIR="/home/nao/ws_rapp_applications"

WS_RAPP_APPLICATIONS_NAO_DIR="/home/nao/ws_rapp_applications_nao"
WS_RAPP_NAO_DIR="/home/nao/ws_rapp_nao"

HZ_DIRECTORY="/home/nao/ws_rapp_applications/rapp-applications/nao/hz_packages"

ROS_ADDITIONAL_PACKAGES_DIR="/home/nao/ws_ros_additional_packages"
ROS_ADDITIONAL_PACKAGES_SRC_DIR=$ROS_ADDITIONAL_PACKAGES_DIR"/src"

VM_SCRIPTS="/home/nao/scripts"

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <flag>"
	echo "flag = 0 - create structure of folders and copy files from downloaded repositories"
	echo "flag = 1 - download <rapp-robot-nao> and <rapp_application> repositories, create structure of folders and copy files from downloaded repositories"
	exit 1
fi

if [ -d $GIT_WS_RAPP_NAO_DIR ]; then #If directory exists
	if [ $1 -eq 1 ]; then #clone from github
		echo "Removing $GIT_WS_RAPP_NAO_DIR - a directory. Cleaning git folder."
		rm $GIT_WS_RAPP_NAO_DIR -rf
		mkdir -p $GIT_WS_RAPP_NAO_DIR
	fi
else
	mkdir -p $GIT_WS_RAPP_NAO_DIR
fi

if [ -d $GIT_WS_RAPP_APPLICATIONS_DIR ]; then #If directory exists
	if [ $1 -eq 1 ]; then #clone from github
		echo "Removing $GIT_WS_RAPP_APPLICATION_DIR - a directory. Cleaning git folder."
		rm $GIT_WS_RAPP_APPLICATIONS_DIR -rf
		mkdir -p $GIT_WS_RAPP_APPLICATIONS_DIR
	fi
else
	mkdir -p $GIT_WS_RAPP_APPLICATIONS_DIR
fi

if [ -d $WS_RAPP_APPLICATIONS_NAO_DIR ]; then #If directory exists
	echo "Directory $WS_RAPP_APPLICATIONS_NAO_DIR exists"
	if [ $1 -eq 1 ]; then #clone from github
		echo "Removing $WS_RAPP_APPLICATIONS_NAO_DIR - a directory. Cleaning git folder."
		rm $WS_RAPP_APPLICATIONS_NAO_DIR -rf
		mkdir -p $WS_RAPP_APPLICATIONS_NAO_DIR
	fi
else
	echo "Creating - a directory. Creating $WS_RAPP_APPLICATIONS_NAO_DIR directory."
	mkdir -p $WS_RAPP_APPLICATIONS_NAO_DIR
fi

if [ $1 -eq 1 ]; then #clone from github
	cd $GIT_WS_RAPP_NAO_DIR
	echo "Clonning rapp-robot-nao repository to $GIT_WS_RAPP_NAO_DIR"
	echo "Enter your github login and password"
	git clone -b master https://github.com/rapp-project/rapp-robot-nao.git
	
	cd $GIT_WS_RAPP_APPLICATIONS_DIR
	echo "Enter your github login and password"
	echo "Clonning rapp-robot-nao repository to $GIT_WS_RAPP_APPLICATIONS_DIR"
	git clone -b master https://github.com/rapp-project/rapp-applications.git
fi

echo "Copying dynamic agent packages to $WS_RAPP_APPLICATIONS_NAO_DIR directory"
cp -r $GIT_WS_RAPP_APPLICATIONS_DIR/rapp-applications/nao/src $WS_RAPP_APPLICATIONS_NAO_DIR

echo "Creating folder $HZ_DIRECTORY"
mkdir -p $HZ_DIRECTORY/packages

if [ -d $ROS_ADDITIONAL_PACKAGES_SRC_DIR ]; then #If directory exists
	echo "Workspace $ROS_ADDITIONAL_PACKAGES_SRC_DIR exists"
else
	echo "Creates $ROS_ADDITIONAL_PACKAGES_SRC_DIR"
	mkdir -p $ROS_ADDITIONAL_PACKAGES_SRC_DIR
fi

echo "Updates virtual machine scripts"
cd $VM_SCRIPTS
rm vm*
cp $GIT_WS_RAPP_NAO_DIR/rapp-robot-nao/scripts/vm_scripts/vm* .
