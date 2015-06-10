#!/bin/bash

# written by Maksym Figat

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"

GIT_WS_RAPP_NAO_DIR="/home/nao/ws_rapp_nao/src"
GIT_WS_RAPP_APPLICATIONS_DIR="/home/nao/ws_rapp_applications"

WS_RAPP_APPLICATIONS_NAO_DIR="/home/nao/ws_rapp_applications_nao"
WS_RAPP_NAO_DIR="/home/nao/ws_rapp_nao"

HZ_DIRECTORY="/home/nao/ws_rapp_applications/rapp-applications/nao/hz_packages"

ROS_ADDITIONAL_PACKAGES_DIR="/home/nao/ws_ros_additional_packages"
ROS_ADDITIONAL_PACKAGES_SRC_DIR=$ROS_ADDITIONAL_PACKAGES_DIR"/src"

VM_SCRIPTS="/home/nao/scripts"

if [ "$#" -ne 1 ]; then
	echo -e "$COL_RED[Error]$COL_RESET - Usage: $0 <flag>"
	echo "flag = 0 - create structure of folders and copy files from downloaded repositories"
	echo "flag = 1 - download <rapp-robot-nao> and <rapp_application> repositories, create structure of folders and copy files from downloaded repositories"
	exit 1
fi

if [ -d $GIT_WS_RAPP_NAO_DIR ]; then #If directory exists
	if [ $1 -eq 1 ]; then #clone from github
		echo "$COL_GREEN[OK]$COL_RESET - Removing $GIT_WS_RAPP_NAO_DIR - a directory. Cleaning git folder."
		rm $GIT_WS_RAPP_NAO_DIR -rf
		mkdir -p $GIT_WS_RAPP_NAO_DIR
	fi
else
	mkdir -p $GIT_WS_RAPP_NAO_DIR
fi

if [ -d $GIT_WS_RAPP_APPLICATIONS_DIR ]; then #If directory exists
	if [ $1 -eq 1 ]; then #clone from github
		echo -e "$COL_GREEN[OK]$COL_RESET - Removing $GIT_WS_RAPP_APPLICATION_DIR - a directory. Cleaning git folder."
		rm $GIT_WS_RAPP_APPLICATIONS_DIR -rf
		mkdir -p $GIT_WS_RAPP_APPLICATIONS_DIR
	fi
else
	mkdir -p $GIT_WS_RAPP_APPLICATIONS_DIR
fi

if [ -d $WS_RAPP_APPLICATIONS_NAO_DIR ]; then #If directory exists
	echo -e "$COL_GREEN[OK]$COL_RESET - Directory $WS_RAPP_APPLICATIONS_NAO_DIR exists"
	if [ $1 -eq 1 ]; then #clone from github
		echo -e "$COL_GREEN[OK]$COL_RESET - Removing $WS_RAPP_APPLICATIONS_NAO_DIR - a directory. Cleaning git folder."
		rm $WS_RAPP_APPLICATIONS_NAO_DIR -rf
		mkdir -p $WS_RAPP_APPLICATIONS_NAO_DIR
	fi
else
	echo -e "$COL_GREEN[OK]$COL_RESET - Creating - a directory. Creating $WS_RAPP_APPLICATIONS_NAO_DIR directory."
	mkdir -p $WS_RAPP_APPLICATIONS_NAO_DIR
fi

if [ $1 -eq 1 ]; then #clone from github
	cd $GIT_WS_RAPP_NAO_DIR
	echo -e "$COL_GREEN[OK]$COL_RESET - Clonning rapp-robot-nao repository to $GIT_WS_RAPP_NAO_DIR"
	echo "Enter your github login and password"
	git clone -b master https://github.com/rapp-project/rapp-robot-nao.git
	
	cd $GIT_WS_RAPP_APPLICATIONS_DIR
	echo -e "$COL_GREEN[OK]$COL_RESET - Clonning rapp-robot-nao repository to $GIT_WS_RAPP_APPLICATIONS_DIR"
	echo "Enter your github login and password"
	git clone -b master https://github.com/rapp-project/rapp-applications.git
fi

echo -e "$COL_GREEN[OK]$COL_RESET - Copying dynamic agent packages to $WS_RAPP_APPLICATIONS_NAO_DIR directory"
cp -r $GIT_WS_RAPP_APPLICATIONS_DIR/rapp-applications/nao/src $WS_RAPP_APPLICATIONS_NAO_DIR

echo -e "$COL_GREEN[OK]$COL_RESET - Creating folder $HZ_DIRECTORY"
mkdir -p $HZ_DIRECTORY/packages

if [ -d $ROS_ADDITIONAL_PACKAGES_SRC_DIR ]; then #If directory exists
	echo "Workspace $ROS_ADDITIONAL_PACKAGES_SRC_DIR exists"
else
	echo -e "$COL_GREEN[OK]$COL_RESET - Creates $ROS_ADDITIONAL_PACKAGES_SRC_DIR"
	mkdir -p $ROS_ADDITIONAL_PACKAGES_SRC_DIR
fi

echo -e "$COL_GREEN[OK]$COL_RESET - Updates virtual machine scripts in $VM_SCRIPTS folder"
cd $VM_SCRIPTS
rm vm*
cp $GIT_WS_RAPP_NAO_DIR/rapp-robot-nao/scripts/vm_scripts/vm* .
