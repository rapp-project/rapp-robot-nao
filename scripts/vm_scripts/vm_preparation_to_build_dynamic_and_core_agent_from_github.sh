#!/bin/bash

# written by Maksym Figat
GIT_DIRECTORY=/home/nao/download/git
WS_RAPP_DIRECTORY=/home/nao/ws_rapp/src
HZ_DIRECTORY=/home/nao/rapp/hz_packages

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <flag>"
	echo "flag = 0 - create structure of folders and copy files from downloaded repository"
	echo "flag = 1 - download rapp-robot-nao repository, create structure of folders and copy files to downloaded repository"
	exit 1
fi

if [ -d $GIT_DIRECTORY ]; then #If direcotory exists
	if [ $1 -eq 1 ]; then #clone from github
		echo "Removing - a directory. Cleaning git folder."
		rm $GIT_DIRECTORY -rf
	fi
fi

if [ -d $WS_RAPP_DIRECTORY ]; then #If direcotory exists
	echo "ws_rapp/src exists"
else
	echo "Creating - a directory. Creating ~/ws_rapp/src folder."
	mkdir -p $WS_RAPP_DIRECTORY
	cd $WS_RAPP_DIRECTORY
	~/my_workspace/src/catkin/bin/catkin_init_workspace
fi

if [ $1 -eq 1 ]; then #clone from github
	echo "Clonning from github"
	mkdir -p ~/download/git
	cd ~/download/git
	git clone https://github.com/rapp-project/rapp-robot-nao.git
fi

echo "Copying scripts to /home/nao/scripts/ directory"
cp -r ~/download/git/rapp-robot-nao/scripts/vm_scripts/* ~/scripts
echo "Copying source code to /home/nao/ws_rapp/src/ directory"
cp -r ~/download/git/rapp-robot-nao/ros_packages/* ~/ws_rapp/src

if [  -d $HZ_DIRECTORY ]; then #if directory exists
	echo "Directory $HZ_DIRECTORY exists"
	rm -rf $HZ_DIRECTORY
fi

echo "Creating folder $HZ_DIRECTORY"
mkdir -p $HZ_DIRECTORY/template_package
mkdir -p $HZ_DIRECTORY/hz


cp -r ~/download/git/rapp-robot-nao/hz_packages/template_package/* $HZ_DIRECTORY/template_package
