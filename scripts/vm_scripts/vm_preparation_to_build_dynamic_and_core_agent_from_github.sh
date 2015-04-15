#!/bin/bash

# written by Maksym Figat
GIT_DIRECTORY=/home/nao/download/git
WS_RAPP_DIRECTORY=/home/nao/ws_rapp/src

if [ -d $GIT_DIRECTORY ]; then #If direcotory exists
  echo "Removing - a directory. Cleaning git folder."
  rm $GIT_DIRECTORY -rf
fi

if [ -d $WS_RAPP_DIRECTORY ]; then #If direcotory exists
  echo "ws_rapp/src exists"
else
  echo "Creating - a directory. Creating ~/ws_rapp/src folder."
  mkdir -p $WS_RAPP_DIRECTORY
  cd $WS_RAPP_DIRECTORY
  ~/my_workspace/src/catkin/bin/catkin_init_workspace
fi

mkdir -p ~/download/git
cd ~/download/git
git clone https://github.com/rapp-project/rapp-robot-nao.git
cp -r ~/download/git/rapp-robot-nao/scripts/vm_scripts/* ~/scripts
cp -r ~/download/git/rapp-robot-nao/rapp_robot_agent ~/ws_rapp/src
cp -r ~/download/git/rapp-robot-nao/rapp_libraries ~/ws_rapp/src #change to RAPP API
cp -r ~/download/git/rapp-robot-nao/rapp_dynamic ~/ws_rapp/src
