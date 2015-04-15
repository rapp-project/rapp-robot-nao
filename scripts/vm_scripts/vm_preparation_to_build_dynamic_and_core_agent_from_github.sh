#!/bin/bash

# written by Maksym Figat
GIT_DIRECTORY=/home/nao/download/git

if [ -d $GIT_DIRECTORY ]; then #If direcotory exists
  echo "Removing - a directory. Cleaning git folder."
  rm $GIT_DIRECTORY -rf
fi

mkdir -p ~/download/git
cd ~/download/git
git clone https://github.com/rapp-project/rapp-robot-nao.git
cp -r ~/download/git/rapp-robot-nao/scripts/vm_scripts/ ~/scripts
cp -r ~/download/git/rapp-robot-nao/rapp_robot_agent ~/ws_rapp/src/
cp -r ~/download/git/rapp-robot-nao/rapp_libraries ~/ws_rapp/src/ #change to RAPP API
cp -r ~/download/git/rapp-robot-nao/rapp_dynamic ~/ws_rapp/src/
