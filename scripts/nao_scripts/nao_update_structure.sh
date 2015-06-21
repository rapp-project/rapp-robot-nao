#!/bin/bash

# written by Maksym Figat

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"


if [ "$#" -ne 1 ]; then
	echo -e "$COL_RED[Error]$COL_RESET - Usage: $COL_GREEN$0 <VIRTUAL MACHINE IP ADDRESS>$COL_RESET"
	echo "Script that updates file structure from virtual machine using rsync mechanism"
	exit 1
fi

echo -e "$COL_GREEN[Ok]$COL_RESET - Synchronization of ws_ros"
cd /home/nao/ws_ros
rsync -avz nao@$1:/home/nao/ws_ros/ .

echo -e "$COL_GREEN[Ok]$COL_RESET - Synchronization of ws_ros_additional_packages"
cd /home/nao/ws_ros_additional_packages
rsync -avz nao@$1:/home/nao/ws_ros_additional_packages/ .

echo -e "$COL_GREEN[Ok]$COL_RESET - Synchronization of ws_rapp_nao"
cd /home/nao/ws_rapp_nao
rsync -avz nao@$1:/home/nao/ws_rapp_nao/ .

echo -e "$COL_GREEN[Ok]$COL_RESET - Synchronization of nao_scripts"
cd /home/nao/scripts/nao_scripts
rsync -avz nao@$1:/home/nao/ws_rapp_nao/src/rapp-robot-nao/scripts/nao_scripts/ .


