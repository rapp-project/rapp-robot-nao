#!/bin/bash

# written by Maksym Figat

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"


if [ "$#" -ne 1 ]; then
	echo -e "$COL_RED[Error]$COL_RESET - Usage: $COL_GREEN$0 <NAO IP ADDRESS>$COL_RESET"
	echo "Script that updates file structure on VM and then synchronizes files with Nao using rsync mechanism"
	exit 1
fi

bash vm_preparation_to_build_packages.sh 1
bash vm_compile_rapp_workspaces.sh


echo -e "$COL_GREEN[Ok]$COL_RESET - Synchronization of ws_ros on Nao"
rsync -az /home/nao/ws_ros/ nao@$1:/home/nao/ws_ros/

echo -e "$COL_GREEN[Ok]$COL_RESET - Synchronization of ws_ros_additional_packages on Nao"
rsync -az /home/nao/ws_ros_additional_packages/ nao@$1:/home/nao/ws_ros_additional_packages/

echo -e "$COL_GREEN[Ok]$COL_RESET - Synchronization of ws_rapp_nao on Nao"
cd /home/nao/ws_rapp_nao
rsync -az /home/nao/ws_rapp_nao/ nao@$1:/home/nao/ws_rapp_nao/

echo -e "$COL_GREEN[Ok]$COL_RESET - Synchronization of nao_scripts on Nao"
rsync -az /home/nao/ws_rapp_nao/src/rapp-robot-nao/scripts/nao_scripts/ nao@$1:/home/nao/scripts/nao_scripts/


