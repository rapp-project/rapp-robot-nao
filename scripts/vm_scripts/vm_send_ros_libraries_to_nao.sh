#!/bin/bash

# written by Maksym Figat & Wojciech Dudek

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_BLUE=$ESC_SEQ"34;01m"
COL_RED=$ESC_SEQ"31;01m"

NAO_SCRIPTS="/home/nao/scripts"

if [ "$#" -ne 1 ]; then
	echo -e "$COL_RED[Error]$COL_RESET - Usage: Usage: $COL_GREEN$0 <NAO_IP_ADDRESS>$COL_RESET"
	exit 1
fi

echo -e "$COL_GREEN[OK]$COL_RESET - Synchronization of ws_ros on Nao"
rsync -az --rsync-path="mkdir -p /home/nao/ws_ros && rsync" /home/nao/ws_ros/install_isolated nao@$1:/home/nao/ws_ros/

echo -e "$COL_GREEN[OK]$COL_RESET - Synchronization of ws_ros_additional_packages on Nao"
rsync -az --rsync-path="mkdir -p /home/nao/ws_ros_additional_packages && rsync" /home/nao/ws_ros_additional_packages/install_isolated nao@$1:/home/nao/ws_ros_additional_packages/
rsync -az --rsync-path="mkdir -p /home/nao/ws_ros_additional_packages && rsync"  /home/nao/gcc-4.8.4/lib/libstdc++.so.6 nao@$1:/home/nao/ws_ros_additional_packages/install_isolated/lib/libstdc++.so.6
rsync -az --rsync-path="mkdir -p /home/nao/ws_ros_additional_packages && rsync"  /home/nao/ws_rapp_nao/src/rapp-robot-nao/rapp_ros_naoqi_wrappings/ros_packages_config nao@$1:/home/nao/ws_ros_additional_packages/
echo -e "$COL_GREEN[OK]$COL_RESET - Synchronization of nao_scripts on Nao"
rsync -az --rsync-path="mkdir -p /home/nao/scripts && rsync" /home/nao/ws_rapp_nao/src/rapp-robot-nao/scripts/nao_scripts/ nao@$1:/home/nao/scripts/

