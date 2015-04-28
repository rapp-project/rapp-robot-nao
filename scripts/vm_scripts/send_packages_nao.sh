#!/bin/bash

# written by Maksym Figat

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <NAO_IP_ADDRESS>"
	exit 1
fi

echo "Packing rapp packages /home/nao/ws_rapp/install_isolated/"
bash /home/nao/ws_rapp_nao/src/rapp-robot-nao/scripts/vm_scripts/vm_archive_workspaces.sh

echo "Sending packages to nao $1:/home/nao/download/"
scp /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz /home/nao/ws_ros.tar.gz nao@$1:/home/nao/

echo "Removing echo /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz"
rm /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz /home/nao/ws_ros.tar.gz

  
