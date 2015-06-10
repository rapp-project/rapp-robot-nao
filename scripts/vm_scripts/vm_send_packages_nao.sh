#!/bin/bash

# written by Maksym Figat

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <NAO_IP_ADDRESS>"
	exit 1
fi

echo "Packing rapp packages /home/nao/ws_rapp/install_isolated/"
bash /home/nao/ws_rapp_nao/src/rapp-robot-nao/scripts/vm_scripts/vm_archive_workspaces.sh

echo "Packing scripts for Nao robot"
cd /home/nao/ws_rapp_nao/src/rapp-robot-nao/scripts/
tar czf /home/nao/nao_scripts.tar.gz nao_scripts/

echo "Sending packages to nao $1:/home/nao/"
scp /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz /home/nao/ws_ros.tar.gz nao@$1:/home/nao/

echo "Sending scripts to nao $1:/home/nao/"
scp /home/nao/nao_scripts.tar.gz nao@$1:/home/nao/

echo "Removing /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz"
rm /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz /home/nao/ws_ros.tar.gz

echo "Removing /home/nao/nao_scripts.tar.gz"
rm /home/nao/nao_scripts.tar.gz
  
