#!/bin/bash

# written by Maksym Figat

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <NAO_IP_ADDRESS>"
	exit 1
fi

NAO_SCRIPTS="/home/scripts"

echo "Packing rapp packages /home/nao/ws_rapp/install_isolated/"
bash /home/nao/scripts/vm_scripts/vm_archive_workspaces.sh

echo "Packing scripts for Nao robot"
cd /home/nao/ws_rapp_nao/src/rapp-robot-nao/scripts/
tar czf /home/nao/nao_scripts.tar.gz nao_scripts/

echo "Sending packages to nao $1:/home/nao/"
scp /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz /home/nao/ws_ros.tar.gz nao@$1:/home/nao/

echo "Sending scripts to nao $1:/home/nao/"
scp /home/nao/nao_scripts.tar.gz nao@$1:/home/nao/

echo "Sending nao data to nao $1:/home/nao/"
scp /home/nap/nao_data.tar.gz nao@$1:/home/nao/

echo "Removing /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz"
rm /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz /home/nao/ws_ros.tar.gz

echo "Removing /home/nao/nao_scripts.tar.gz"
rm /home/nao/nao_scripts.tar.gz

echo "Connecting with $1 by ssh. Write password for your NAO (nao)"
ssh nao@$1<< EOF
if [ ! -d $NAO_SCRIPTS ]; then #If NAO_SCRIPTS doesnt exist
   echo "Creating $NAO_SCRIPTS folder"  
   mkdir $NAO_SCRIPTS
fi
cd $NAO_SCRIPTS
echo "Unpacking scripts for nao into /home/scripts/"
tar xf ../nao_scripts.tar.gz

bash $NAO_SCRIPTS/nao_scripts/nao_extract_workspaces.sh
EOF

echo "Disconnecting with NAO robot $1" 
exit
  
