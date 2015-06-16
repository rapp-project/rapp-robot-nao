#!/bin/bash

# written by Maksym Figat

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

echo -e "$COL_GREEN[OK]$COL_RESET - Packing rapp packages /home/nao/ws_rapp/install_isolated/"
bash /home/nao/scripts/vm_archive_workspaces.sh

echo -e "$COL_GREEN[OK]$COL_RESET - Packing scripts for Nao robot"
cd /home/nao/ws_rapp_nao/src/rapp-robot-nao/scripts/
tar czf /home/nao/nao_scripts.tar.gz nao_scripts/

echo -e "$COL_GREEN[OK]$COL_RESET - Sending packages to nao $1:/home/nao/"
echo -e "$COL_GREEN[OK] - Enter password for NAO [nao] $COL_RESET"
scp /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz /home/nao/ws_ros.tar.gz nao@$1:/home/nao/

echo -e "$COL_GREEN[OK]$COL_RESET - Sending scripts to nao $1:/home/nao/"
echo -e "$COL_GREEN[OK] - Enter password for NAO [nao] $COL_RESET"
scp /home/nao/nao_scripts.tar.gz nao@$1:/home/nao/

echo -e "$COL_GREEN[OK]$COL_RESET - Sending nao data to nao $1:/home/nao/"
echo -e "$COL_GREEN[OK] - Enter password for NAO [nao] $COL_RESET"
scp /home/nao/nao_data.tar.gz nao@$1:/home/nao/

echo -e "$COL_GREEN[OK]$COL_RESET - Removing /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz"
rm /home/nao/ws_rapp_nao.tar.gz /home/nao/ws_ros_additional_packages.tar.gz /home/nao/ws_ros.tar.gz

echo -e "$COL_GREEN[OK]$COL_RESET - Removing /home/nao/nao_scripts.tar.gz"
rm /home/nao/nao_scripts.tar.gz

echo -e "$COL_GREEN[OK]$COL_RESET - Removing /home/nao/nao_data.tar.gz"
rm /home/nao/nao_data.tar.gz

echo -e "$COL_GREEN[OK]$COL_RESET - Connecting with $1 by ssh."
echo -e "$COL_GREEN[OK] - Enter password for NAO [nao] $COL_RESET"
ssh nao@$1<< EOF
if [ ! -d $NAO_SCRIPTS ]; then #If NAO_SCRIPTS doesnt exist
   echo -e "$COL_GREEN[OK]$COL_BLUE - $1 $COL_RESET - Creating $NAO_SCRIPTS folder"  
   mkdir $NAO_SCRIPTS
fi
cd $NAO_SCRIPTS
echo -e "$COL_GREEN[OK]$COL_BLUE - $1 $COL_RESET - Unpacking scripts for nao into /home/scripts/"
tar xf ../nao_scripts.tar.gz

bash $NAO_SCRIPTS/nao_scripts/nao_extract_workspaces.sh
EOF

echo -e "$COL_GREEN[OK]$COL_RESET - Disconnecting with NAO robot $1" 
exit
  
