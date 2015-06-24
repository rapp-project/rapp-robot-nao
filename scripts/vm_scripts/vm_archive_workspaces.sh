#!/bin/bash

# written by Maksym Figat

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"

echo -e "$COL_GREEN[OK]$COL_RESET - Archives ws_ros workspace"
cd /home/nao/ws_ros
tar czf /home/nao/ws_ros.tar.gz install_isolated

echo -e "$COL_GREEN[OK]$COL_RESET - Archives ws_ros_additional_packages workspace"
cd /home/nao/ws_ros_additional_packages
tar czf /home/nao/ws_ros_additional_packages.tar.gz install_isolated

echo -e "$COL_GREEN[OK]$COL_RESET - Archives ws_rapp_nao workspace"
cd /home/nao/ws_rapp_nao
tar czf /home/nao/ws_rapp_nao.tar.gz install_isolated

echo -e "$COL_GREEN[OK]$COL_RESET - Archives ws_rapp_applications_nao workspace"
cd /home/nao/ws_rapp_applications_nao
tar czf /home/nao/ws_rapp_applications_nao.tar.gz install_isolated

echo -e "$COL_GREEN[OK]$COL_RESET - Archives nao data - sound and pictures needed for voicemail"
cd /home/nao/ws_rapp_applications/rapp-applications/nao
tar czf /home/nao/nao_data.tar.gz data hz_packages/store_interaction.js
