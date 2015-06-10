#!/bin/bash

# written by Maksym Figat

echo "Archives ws_ros workspace"
cd /home/nao/ws_ros
tar czf /home/nao/ws_ros.tar.gz install_isolated

echo "Archives ws_ros_additional_packages workspace"
cd /home/nao/ws_ros_additional_packages
tar czf /home/nao/ws_ros_additional_packages.tar.gz install_isolated

echo "Archives ws_rapp_nao workspace"
cd /home/nao/ws_rapp_nao
tar czf /home/nao/ws_rapp_nao.tar.gz install_isolated

echo "Archives nao data - sound and pictures needed for voicemail"
cd /home/nao/ws_rapp_applications/rapp-applications
tar czf /home/nao/nao_data.tar.gz nao
