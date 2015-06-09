#!/bin/bash

# written by Maksym Figat

cd /home/nao/ws_ros
tar czf /home/nao/ws_ros.tar.gz install_isolated

cd /home/nao/ws_ros_additional_packages
tar czf /home/nao/ws_ros_additional_packages.tar.gz install_isolated

cd /home/nao/ws_rapp_nao
tar czf /home/nao/ws_rapp_nao.tar.gz install_isolated
