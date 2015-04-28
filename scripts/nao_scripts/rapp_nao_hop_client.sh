#!/bin/bash

# written by Maksym Figat

cd /home/nao/ws_ros_additional_packages/install_isolated/bin 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nao/ws_ros_additional_packages/install_isolated/lib/bigloo/4.2a/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/nao/ws_ros_additional_packages/install_isolated/lib/hop/3.0.0/
./hop -v -g /home/nao/ws_rapp_applications/rapp-applications/nao/hz_packeges/store_interaction.js
