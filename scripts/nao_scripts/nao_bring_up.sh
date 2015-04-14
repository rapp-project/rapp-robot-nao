#!/bin/bash
# written by Maksym Figat 

if [ "$#" -ne 1 ]; then
  echo "Usage: bash $0 <IP_ADDRESS>"
  exit 1
else
  source ~/my_workspace/install_isolated/setup.bash
  export ROS_MASTER_URI=http://$1:11311
  export ROS_IP=$1
  roslaunch nao_bringup nao.launch force_python:=true
fi

