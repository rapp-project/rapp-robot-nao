#!/bin/bash

# written by Maksym Figat

# Script that is used for compilation packages on virtual machine
source /home/nao/my_workspace/install_isolated/setup.bash
cd /home/nao/ws_rapp/
/home/nao/my_workspace/src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

