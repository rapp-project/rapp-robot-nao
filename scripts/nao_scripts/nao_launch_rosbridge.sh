#!/bin/bash

# written by Maksym Figat

export PYTHONPATH=$PYTHONPATH:/home/nao/ws_ros_additional_packages/install_isolated/lib/python2.7/site-packages/pytz-2015.7-py2.7.egg:/home/nao/ws_ros_additional_packages/install_isolated/lib/python2.7/site-packages/six-1.10.0-py2.7.egg:/home/nao/ws_ros_additional_packages/install_isolated/lib/python2.7/site-packages/six-1.10.0-py2.7.egg
source /home/nao/ws_rapp_nao/install_isolated/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
