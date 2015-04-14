#!/bin/bash

# written by Maksym Figat

echo "Removing packages"
bash /home/nao/scripts/remove_rapp_packages.sh
echo "Unzip rapp packages"
bash /home/nao/scripts/unzip_rapp_packages.sh
echo "Catkin make isolated"
bash /home/nao/scripts/catkin_make_isolated.sh
