#!/bin/bash

# written by Maksym Figat

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <package_name>"
  exit 1
fi

echo "Setting - system variable"
# Set RAPP_PATH to "/home/nao" if you are running script on nao robot or VM.
RAPP_PATH="/home/nao"
RAPP_DIRECTORY=$RAPP_PATH"/rapp/hz_packages/packages/$1"
RAPP_WORKSPACE=$RAPP_PATH"/ws_rapp"
HZ_PACKAGES="/home/nao/rapp/hz_packages/"

echo "Removing - devel/build/install _ isolated folders from workspace: /home/nao/ws_rapp"
rm $RAPP_PATH/ws_rapp/build_isolated /home/nao/ws_rapp/devel_isolated /home/nao/ws_rapp/install_isolated -rf

echo "Building - packages from workspace: /home/nao/ws_rapp"
bash $RAPP_PATH/scripts/catkin_make_isolated.sh

echo "Creating - temporary catalog for package: $1"

if [ -d $RAPP_DIRECTORY ]; then #If package exists
  echo "Removing - a package. Package $1 already exists."
  rm $HZ_PACKAGES/packages/$1 -rf
  rm $HZ_PACKAGES/hz/$1* -rf
fi

mkdir -p $HZ_PACKAGES/packages/$1/rapp_dynamic_agent/lib
mkdir -p $HZ_PACKAGES/packages/$1/rapp_dynamic_agent/share

echo "Copying - $1 libs and shares folders to corresponding folders in: $RAPP_DIRECTORY"
cp -r $RAPP_WORKSPACE/install_isolated/lib/$1 $HZ_PACKAGES/packages/$1/rapp_dynamic_agent/lib/
cp -r $RAPP_WORKSPACE/install_isolated/share/$1 $HZ_PACKAGES/packages/$1/rapp_dynamic_agent/share/

echo "Copying - hz template files to temporary catalog for package: $1"
cp -r $HZ_PACKAGES/template_package/* $HZ_PACKAGES/packages/$1/

echo "Creates - hz package $1-1.0.0.hz"
tar czvf $HZ_PACKAGES/hz/$1-1.0.0.hz $HZ_PACKAGES/packages/$1

exit 0
