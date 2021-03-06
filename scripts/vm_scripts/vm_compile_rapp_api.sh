#!/bin/bash

# written by Wojciech Dudek

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"

# Clone rapp-api repositories 
WS_RAPP_API_DIR="/home/nao/ws_rapp_api"
mkdir -p $WS_RAPP_API_DIR/src
cd $WS_RAPP_API_DIR/src
git clone -b cpp-wut https://github.com/rapp-project/rapp-api.git
git clone -b wut https://github.com/rapp-project/rapp-robots-api.git
# Rapp - ws_rapp_nao - core agent packages
mkdir -p $WS_RAPP_API_DIR/build/rapp-api
cd $WS_RAPP_API_DIR/build/rapp-api
cmake $WS_RAPP_API_DIR/src/rapp-api/cpp -DOPEN_NAO=ON -DCMAKE_INSTALL_PREFIX=$WS_RAPP_API_DIR/install
make 
make install

# rapp-robots-api
mkdir -p $WS_RAPP_API_DIR/build/rapp-robots-api
cd $WS_RAPP_API_DIR/build/rapp-robots-api
source $WS_RAPP_API_DIR/install/setup.bash
cmake $WS_RAPP_API_DIR/src/rapp-robots-api -DCMAKE_INSTALL_PREFIX=$WS_RAPP_API_DIR/install -DRapp_DIR=$WS_RAPP_API_DIR/install/share/Rapp -DBUILD_ALL=ON -DROBOT_ARCH_TYPE=Nao
make
make install

# rapp-cloud-api
#mkdir -p $WS_RAPP_API_DIR/build/rapp-cloud-api
#cd $WS_RAPP_API_DIR/build/rapp-cloud-api
#cmake $WS_RAPP_API_DIR/src/rapp-cloud-api -DCMAKE_INSTALL_PREFIX=$WS_RAPP_API_DIR/install
#make
#make install
