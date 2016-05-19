#!/bin/bash

# written by Maksym Figat

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"

if [ "$#" -ne 1 ]; then
  echo "Enter a name of an application"
  exit 1
else
  RAPP_BASE="/home/nao"
  WS_RAPP_API_DIR=$RAPP_BASE"/ws_rapp_api"

  export Rapp_DIR=$WS_RAPP_API_DIR/install/share/Rapp
  export RappRobots_DIR=$WS_RAPP_API_DIR/install/share/RappRobots
  export LD_LIBRARY_PATH=$WS_RAPP_API_DIR/install/lib:$LD_LIBRARY_PATH

  GIT_WS_RAPP_HACKATON_APPS_DIR=$RAPP_BASE"/ws_rapp_hacaton_apps"
  GIT_WS_RAPP_BUILD_HACKATON_APPLICATION_DIR=$GIT_WS_RAPP_HACKATON_APPS_DIR/build/$1

  if [ ! -d $GIT_WS_RAPP_BUILD_HACKATON_APPLICATION_DIR ]; then
    mkdir -p $GIT_WS_RAPP_BUILD_HACKATON_APPLICATION_DIR
  else
    rm $GIT_WS_RAPP_BUILD_HACKATON_APPLICATION_DIR -rf
    mkdir -p $GIT_WS_RAPP_BUILD_HACKATON_APPLICATION_DIR
  fi

  cd $GIT_WS_RAPP_BUILD_HACKATON_APPLICATION_DIR
  cmake  $GIT_WS_RAPP_HACKATON_APPS_DIR/src/$1 -DCMAKE_INSTALL_PREFIX=$GIT_WS_RAPP_HACKATON_APPS_DIR/install/$1
  make install
fi
