#!/bin/bash

# written by Maksym Figat

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <VM IP ADDRESS> <NAME OF PACKAGE>"
  exit 1
fi

RAPP_PATH="/home/max"

cd $RAPP_PATH/rapp/generated_hz_packages
scp nao@$1:/home/nao/rapp/hz_packages/$2-1.0.0.hz .
