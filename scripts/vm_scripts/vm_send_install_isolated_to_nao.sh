#!/bin/bash

# written by Maksym Figat

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <NAO address IP>"
  exit 1
fi

cd /home/nao/ws_ros/

tar -czvf install_clean.tar.gz install_isolated
scp install_clean.tar.gz nao@$1:/home/nao/ws_ros/
