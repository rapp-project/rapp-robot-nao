#!/bin/bash

# written by Maksym Figat

echo "Unpacking a package in /home/nao/package/rapp"
cd /home/nao/ws_rapp
tar zxvf /home/nao/download/rapp/rapp_install_isolated.tar.gz

: 'echo "Removing lib - core_agent and rapp"
cd /home/nao/ws_rapp/install_isolated
rm lib/rapp_core_agent -rf
rm lib/rapp -rf

echo "Removing share - core agent and rapp"
rm share/rapp_core_agent -rf
rm share/rapp -rf

echo "Removing include - core agent and rapp"
rm include/rapp_core_agent -rf
rm include/rapp -rf

echo "Copy packages to correct folders"
cd /home/nao/packages/rapp/install_isolated
cp -r lib/rapp_core_agent /home/nao/ws_rapp/install_isolated/lib
cp -r lib/rapp /home/nao/ws_rapp/install_isolated/lib

cp -r share/rapp_core_agent /home/nao/ws_rapp/install_isolated/share
cp -r share/rapp /home/nao/ws_rapp/install_isolated/share

cp -r include/rapp_core_agent /home/nao/ws_rapp/install_isolated/include
cp -r include/rapp /home/nao/ws_rapp/install_isolated/include

'

