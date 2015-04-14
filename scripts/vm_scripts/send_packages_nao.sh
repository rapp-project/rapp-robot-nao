#!/bin/bash

# written by Maksym Figat

echo "Packing rapp packages /home/nao/ws_rapp/install_isolated/"
bash /home/nao/script/pack_rapp_packages.sh
echo "Sending packages to nao 192.168.18.91:/home/nao/download/rapp"
scp /home/nao/packages/rapp_install_isolated.tar.gz nao@192.168.18.91:/home/nao/download/rapp/

  
