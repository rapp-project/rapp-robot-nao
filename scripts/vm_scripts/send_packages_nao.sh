#!/bin/bash

# written by Maksym Figat

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <NAO_IP_ADDRESS>"
	exit 1
fi

echo "Packing rapp packages /home/nao/ws_rapp/install_isolated/"
bash /home/nao/script/rapp_tar_packages.sh
echo "Sending packages to nao $1:/home/nao/download/"
scp /home/nao/packages/rapp_install_isolated.tar.gz nao@$1:/home/nao/download/

  
