#!/bin/bash

# written by Maksym Figat


if [ "$#" -ne 1 ]; then
	echo "Enter a name of an application"
  	exit 1
else
	export LD_LIBRARY_PATH=/home/nao/ws_rapp_api/install/lib:$LD_LIBRARY_PATH
	source /home/nao/ws_rapp_nao/install_isolated/setup.bash
	cd /home/nao/ws_rapp_hackaton_apps/install/$1
	sh ./run.sh `pwd`
fi
