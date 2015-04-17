#!/bin/bash

# written by Maksym Figat

RAPP_USER="/home/max"
HZ_DIRECTORY=$RAPP_USER"/rapp/hz_packages/hz"

if [ "$#" -ne 2 ]; then
	echo "Script that downloads a given RApp as a hz from virtual machine."
	echo "If script doesnt work change the RAPP_USER to /home/<YOUR_USER>"
	echo "Usage: $0 <VM_IP_ADDRESS> <PACKAGE_NAME> "
	echo "VM_IP_ADDRESS - ip address of virtual machine from which hz will be downloaded"
	echo "PACKAGE_NAME - name of RApp e.g. helloworld, voicemail "
	exit 1
fi

if [ -d $HZ_DIRECTORY ]; then #If package exists
	echo "$HZ_DIRECTORY already exists."  
else
	mkdir -p $HZ_DIRECTORY
fi

scp nao@$1:/home/nao/rapp/hz_packages/hz/$2-1.0.0.hz $HZ_DIRECTORY
