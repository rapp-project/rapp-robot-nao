#!/bin/bash

# written by Maksym Figat

RAPP_USER="/home/max"
HZ_DIRECTORY=$RAPP_USER"/rapp/hz_packages/hz"
HZ_PACKAGES_FOLDER="/home/nao/hz_packages/hz"

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

echo "Downloading hz package from virtaul machine. Enter --  nao -- as a password"
scp nao@$1:$HZ_PACKAGES_FOLDER/$2-1.0.0.hz $HZ_DIRECTORY

echo "List of hz packages:"
cd $HZ_DIRECTORY
ls
