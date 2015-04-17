#!/bin/bash

# written by Maksym Figat

PATH_TO_SCRIPTS="/home/max/scripts/rapp/"

if [ "$#" -ne 4 ]; then
	echo "Generates hz package"
	echo "Usage: $0 <VM_IP_ADDRESS> <package_name> <FLAG1> <FLAG2>"
	echo "FLAG1 = 0 - without downloading repository from github"
	echo "FLAG1 = 1 - downloading repository from github"
	echo "FLAG2 = 0 - without compilation"
	echo "FLAG2 = 1 - with compilation"
	exit 1
fi

echo "Connects by ssh to virtual machine. Enter --  nao -- as a password"
ssh nao@$1 << EOF
echo "Invokes preparation script: vm_preparation_to_build_packages.sh"
bash ~/scripts/vm_preparation_to_build_packages.sh $3
echo "Generating hz package: $2"
bash ~/scripts/rapp_create_hz.sh $2 $4 
EOF

echo "Downloading hz package $2 from virtual machine $1"
bash $PATH_TO_SCRIPTS/rappstore_get_hz.sh $1 $2

exit

