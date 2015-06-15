#!/bin/bash

# written by Maksym Figat

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"

PROGRAMS_DIRECTORY="/home/nao/programs"

if [ -d $PROGRAMS_DIRECTORY ]; then #If directory exists
	echo -e "Folder $PROGRAMS_DIRECTORY exists"
else
	mkdir -p $PROGRAMS_DIRECTORY
	echo -e "$COL_GREEN[OK]$COL_RESET - Creating $PROGRAMS_DIRECTORY folder"
fi

# Openssl
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Openssl"
wget https://www.openssl.org/source/openssl-1.0.2c.tar.gz
tar zxvf openssl-1.0.2c.tar.gz
cd openssl-1.0.2c
./config --prefix=/usr/local --openssldir=/usr/local/openssl
make
sudo make install
make clean

# Exporting LIBRARY PATH
echo -e "$COL_GREEN[OK]$COL_RESET - Exporting LIBRARY PATH"
export LIBRARY_PATH=/usr/local/lib

# Bigloo
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Bigloo"
wget ftp://ftp-sop.inria.fr/indes/fp/Bigloo/bigloo4.2a-alpha22Apr15.tar.gz
tar zxvf bigloo4.2a-alpha22Apr15.tar.gz
cd bigloo4.2a
./configure #--prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make
sudo make install
make clean

# Hop
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Hop"
wget ftp://ftp-sop.inria.fr/indes/fp/Hop/hop-3.0.0-pre15.tar.gz
tar zxvf hop-3.0.0-pre15.tar.gz
cd hop-3.0.0-pre15
./configure #--prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make
sudo make install
make clean
