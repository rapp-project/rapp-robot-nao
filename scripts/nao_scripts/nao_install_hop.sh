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

# Bigloo
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Bigloo"
wget ftp://ftp-sop.inria.fr/indes/rapp/hop/2015-05-07/bigloo4.2a.tar.gz
tar zxvf bigloo4.2a.tar.gz
cd bigloo4.2a
./configure #--prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make
sudo make install
make clean

# Hop
cd $PROGRAMS_DIRECTORY
echo -e "$COL_GREEN[OK]$COL_RESET - Downloading source code of Hop"
wget ftp://ftp-sop.inria.fr/indes/rapp/hop/2015-05-07/hop-3.0.0-pre14.tar.gz
tar zxvf hop-3.0.0-pre14.tar.gz
cd hop-3.0.0-pre14
./configure #--prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make
sudo make install
make clean
