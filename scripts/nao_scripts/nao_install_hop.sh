#!/bin/bash

# written by Maksym Figat

PROGRAMS_DIRECTORY="/home/nao/programs"

if [ -d $PROGRAMS_DIRECTORY ]; then #If directory exists
	echo "Folder $PROGRAMS_DIRECTORY exists"
else
	mkdir -p $PROGRAMS_DIRECTORY
	echo "Creating $PROGRAMS_DIRECTORY folder"
fi

# Bigloo
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of Bigloo"
wget ftp://ftp-sop.inria.fr/indes/fp/Bigloo/bigloo4.2a-alpha22Apr15.tar.gz
tar zxvf bigloo4.2a-alpha22Apr15.tar.gz
cd bigloo4.2a
./configure #--prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make
sudo make install
make clean

# Hop
cd $PROGRAMS_DIRECTORY
echo "Downloading source code of Hop"
wget ftp://ftp-sop.inria.fr/indes/fp/Hop/hop-3.0.0-pre14.tar.gz
tar zxvf hop-3.0.0-pre14.tar.gz
cd hop-3.0.0-pre14
./configure #--prefix=$ROS_ADDITIONAL_PACKAGES_ISOLATED
make
sudo make install
make clean
