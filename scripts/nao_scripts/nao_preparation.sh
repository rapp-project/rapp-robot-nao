#!/bin/bash

RECORDINGS_PATH="/home/nao/recordings/microphones"

# If folder doesnt exist
if [ ! -d $RECORDINGS_PATH ]; then
	echo -e "$COL_GREEN[OK]$COL_RESET - Creating $RECORDINGS_PATH directory."
	mkdir -p $RECORDINGS_PATH
fi

