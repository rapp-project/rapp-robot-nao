#!/bin/bash

# written by Maksym Figat

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_BLUE=$ESC_SEQ"34;01m"
COL_RED=$ESC_SEQ"31;01m"

if [ "$#" -ne 1 ]; then
	echo -e "$COL_RED[Error]$COL_RESET - Usage: $COL_GREEN$0 <NAO_IP_ADDRESS>$COL_RESET"
	exit 1
fi

echo -e "$COL_GREEN[OK]$COL_RESET - Synchronization of ws_rapp_hackaton_apps on Nao"
rsync -az --rsync-path="mkdir -p /home/nao/ws_rapp_hackaton_apps && rsync" /home/nao/ws_rapp_hackaton_apps/install nao@$1:/home/nao/ws_rapp_hackaton_apps/