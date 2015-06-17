#!/bin/bash

# written by Maksym Figat

ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"

HZ_PACKAGES_FOLDER="/home/nao/hz_packages"

if [ "$#" -ne 1 ]; then
  echo -e "$COL_RED[Error] - Usage:$COL_RESET - $COL_GREEN$0 <package_name>$COL_RESET"
  exit 1
fi

cd $HZ_PACKAGES_FOLDER/packages/$1

echo -e "$COL_GREEN[Ok]$COL_RESET - Generating $1.js file in $HZ_PACKAGES_FOLDER/packages/$1 folder"


