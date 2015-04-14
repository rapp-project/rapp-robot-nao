#!/bin/bash

# author: Maksym Figat

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <port number>"
  exit 1
fi

/home/max/programs/hop-3.0.0-pre14/bin/hop -p $1
