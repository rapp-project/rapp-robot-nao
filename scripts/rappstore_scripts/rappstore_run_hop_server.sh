#!/bin/bash

# author: Maksym Figat

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <port number>"
  exit 1
fi

/usr/local/bin/hop -p $1
