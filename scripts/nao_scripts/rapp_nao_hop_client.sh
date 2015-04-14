#!/bin/bash

# written by Maksym Figat

cd ~/hop/hop-3.0.0/bin 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/hop/bigloo/lib/bigloo/4.2a/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/hop/hop-3.0.0/lib/hop/3.0.0/
./hop -v -g ../../store_interaction.js
