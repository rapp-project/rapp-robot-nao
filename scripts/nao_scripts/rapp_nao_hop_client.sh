#!/bin/bash

# written by Maksym Figat

cd ~/hop/hop-3.0.0/bin 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/my_workspace/install_isolated/lib/bigloo/4.2a/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/my_workspace/install_isolated/lib/hop/3.0.0/
./hop -v -g ~/rapp/store_interaction.js
