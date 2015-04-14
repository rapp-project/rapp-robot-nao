#!/bin/bash

# written by Maksym Figat

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <NAO address IP>"
  exit 1
fi

cd /home/nao/my_workspace/

cp /usr/lib/liblog4cxx* install_isolated/lib/
cp -r /usr/include/log4cxx install_isolated/include/
cp /usr/lib/libapr* install_isolated/lib/
cp -r /usr/include/apr* install_isolated/include/
cp /usr/lib/libtinyxml* install_isolated/lib/
cp /usr/lib/libPoco* install_isolated/lib/
cp -r /usr/include/Poco* install_isolated/include/
cp -r /usr/lib/liburdfdom* install_isolated/lib/
cp -r /usr/include/urdf* install_isolated/include/
cp -r /usr/lib/libcxsparse* install_isolated/lib/
cp -r /usr/lib/libcholmod* install_isolated/lib/
cp -r /usr/include/cholmod* install_isolated/include/
cp -r /usr/lib/liblz4* install_isolated/lib/
cp -r /usr/include/lz4* install_isolated/include/
cp -r /usr/local/lib/libconsole_bridge* install_isolated/lib/
cp -r /usr/local/include/console_bridge* install_isolated/include/
cp -r /usr/lib/python2.7/site-packages/* install_isolated/lib/python2.7/site-packages/
cp /usr/bin/rosversion install_isolated/bin/
cp /usr/bin/ros* install_isolated/bin/

cp /usr/local/lib/libvmime* install_isolated/lib/
cp /usr/local/include/vmime install_isolated/include/

cp /usr/local/lib/libgsasl* install_isolated/lib/
cp /usr/local/include/gsasl* install_isolated/include/

tar -czvf install_clean.tar.gz install_isolated
scp install_clean.tar.gz nao@$1:/home/nao/my_workspace/
