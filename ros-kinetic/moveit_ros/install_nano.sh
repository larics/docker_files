#!/bin/bash 

wget https://mirror.easyname.at/gnu/nano/nano-4.8.tar.gz
tar -xf nano-4.8.tar.gz
cd nano-4.8
./configure --enable-utf8
make 
sudo apt-get remove nano
sudo make install 
source ~/.bashrc

