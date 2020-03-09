#!/bin/bash
# Author: Matt Bunting

echo "----------------------------"
echo "Installing libpandac:"

mkdir build
cd build
cmake ..
make -j4
make install
cd ..

echo "Installing panda services:"
cd scripts/panda
./install.sh

echo "Done."
echo "----------------------------"
