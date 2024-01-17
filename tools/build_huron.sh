#!/bin/sh

echo "Target platform: $1"

#Build huron
cd /huron
export LD_LIBRARY_PATH=/usr/local/lib:/usr/lib
cmake -DBUILD_TYPE=$1 -Bbuild
cmake --build build --parallel --target install
cd build && ctest
