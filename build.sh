#!/bin/bash

################################################################################

./clean.sh

mkdir build
cd build

if [ "$1" = "--with-tests" ]; then
    cmake .. \
	-DTESTING=On \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX=/usr/local
else
    cmake .. \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX=/usr/local
fi

make -j4

################################################################################
