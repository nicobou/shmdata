#!/usr/bin/env bash

echo SHMDATA BUILD
git clone https://gitlab.com/sat-metalab/shmdata.git libshmdata
cd libshmdata
git checkout $1
mkdir build
cd build
cmake ..
make -j"$(nproc)"
make install
ldconfig
