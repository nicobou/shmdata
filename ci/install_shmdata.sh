#!/usr/bin/env bash

shmdata_branch = "develop"

if [[ "$1" = "master" ]]; then
    shmdata_branch = "master"
fi

echo SHMDATA BUILD
git clone https://github.com/sat-metalab/shmdata.git libshmdata
cd libshmdata
git checkout ${shmdata_branch}
mkdir build
cd build
cmake ..
make -j"$(nproc)"
make install
ldconfig
