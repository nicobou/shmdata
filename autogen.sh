#!/bin/bash

autoreconf --install --verbose

if [ $? != 0 ]; then
    echo "autoreconf return value is $?"
    exit 1
fi