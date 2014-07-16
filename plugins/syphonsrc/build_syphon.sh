#!/bin/bash
cd Syphon-framework
xcodebuild -project Syphon.xcodeproj -configuration Release $@ SYMROOT=./build DSTROOT=/
