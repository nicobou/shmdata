#!/bin/bash
cd pjproject-2.2.1
mkdir install_dir
patch -p0 < ../more_codecs.patch
./configure --prefix `pwd`/install_dir --disable-video  --disable-sound --disable-g711-codec --disable-g7221-codec  --disable-g722-codec --disable-gsm-codec --disable-ilbc-codec --disable-l16-codec --disable-speex-aec  --disable-speex-codec
CFLAGS=-fPIC make dep
CFLAGS=-fPIC make
make install

