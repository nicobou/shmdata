#!/usr/bin/env bash
set -e
sed -iold 's/isblank/MACROIsBlank/' pjlib/include/pj/compat/ctype.h
sed -iold 's/isblank/MACROIsBlank/' pjlib/include/pj/compat/ctype.h
sed -iold 's/isblank/MACROIsBlank/' pjlib/include/pj/ctype.h
sed -iold 's/pj_ioqueue_create(icedemo.pool, 16/pj_ioqueue_create(icedemo.pool, 128/' pjsip-apps/src/samples/icedemo.c
sed -iold 's/buffer\[1000\]/buffer\[1000000\]/' pjsip-apps/src/samples/icedemo.c
sed -iold 's/REGC_TSX_TIMEOUT\t33000/REGC_TSX_TIMEOUT  3000/' pjsip/src/pjsip-ua/sip_reg.c