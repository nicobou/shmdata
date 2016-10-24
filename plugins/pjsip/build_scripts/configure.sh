#!/usr/bin/env bash
set -e
CFLAGS="-DPJ_ICE_ST_MAX_CAND=512 -DPJ_ICE_MAX_CHECKS=512 -DPJ_ICE_COMP_BITS=8 -DPJ_ICE_MAX_CAND=1024 -DPJ_ICE_CAND_TYPE_PREF_BITS=16 -DPJ_IOQUEUE_MAX_EVENTS_IN_SINGLE_POLL=512 -DPJ_IOQUEUE_MAX_HANDLES=512 -DPJSIP_MAX_PKT_LEN=8000 -DPJSIP_TD_TIMEOUT=5000" \
./configure --prefix `pwd`/install_dir --disable-video --disable-sound --disable-g711-codec --disable-g7221-codec \
--disable-g722-codec --disable-gsm-codec --disable-ilbc-codec --disable-l16-codec --disable-speex-aec  --disable-speex-codec