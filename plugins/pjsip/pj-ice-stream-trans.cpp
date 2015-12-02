/*
 * This file is part of switcher-pjsip.
 *
 * switcher-pjsip is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "glib.h"
#include "./pj-ice-stream-trans.hpp"

namespace switcher {
PJICEStreamTrans::PJICEStreamTrans(pj_ice_strans_cfg &ice_cfg,
                                   unsigned comp_cnt,
                                   pj_ice_sess_role role){
  pj_ice_strans_cb icecb;
  /* init the callback */
  pj_bzero(&icecb, sizeof(icecb));
  icecb.on_rx_data = cb_on_rx_data;
  icecb.on_ice_complete = cb_on_ice_complete;
  /* create the instance */
  if (PJ_SUCCESS != pj_ice_strans_create("icetest", /* object name */
                                         &ice_cfg,  /* settings */
                                         comp_cnt,  /* comp_cnt */
                                         NULL,      /* user data */
                                         &icecb,    /* callback */
                                         &icest_)){ /* instance ptr */
    g_warning("error creating ice");
    return;
  }
  if (PJ_SUCCESS != pj_ice_strans_init_ice(icest_,
                                           role,
                                           NULL,
                                           NULL)){
    g_warning("error initializing ICE");
    return;
  }
  //done
  is_valid_ = true;
}

PJICEStreamTrans::~PJICEStreamTrans() {
  if(nullptr != icest_){
    if (pj_ice_strans_has_sess(icest_)) {
      if(PJ_SUCCESS != pj_ice_strans_stop_ice(icest_))
        g_warning("issue stoping ICE session");
    }
    pj_ice_strans_destroy(icest_);
  }
  // FIXME wait for destruction done ??
}

/*
 * This is the callback that is registered to the ICE stream transport to
 * receive notification about incoming data. By "data" it means application
 * data such as RTP/RTCP, and not packets that belong to ICE signaling (such
 * as STUN connectivity checks or TURN signaling).
 */
void PJICEStreamTrans::cb_on_rx_data(pj_ice_strans *ice_st,
                                     unsigned comp_id, 
                                     void *pkt, pj_size_t size,
                                     const pj_sockaddr_t *src_addr,
                                     unsigned src_addr_len){
  char ipstr[PJ_INET6_ADDRSTRLEN+10];
  PJ_UNUSED_ARG(ice_st);
  PJ_UNUSED_ARG(src_addr_len);
  PJ_UNUSED_ARG(pkt);
  // Don't do this! It will ruin the packet buffer in case TCP is used!
  //((char*)pkt)[size] = '\0';
  g_print("Component %d: received %lu bytes data from %s: \"%.*s\"",
          comp_id, size,
          pj_sockaddr_print(src_addr, ipstr, sizeof(ipstr), 3),
          (unsigned)size,
          (char*)pkt);
}

/*
 * This is the callback that is registered to the ICE stream transport to
 * receive notification about ICE state progression.
 */
void PJICEStreamTrans::cb_on_ice_complete(pj_ice_strans *ice_st, 
                                          pj_ice_strans_op op,
                                          pj_status_t status){
  const char *opname = 
      (op==PJ_ICE_STRANS_OP_INIT? "initialization" :
       (op==PJ_ICE_STRANS_OP_NEGOTIATION ? "negotiation" : "unknown_op"));
  
  if (status == PJ_SUCCESS) {
    g_debug("ICE %s successful", opname);
  } else {
    char errmsg[PJ_ERR_MSG_SIZE];
    pj_strerror(status, errmsg, sizeof(errmsg));
    g_warning("ICE %s failed: %s", opname, errmsg);
    // pj_ice_strans_destroy(ice_st);
    // icedemo.icest = NULL;
  }
}

}  // namespace switcher
