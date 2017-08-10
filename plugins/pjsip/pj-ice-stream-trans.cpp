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

#include "./pj-ice-stream-trans.hpp"
#include <chrono>
#include <thread>
#include "./pj-sip-plugin.hpp"

namespace switcher {
PJICEStreamTrans::PJICEStreamTrans(pj_ice_strans_cfg& ice_cfg,
                                   unsigned comp_cnt,
                                   pj_ice_sess_role role)
    : comp_cnt_(comp_cnt), data_cbs_(comp_cnt, nullptr) {
  pj_ice_strans_cb icecb;
  /* init the callback */
  pj_bzero(&icecb, sizeof(icecb));
  icecb.on_rx_data = cb_on_rx_data;
  icecb.on_ice_complete = cb_on_ice_complete;
  /* create the instance */
  if (PJ_SUCCESS != pj_ice_strans_create("icetest",  /* object name */
                                         &ice_cfg,   /* settings */
                                         comp_cnt,   /* comp_cnt */
                                         this,       /* user data */
                                         &icecb,     /* callback */
                                         &icest_)) { /* instance ptr */
    SIPPlugin::this_->message("ERROR:Error creating ICE");
    return;
  }
  std::unique_lock<std::mutex> lock(cand_ready_mtx_);
  while (!cand_ready_)
    if (cand_ready_cv_.wait_for(lock, std::chrono::seconds(3)) == std::cv_status::timeout) return;
  if (PJ_SUCCESS != pj_ice_strans_init_ice(icest_, role, nullptr, nullptr)) {
    SIPPlugin::this_->message("ERROR:Error initializing ICE");
    return;
  }
  // done
  is_valid_ = true;
}

PJICEStreamTrans::~PJICEStreamTrans() {
  if (nullptr != icest_) {
    if (pj_ice_strans_has_sess(icest_)) {
      if (PJ_SUCCESS != pj_ice_strans_stop_ice(icest_))
        SIPPlugin::this_->warning("issue stoping ICE session");
    }
    pj_ice_strans_destroy(icest_);
  }
}

/*
 * This is the callback that is registered to the ICE stream transport to
 * receive notification about incoming data. By "data" it means application
 * data such as RTP/RTCP, and not packets that belong to ICE signaling (such
 * as STUN connectivity checks or TURN signaling).
 */
void PJICEStreamTrans::cb_on_rx_data(pj_ice_strans* ice_st,
                                     unsigned comp_id,
                                     void* pkt,
                                     pj_size_t size,
                                     const pj_sockaddr_t* /*src_addr*/,
                                     unsigned /*src_addr_len*/) {
  PJICEStreamTrans* context = static_cast<PJICEStreamTrans*>(pj_ice_strans_get_user_data(ice_st));
  if (nullptr != context->data_cbs_[comp_id - 1]) context->data_cbs_[comp_id - 1](pkt, size);
}

/*
 * This is the callback that is registered to the ICE stream transport to
 * receive notification about ICE state progression.
 */
void PJICEStreamTrans::cb_on_ice_complete(pj_ice_strans* ice_st,
                                          pj_ice_strans_op op,
                                          pj_status_t status) {
  const char* opname = (op == PJ_ICE_STRANS_OP_INIT
                            ? "initialization"
                            : (op == PJ_ICE_STRANS_OP_NEGOTIATION ? "negotiation" : "unknown_op"));

  SIPPlugin::this_->debug("ICE state %",
                          std::string(pj_ice_strans_state_name(pj_ice_strans_get_state(ice_st))));

  PJICEStreamTrans* context = static_cast<PJICEStreamTrans*>(pj_ice_strans_get_user_data(ice_st));
  if (op == PJ_ICE_STRANS_OP_INIT) {
    std::unique_lock<std::mutex> lock(context->cand_ready_mtx_);
    if (status == PJ_SUCCESS) context->cand_ready_ = true;
    context->cand_ready_cv_.notify_one();
  }

  if (status == PJ_SUCCESS) {
    SIPPlugin::this_->debug("ICE % successful", std::string(opname));
  } else {
    char errmsg[PJ_ERR_MSG_SIZE];
    pj_strerror(status, errmsg, sizeof(errmsg));
    SIPPlugin::this_->warning("ICE % failed: %", std::string(opname), std::string(errmsg));
    SIPPlugin::this_->message("ERROR:ICE % failed: %", std::string(opname), std::string(errmsg));
    // pj_ice_strans_destroy(ice_st);
    // icest_ = NULL;
  }
}

std::pair<pj_str_t, pj_str_t> PJICEStreamTrans::get_ufrag_and_passwd() {
  pj_str_t local_ufrag, local_pwd;
  /* Get ufrag and pwd from current session */
  pj_ice_strans_get_ufrag_pwd(icest_, &local_ufrag, &local_pwd, nullptr, nullptr);
  // return std::string("a=ice-ufrag:")
  //     + std::string(local_ufrag.ptr, 0, local_ufrag.slen)
  //     + "\na=ice-pwd:"
  //     + std::string(local_pwd.ptr, 0, local_pwd.slen)
  //     + "\n";
  return std::make_pair(local_ufrag, local_pwd);
}

std::vector<std::vector<std::string>> PJICEStreamTrans::get_components() {
  std::vector<std::vector<std::string>> res;
  /* Write each component */
  for (unsigned comp = 0; comp < comp_cnt_; ++comp) {
    res.emplace_back();
    unsigned j, cand_cnt;
    pj_ice_sess_cand cand[PJ_ICE_ST_MAX_CAND];
    char ipaddr[PJ_INET6_ADDRSTRLEN];

    /* Get default candidate for the component */
    if (PJ_SUCCESS != pj_ice_strans_get_def_cand(icest_, comp + 1, &cand[0])) {
      SIPPlugin::this_->warning("issue with pj_ice_strans_get_def_cand");
      return res;
    }

    /* Enumerate all candidates for this component */
    cand_cnt = PJ_ARRAY_SIZE(cand);
    if (PJ_SUCCESS != pj_ice_strans_enum_cands(icest_, comp + 1, &cand_cnt, cand)) {
      SIPPlugin::this_->warning("issue with pj_ice_strans_enum_cands");
      return res;
    }
    /* And encode the candidates as SDP */
    for (j = 0; j < cand_cnt; ++j) {
      res.back().emplace_back(
          // std::string("a=candidate:") +
          std::string(cand[j].foundation.ptr, 0, cand[j].foundation.slen) + " " +
          std::to_string(cand[j].comp_id) + " UDP " + std::to_string(cand[j].prio) + " " +
          std::string(pj_sockaddr_print(&cand[j].addr, ipaddr, sizeof(ipaddr), 0)) + " " +
          std::to_string(pj_sockaddr_get_port(&cand[j].addr)) + " typ " +
          pj_ice_get_cand_type_name(cand[j].type));
    }
  }
  return res;
}

std::vector<pj_uint16_t> PJICEStreamTrans::get_first_candidate_ports() {
  std::vector<pj_uint16_t> res;
  /* Write each component */
  for (unsigned comp = 0; comp < comp_cnt_; ++comp) {
    pj_ice_sess_cand cand[PJ_ICE_ST_MAX_CAND];
    /* Get default candidate for the component */
    if (PJ_SUCCESS != pj_ice_strans_get_def_cand(icest_, comp + 1, &cand[0])) {
      SIPPlugin::this_->warning("issue with pj_ice_strans_get_def_cand");
      return res;
    }
    res.emplace_back(pj_sockaddr_get_port(&cand[0].addr));
  }
  return res;
}

std::string PJICEStreamTrans::get_first_candidate_host() {
  pj_ice_sess_cand cand[PJ_ICE_ST_MAX_CAND];
  if (PJ_SUCCESS != pj_ice_strans_get_def_cand(icest_, 1, &cand[0])) {
    SIPPlugin::this_->warning("issue with %", std::string(__FUNCTION__));
    return std::string();
  }
  char addr_buf[128];
  pj_sockaddr_print(&cand[0].addr, addr_buf, 128, 0);
  return std::string(addr_buf);
}

bool PJICEStreamTrans::start_nego(const pj_str_t* rem_ufrag,
                                  const pj_str_t* rem_passwd,
                                  unsigned rcand_cnt,
                                  const pj_ice_sess_cand rcand[]) {
  pj_status_t status = pj_ice_strans_start_ice(icest_, rem_ufrag, rem_passwd, rcand_cnt, rcand);
  // char buffer[100000];
  // char *p = buffer;
  // for (unsigned j=0; j<rcand_cnt; ++j) {
  //   int printed = print_cand(p, 100000 - (unsigned)(p-buffer), &rcand[j]);
  //   if (printed < 0)
  //     return -PJ_ETOOSMALL;
  //   p += printed;
  // }
  // *p = '\0';
  if (PJ_SUCCESS != status) {
    char errmsg[PJ_ERR_MSG_SIZE];
    pj_strerror(status, errmsg, sizeof(errmsg));
    SIPPlugin::this_->warning("ICE negociation issue: %", std::string(errmsg));
    return false;
  }
  return true;
}

bool PJICEStreamTrans::sendto(unsigned comp_id,
                              const void* data,
                              pj_size_t size,
                              const pj_sockaddr_t* dst_addr,
                              int dst_addr_len) {
  if (pj_ice_strans_sess_is_complete(icest_))
    return PJ_SUCCESS == pj_ice_strans_sendto(icest_, comp_id, data, size, dst_addr, dst_addr_len);
  return true;  // not sent, but this is an expected behaviour
}

bool PJICEStreamTrans::set_data_cb(unsigned comp_id, on_data_cb_t cb) {
  if (comp_id > data_cbs_.size()) return false;
  data_cbs_[comp_id - 1] = cb;
  return true;
}

}  // namespace switcher
