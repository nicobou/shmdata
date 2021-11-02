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

#include "./pj-stun-turn.hpp"
#include <pjnath.h>
#include "./pj-sip-plugin.hpp"
#include "switcher/infotree/json-serializer.hpp"
#include "switcher/utils/scope-exit.hpp"
#include "switcher/utils/string-utils.hpp"

namespace switcher {
namespace quiddities {
PJStunTurn::PJStunTurn() {
  if (PJ_SUCCESS != pjnath_init()) {
    SIPPlugin::this_->warning("cannot init pjnath");
    return;
  }
  pj_ice_strans_cfg_default(&ice_cfg_);
  pj_timer_heap_create(PJSIP::this_->pool_, 100, &ice_cfg_.stun_cfg.timer_heap);
  pj_ioqueue_create(PJSIP::this_->pool_, 512, &ice_cfg_.stun_cfg.ioqueue);
  ice_cfg_.stun_cfg.pf = PJSIP::this_->pool_->factory;
  if (PJ_SUCCESS !=
      pj_thread_create(PJSIP::this_->pool_, "switcherSIP", &worker_thread, this, 0, 0, &thread_)) {
    SIPPlugin::this_->warning("STUN TURN thread creation failed");
    return;
  }
  ice_cfg_.af = pj_AF_INET();
  ice_cfg_.stun.cfg.max_pkt_size = 8192;
  ice_cfg_.turn.cfg.max_pkt_size = 8192;
  ice_cfg_.opt.aggressive = PJ_FALSE;
  ice_cfg_.stun_cfg.rto_msec = 500;

  // set stun/turn config
  using set_stun_turn_t = std::function<bool(std::string, std::string, std::string, std::string)>;
  SIPPlugin::this_->mmanage<MPtr(&method::MBag::make_method<set_stun_turn_t>)>(
      "set_stun_turn",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Set STUN/TURN parameters",
                   "description" : "Set STUN/TURN configuration",
                   "arguments" : [
                     {
                        "long name" : "STUN server address",
                        "description" : "string"
                     }, {
                        "long name" : "TURN server address",
                        "description" : "string"
                     }, {
                        "long name" : "TURN user name",
                        "description" : "string"
                     }, {
                        "long name" : "TURN user password",
                        "description" : "string"
                     }
                   ]
                  }
              )"),
      [this](const std::string& stun,
             const std::string& turn,
             const std::string& login,
             const std::string& pass) { return set_stun_turn(stun, turn, login, pass); });

  SIPPlugin::this_->pmanage<MPtr(&property::PBag::make_bool)>(
      "lower-case-turn-account",
      [this](const bool val) {
        lower_case_turn_account_ = val;
        return true;
      },
      [this]() { return lower_case_turn_account_; },
      "Lower Case TURN Account",
      "Lower Case TURN Account",
      lower_case_turn_account_);
}

PJStunTurn::~PJStunTurn() {
  worker_quit_ = true;
  if (nullptr != thread_) {
    pj_thread_join(thread_);
    pj_thread_destroy(thread_);
  }
  if (ice_cfg_.stun_cfg.ioqueue) pj_ioqueue_destroy(ice_cfg_.stun_cfg.ioqueue);

  if (ice_cfg_.stun_cfg.timer_heap) pj_timer_heap_destroy(ice_cfg_.stun_cfg.timer_heap);
}

int PJStunTurn::worker_thread(void* data) {
  auto context = static_cast<PJStunTurn*>(data);
  while (!context->worker_quit_) {
    context->handle_events(500, NULL);
  }
  return 0;
}

pj_status_t PJStunTurn::handle_events(unsigned max_msec, unsigned* p_count) {
  enum { MAX_NET_EVENTS = 1 };
  pj_time_val max_timeout = {0, 0};
  pj_time_val timeout = {0, 0};
  unsigned count = 0, net_event_count = 0;
  int c;
  max_timeout.msec = max_msec;
  /* Poll the timer to run it and also to retrieve the earliest entry. */
  timeout.sec = timeout.msec = 0;
  c = pj_timer_heap_poll(ice_cfg_.stun_cfg.timer_heap, &timeout);
  if (c > 0) count += c;
  /* timer_heap_poll should never ever returns negative value, or otherwise
   * ioqueue_poll() will block forever!
   */
  pj_assert(timeout.sec >= 0 && timeout.msec >= 0);
  if (timeout.msec >= 1000) timeout.msec = 999;
  /* compare the value with the timeout to wait from timer, and use the
   * minimum value.
   */
  if (PJ_TIME_VAL_GT(timeout, max_timeout)) timeout = max_timeout;
  /* Poll ioqueue.
   * Repeat polling the ioqueue while we have immediate events, because
   * timer heap may process more than one events, so if we only process
   * one network events at a time (such as when IOCP backend is used),
   * the ioqueue may have trouble keeping up with the request rate.
   *
   * For example, for each send() request, one network event will be
   *   reported by ioqueue for the send() completion. If we don't poll
   *   the ioqueue often enough, the send() completion will not be
   *   reported in timely manner.
   */
  do {
    c = pj_ioqueue_poll(ice_cfg_.stun_cfg.ioqueue, &timeout);
    if (c < 0) {
      pj_status_t err = pj_get_netos_error();
      pj_thread_sleep(PJ_TIME_VAL_MSEC(timeout));
      if (p_count) *p_count = count;
      return err;
    } else if (c == 0) {
      break;
    } else {
      net_event_count += c;
      timeout.sec = timeout.msec = 0;
    }
  } while (c > 0 && net_event_count < MAX_NET_EVENTS);
  count += net_event_count;
  if (p_count) *p_count = count;
  return PJ_SUCCESS;
}

bool PJStunTurn::set_stun_turn(const std::string& stun,
                               const std::string& turn,
                               const std::string& turn_user,
                               const std::string& turn_pass) {
  // saving old config for reverting if error with the new one to be tested:
  bool success = false;
  pj_ice_strans_cfg old_ice_cfg = ice_cfg_;
  On_scope_exit {
    if (!success) ice_cfg_ = old_ice_cfg;
  };

  /* Configure STUN/srflx candidate resolution */
  if (!stun.empty()) {
    std::string tmp_stun = std::string(stun);
    auto pos = tmp_stun.find(':');
    /* Command line option may contain port number */
    if (std::string::npos != pos) {
      stun_srv_ = std::string(stun, 0, pos);
      ice_cfg_.stun.server.ptr = (char*)stun_srv_.c_str();
      ice_cfg_.stun.server.slen = stun_srv_.size();
      ice_cfg_.stun.port = (pj_uint16_t)atoi(tmp_stun.c_str() + pos + 1);
    } else {
      stun_srv_ = tmp_stun;
      ice_cfg_.stun.server.ptr = (char*)stun_srv_.c_str();
      ice_cfg_.stun.server.slen = stun_srv_.size();
      ice_cfg_.stun.port = PJ_STUN_PORT;
    }
    /* For this demo app, configure longer STUN keep-alive time
       * so that it does't clutter the screen output.
       */
    ice_cfg_.stun.cfg.ka_interval = 300;  // seconds
  }

  /* Configure TURN candidate */
  if (!turn.empty()) {
    std::string tmp_turn = std::string(turn);
    auto pos = tmp_turn.find(':');
    /* Command line option may contain port number */
    if (std::string::npos != pos) {
      turn_srv_ = std::string(turn, 0, pos);
      ice_cfg_.turn.server.ptr = (char*)turn_srv_.c_str();
      ice_cfg_.turn.server.slen = turn_srv_.size();
      ice_cfg_.turn.port = (pj_uint16_t)atoi(tmp_turn.c_str() + pos + 1);
    } else {
      turn_srv_ = tmp_turn;
      ice_cfg_.turn.server.ptr = (char*)turn_srv_.c_str();
      ice_cfg_.turn.server.slen = turn_srv_.size();
      ice_cfg_.turn.port = PJ_STUN_PORT;
    }

    /* TURN credential */
    ice_cfg_.turn.auth_cred.type = PJ_STUN_AUTH_CRED_STATIC;
    if (!turn_user.empty()) {
      turn_user_ = std::string(turn_user);
      if (lower_case_turn_account_) stringutils::tolower(turn_user_);
      ice_cfg_.turn.auth_cred.data.static_cred.username.ptr = (char*)turn_user_.c_str();
      ice_cfg_.turn.auth_cred.data.static_cred.username.slen = turn_user_.size();
    }
    ice_cfg_.turn.auth_cred.data.static_cred.data_type = PJ_STUN_PASSWD_PLAIN;
    if (!turn_pass.empty()) {
      turn_pass_ = std::string(turn_pass);
      ice_cfg_.turn.auth_cred.data.static_cred.data.ptr = (char*)turn_pass_.c_str();
      ice_cfg_.turn.auth_cred.data.static_cred.data.slen = turn_pass_.size();
    }

    // Connection type to TURN server
    ice_cfg_.turn.conn_type = PJ_TURN_TP_UDP;

    // configure longer keep-alive time
    // so that it does't clutter the screen output.

    ice_cfg_.turn.alloc_param.ka_interval = 300;
  }

  if (SIPPlugin::this_->pjsip_->run<bool>([&]() -> bool {
        return static_cast<bool>(PJICEStreamTrans(ice_cfg_, 1, PJ_ICE_SESS_ROLE_CONTROLLING));
      })) {
    success = true;  // validate current config
    stun_turn_valid_ = true;
    return true;
  } else {
    return false;
  }
}

std::unique_ptr<PJICEStreamTrans> PJStunTurn::get_ice_transport(unsigned comp_cnt,
                                                                pj_ice_sess_role role) {
  // if (!stun_turn_valid_)
  //   return std::unique_ptr<PJICEStreamTrans>(nullptr);
  auto res = std::make_unique<PJICEStreamTrans>(ice_cfg_, comp_cnt, role);
  if (!static_cast<bool>(*res.get())) res.reset(nullptr);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return res;
}

}  // namespace quiddities
}  // namespace switcher
