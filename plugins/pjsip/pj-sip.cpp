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

#include "./pj-sip.hpp"
#include <glib.h>
#include "./pj-presence.hpp"
#include "./pj-sip-plugin.hpp"
#include "switcher/net-utils.hpp"

namespace switcher {
// according to pjsip documentation:
// Application should only instantiate
// one SIP endpoint instance for every process.
// Accordingly, SIPPlugin is a singleton
PJSIP* PJSIP::this_ = nullptr;  // static pointer to the instance
// std::atomic<bool> does not have fetch_* speciliazed members,
// using unsigned short instead
std::atomic<unsigned short> PJSIP::sip_endpt_used_(0);

PJSIP::PJSIP(std::function<bool()> init_fun, std::function<void()> destruct_fun)
    : cp_(), destruct_fun_(destruct_fun) {
  if (1 == sip_endpt_used_.fetch_or(1)) {
    SIPPlugin::this_->warning("an other sip quiddity is instancied, cannot init");
    return;
  }
  i_m_the_one_ = true;
  this_ = this;

  pj_status_t status = pj_init();
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->warning("cannot init pjsip library");
    return;
  }
  pj_log_set_level(log_level_);
  // Register the thread, after pj_init() is called
  pj_thread_register("switcher-pjsip-singleton", thread_handler_desc_, &pj_thread_ref_);
  status = pjsua_create();
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->warning("Error in pjsua_create()");
    return;
  }
  /* Init pjsua */
  {
    pjsua_config cfg;
    pjsua_logging_config log_cfg;
    pjsua_config_default(&cfg);
    cfg.cb.on_buddy_state = &PJPresence::on_buddy_state;
    cfg.cb.on_reg_state2 = &PJPresence::on_registration_state;
    // cfg.cb.on_create_media_transport = &on_create_media_transport;---
    // see pjsip-apps/src/pjsua/pjsua_app.c
    // cfg.cb.on_call_state = &on_call_state;
    // cfg.cb.on_call_media_state = &on_call_media_state;---
    // cfg.cb.on_incoming_call = &on_incoming_call;---
    // cfg.cb.on_call_tsx_state = &on_call_tsx_state;
    // cfg.cb.on_dtmf_digit = &call_on_dtmf_callback;
    // cfg.cb.on_call_redirected = &call_on_redirected;
    cfg.cb.on_reg_state = &PJPresence::on_reg_state;
    cfg.cb.on_incoming_subscribe = &PJPresence::on_incoming_subscribe;
    cfg.cb.on_buddy_evsub_state = &PJPresence::on_buddy_evsub_state;
    // cfg.cb.on_pager = &on_pager;
    // cfg.cb.on_typing = &on_typing;
    // cfg.cb.on_call_transfer_status = &on_call_transfer_status;
    // cfg.cb.on_call_replaced = &on_call_replaced;
    // cfg.cb.on_nat_detect = &on_nat_detect;
    // cfg.cb.on_mwi_info = &on_mwi_info;
    // cfg.cb.on_transport_state = &on_transport_state;
    // cfg.cb.on_ice_transport_error = &on_ice_transport_error;
    // cfg.cb.on_snd_dev_operation = &on_snd_dev_operation;
    // cfg.cb.on_call_media_event = &on_call_media_event;
    pjsua_logging_config_default(&log_cfg);
    log_cfg.console_level = log_level_;
    status = pjsua_init(&cfg, &log_cfg, nullptr);
    if (status != PJ_SUCCESS) {
      SIPPlugin::this_->warning("Error in pjsua_init()");
      return;
    }
    sip_endpt_ = pjsua_get_pjsip_endpt();
  }
  /* Must create a pool factory before we can allocate any memory. */
  pj_caching_pool_init(&cp_, &pj_pool_factory_default_policy, 0);
  /* Create application pool for misc. */
  pool_ = pj_pool_create(&cp_.factory, "switcher_sip", 8000, 8000, nullptr);
  // pj_dns_resolver *resv = pjsip_endpt_get_resolver(sip_endpt_);
  // if (nullptr == resv) printf ("NULL RESOLVER -------------------------\n");
  create_resolver(SIPPlugin::this_->dns_address_);
  if (!init_fun()) {
    SIPPlugin::this_->warning("pj-sip custom initialization failed");
    return;
  }
  sip_work_ = true;
  sip_worker_ = std::thread(&PJSIP::sip_worker_thread, this);
  /* Initialization is done, now start pjsua */
  status = pjsua_start();
  if (status != PJ_SUCCESS) {
    SIPPlugin::this_->warning("Error starting pjsua");
    return;
  }
  // done
  is_valid_ = true;
}

PJSIP::~PJSIP() {
  if (!i_m_the_one_) {
    return;
  }
  if (sip_worker_.joinable()) {
    sip_work_ = false;
    sip_worker_.join();
  }
  this_ = nullptr;
  i_m_the_one_ = false;
  sip_endpt_used_ = 0;
}

void PJSIP::sip_worker_thread() {
  // Register the thread, after pj_init() is called
  pj_thread_register("sip_worker_thread", worker_handler_desc_, &worker_thread_ref_);
  while (sip_work_) {
    pj_time_val timeout = {0, 10};
    pjsip_endpt_handle_events(sip_endpt_, &timeout);
  }

  /* Shutting down... */
  if (destruct_fun_) destruct_fun_();
  if (nullptr != pool_) {
    pj_pool_release(pool_);
    pool_ = nullptr;
    pj_caching_pool_destroy(&cp_);
  }
  pjsua_destroy();
  pj_shutdown();
}

bool PJSIP::create_resolver(std::string dns_address) {
  pj_dns_resolver* resv;
  if (PJ_SUCCESS != pjsip_endpt_create_resolver(sip_endpt_, &resv)) {
    SIPPlugin::this_->warning("pjsip failed to create a resolver.");
    return false;
  }
  pj_str_t nameserver = pj_str(const_cast<char*>(dns_address.c_str()));
  pj_uint16_t port = 53;

  if (PJ_SUCCESS != pj_dns_resolver_set_ns(resv, 1, &nameserver, &port)) {
    SIPPlugin::this_->warning("pjsip failed to set name resolution server.");
    return false;
  }

  if (PJ_SUCCESS != pjsip_endpt_set_resolver(sip_endpt_, resv)) {
    SIPPlugin::this_->warning("pjsip failed to set the resolver.");
    return false;
  }

  return true;
}

}  // namespace switcher
