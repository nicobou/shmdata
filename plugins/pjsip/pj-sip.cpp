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

#include "switcher/net-utils.hpp"
#include "./pj-sip.hpp"

namespace switcher {
SWITCHER_DECLARE_PLUGIN(PJSIP);
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    PJSIP,
    "sip",
    "SIP (Session Initiation Protocol)",
    "network",
    "writer",
    "Manages user sessions",
    "LGPL",
    "Nicolas Bouillot");

// according to pjsip documentation:
// Application should only instantiate
// one SIP endpoint instance for every process.
// Accordingly, PJSIP is a singleton
PJSIP *PJSIP::this_ = nullptr;  // static pointer to the instance
// std::atomic<bool> does not have fetch_* speciliazed members,
// using unsigned short instead
std::atomic<unsigned short> PJSIP::sip_endpt_used_(0);

PJSIP::PJSIP(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper>()),
    cp_() {
}

PJSIP::~PJSIP() {
  if (!i_m_the_one_) {
    return;
  }
  if (pj_sip_inited_) {
    run_command_sync(std::bind(&PJSIP::exit_cmd, this));
    if (sip_thread_.joinable())
      sip_thread_.join();
    if (sip_worker_.joinable()) {
      sip_work_ = false;
      sip_worker_.join();
    }
  }
  this_ = nullptr;
  sip_endpt_used_ = 0;
}

bool PJSIP::init() {
  if (1 == sip_endpt_used_.fetch_or(1)) {
    g_warning("an other sip quiddity is instancied, cannot init");
    return false;
  }
  i_m_the_one_ = true;
  this_ = this;
  sip_port_spec_ =
      custom_props_->make_int_property("port",
                                       "SIP port used when registering",
                                       0,
                                       65535,
                                       sip_port_,
                                       (GParamFlags) G_PARAM_READWRITE,
                                       set_port,
                                       get_port, this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            sip_port_spec_,
                            "port", "SIP port used when registering");
  std::unique_lock<std::mutex> lock(pj_init_mutex_);
  sip_thread_ = std::thread(&PJSIP::sip_handling_thread, this);
  pj_init_cond_.wait(lock);
  if (!pj_sip_inited_)
    return false;
  return true;
}

void PJSIP::run_command_sync(std::function<void()> command) {
  {
    std::unique_lock<std::mutex> lock(work_mutex_);
    command_ = command;
  }
  std::unique_lock<std::mutex> lock_done(done_mutex_);
  work_cond_.notify_one();
  done_cond_.wait(lock_done);
}

bool PJSIP::pj_sip_init() {
  pj_status_t status = pj_init();
  if (status != PJ_SUCCESS) {
    g_warning("cannot init pjsip library");
    return false;
  }
  pj_log_set_level(6);
  // Register the thread, after pj_init() is called
  pj_thread_register(Quiddity::get_name().c_str(),
                     thread_handler_desc_, &pj_thread_ref_);
  status = pjsua_create();
  if (status != PJ_SUCCESS) {
    g_warning("Error in pjsua_create()");
    return false;
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
    log_cfg.console_level = 0;
    status = pjsua_init(&cfg, &log_cfg, nullptr);
    if (status != PJ_SUCCESS) {
      g_warning("Error in pjsua_init()");
      return false;
    }
    sip_endpt_ = pjsua_get_pjsip_endpt();
  }
  /* Must create a pool factory before we can allocate any memory. */
  pj_caching_pool_init(&cp_, &pj_pool_factory_default_policy, 0);
  /* Create application pool for misc. */
  pool_ = pj_pool_create(&cp_.factory, "switcher_sip", 1000, 1000, nullptr);
  start_tcp_transport();
  // pj_dns_resolver *resv = pjsip_endpt_get_resolver(sip_endpt_);
  // if (nullptr == resv) printf ("NULL RESOLVER -------------------------\n");
  pj_dns_resolver *resv;
  pjsip_endpt_create_resolver(sip_endpt_, &resv);
  pj_str_t nameserver = pj_str("8.8.8.8");;
  pj_uint16_t port = 53;
  pj_dns_resolver_set_ns(resv, 1, &nameserver, &port);
  pjsip_endpt_set_resolver(sip_endpt_, resv);
  sip_calls_ = new PJCall(this);
  sip_presence_ = new PJPresence(this);
  sip_work_ = true;
  sip_worker_ = std::thread(&PJSIP::sip_worker_thread, this);
  /* Initialization is done, now start pjsua */
  status = pjsua_start();
  if (status != PJ_SUCCESS) {
    g_warning("Error starting pjsua");
    return false;
  }
  return true;
}

void PJSIP::sip_worker_thread() {
  // Register the thread, after pj_init() is called
  pj_thread_register("sip_worker_thread",
                     worker_handler_desc_, &worker_thread_ref_);
  while (sip_work_) {
    pj_time_val timeout = { 0, 10 };
    pjsip_endpt_handle_events(sip_endpt_, &timeout);
  }

  /* Shutting down... */
  if (nullptr != sip_calls_) {
    delete(sip_calls_);
    sip_calls_ = nullptr;
  }
  if (nullptr == sip_presence_) {
    delete(sip_presence_);
    sip_presence_ = nullptr;
  }
  if (nullptr != pool_) {
    pj_pool_release(pool_);
    pool_ = nullptr;
    pj_caching_pool_destroy(&cp_);
  }
  pjsua_destroy();
  pj_shutdown();
}

void PJSIP::sip_handling_thread() {
  {                           // init pj sip
    std::unique_lock<std::mutex> lock(pj_init_mutex_);
    pj_sip_inited_ = pj_sip_init();
    pj_init_cond_.notify_all();
  }
  while (continue_) {
    std::unique_lock<std::mutex> lock_work(work_mutex_);
    work_cond_.wait(lock_work);
    // do_something
    {
      std::unique_lock<std::mutex> lock_done(done_mutex_);
      command_();
    }
    done_cond_.notify_one();
  }
}

void PJSIP::exit_cmd() {
  continue_ = false;
}

void PJSIP::start_tcp_transport() {
  if (-1 != transport_id_)
    pjsua_transport_close(transport_id_, PJ_FALSE);
  if (NetUtils::is_used(sip_port_)) {
    g_warning("SIP port cannot be binded (%u)", sip_port_);
    return;
  }
  pjsua_transport_config cfg;
  pjsua_transport_config_default(&cfg);
  cfg.port = sip_port_;
  pj_status_t status =
      pjsua_transport_create(PJSIP_TRANSPORT_UDP, &cfg, &transport_id_);
  if (status != PJ_SUCCESS) {
    g_warning("Error creating transport");
    return;
  }
}

void PJSIP::set_port(const gint value, void *user_data) {
  PJSIP *context = static_cast<PJSIP *>(user_data);
  if (value == (gint)context->sip_port_)
    return;
  context->sip_port_ = value;
  context->run_command_sync(std::bind(&PJSIP::start_tcp_transport,
                                      context));
  GObjectWrapper::notify_property_changed(context->gobject_->get_gobject(),
                                          context->sip_port_spec_);
}

gint PJSIP::get_port(void *user_data) {
  PJSIP *context = static_cast<PJSIP *>(user_data);
  return context->sip_port_;
}

}  // namespace switcher
