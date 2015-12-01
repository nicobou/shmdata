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
#include "switcher/std2.hpp"
#include "./pj-sip-plugin.hpp"

namespace switcher {
SWITCHER_DECLARE_PLUGIN(SIPPlugin);
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    SIPPlugin,
    "sip",
    "SIP (Session Initiation Protocol)",
    "network",
    "writer",
    "Manages user sessions",
    "LGPL",
    "Nicolas Bouillot");

SIPPlugin *SIPPlugin::this_ = nullptr;
std::atomic<unsigned short> SIPPlugin::sip_plugin_used_(0);


SIPPlugin::SIPPlugin(const std::string &) {
}

SIPPlugin::~SIPPlugin() {
  if (!i_m_the_one_)
    return;
  pjsip_.reset(nullptr);
  this_ = nullptr;
  i_m_the_one_ = false;
  sip_plugin_used_ = 0;
}

bool SIPPlugin::init() {
  if (1 == sip_plugin_used_.fetch_or(1)) {
    g_warning("an other sip quiddity is instancied, cannot init");
    return false;
  }
  i_m_the_one_ = true;
  this_ = this;

  pmanage<MPtr(&PContainer::make_unsigned_int)>(
      "port",
      [this](const unsigned int &val){
        if (val == sip_port_)
          return true;
        sip_port_ = val;
        return pjsip_->run<bool>([this](){return start_sip_transport();});
      },
      [this](){return sip_port_;},
      "SIP Port",
      "SIP port used when registering",
      sip_port_,
      0u,
      65535u);

  pjsip_ = std2::make_unique<ThreadedWrapper<PJSIP>>(
      // init
      [&](){
        start_sip_transport();
        sip_calls_ = new PJCall();
        sip_presence_ = new PJPresence();
        return true;
      },
      // destruct
      [&](){
        if (nullptr != sip_calls_) {
          delete(sip_calls_);
          sip_calls_ = nullptr;
        }
        if (nullptr == sip_presence_) {
          delete(sip_presence_);
          sip_presence_ = nullptr;
        }
      });
  return pjsip_->invoke<MPtr(&PJSIP::safe_bool_idiom)>();
}

bool SIPPlugin::start_sip_transport() {
  if (-1 != transport_id_)
    pjsua_transport_close(transport_id_, PJ_FALSE);
  if (NetUtils::is_used(sip_port_)) {
    g_warning("SIP port cannot be binded (%u)", sip_port_);
    return false;
  }
  pjsua_transport_config cfg;
  pjsua_transport_config_default(&cfg);
  cfg.port = sip_port_;
  pj_status_t status =
      pjsua_transport_create(PJSIP_TRANSPORT_UDP, &cfg, &transport_id_);
  if (status != PJ_SUCCESS) {
    g_warning("Error creating transport");
    return false;
  }
  return true;
}

}  // namespace switcher
