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

#include "./pj-sip-plugin.hpp"
#include "switcher/net-utils.hpp"
#include "switcher/scope-exit.hpp"

namespace switcher {
SWITCHER_DECLARE_PLUGIN(SIPPlugin);
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(SIPPlugin,
                                     "sip",
                                     "SIP (Session Initiation Protocol)",
                                     "network",
                                     "occasional-writer",
                                     "Manages user sessions",
                                     "LGPL",
                                     "Nicolas Bouillot");

SIPPlugin* SIPPlugin::this_ = nullptr;

std::atomic<unsigned short> SIPPlugin::sip_plugin_used_(0);

SIPPlugin::SIPPlugin(const std::string&)
    : port_id_(pmanage<MPtr(&PContainer::make_string)>(
          "port",
          [this](const std::string& valstr) {
            if (valstr.empty()) return false;
            unsigned int val;
            if (!isdigit(valstr[0])) return false;
            val = atoi(valstr.c_str());
            if (sip_port_ == val && -1 != transport_id_) return true;
            if (val > 65535) return false;
            sip_port_ = val;
            return pjsip_->run<bool>([this]() { return start_sip_transport(); });
          },
          [this]() { return std::to_string(sip_port_); },
          "SIP Port",
          "SIP port used when registering",
          std::to_string(sip_port_))),
      dns_address_(NetUtils::get_system_dns()),
      dns_address_id_(pmanage<MPtr(&PContainer::make_string)>(
          "dns_addr",
          [this](const std::string& val) {
            if (val.empty()) return false;
            if (!NetUtils::is_valid_IP(val)) {
              g_message(
                  "ERROR:Not a valid IP address, expected x.y.z.a with x, y, z, a in [0:255].");
              return false;
            }

            // Try setting the new DNS address, if it doesn't work try the old one.
            auto old_dns_address = dns_address_;
            dns_address_ = val;
            if (!pjsip_->invoke<MPtr(&PJSIP::create_resolver)>(dns_address_)) {
              dns_address_ = old_dns_address;
              g_message("ERROR:Could not set the DNS address (PJSIP), setting the old one %s.",
                        dns_address_.c_str());
              pjsip_->invoke<MPtr(&PJSIP::create_resolver)>(dns_address_);
              return false;
            }

            return true;
          },
          [this]() { return dns_address_; },
          "DNS address",
          "IP address used for DNS",
          dns_address_)),
      decompress_streams_id_(
          pmanage<MPtr(&PContainer::make_bool)>("decompress",
                                                [this](const bool& val) {
                                                  decompress_streams_ = val;
                                                  return true;
                                                },
                                                [this]() { return decompress_streams_; },
                                                "Decompress",
                                                "Decompress received streams",
                                                decompress_streams_)) {}

SIPPlugin::~SIPPlugin() {
  if (!i_m_the_one_) return;

  auto manager = manager_impl_.lock();
  if (manager) manager->unregister_removal_cb(quiddity_removal_cb_id_);

  sip_calls_->finalize_calls();
  sip_calls_.reset(nullptr);

  pjsip_->run([this]() {
    stun_turn_.reset(nullptr);
    sip_presence_.reset(nullptr);
  });

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

  pjsip_ = std::make_unique<ThreadedWrapper<PJSIP>>(
      // init
      [&]() {
        start_sip_transport();
        sip_calls_ = std::make_unique<PJCall>();
        white_list_ = std::make_unique<PJWhiteList>();
        sip_presence_ = std::make_unique<PJPresence>();
        stun_turn_ = std::make_unique<PJStunTurn>();
        return true;
      },
      // destruct
      [&]() {
        sip_calls_.reset(nullptr);
        sip_presence_.reset(nullptr);
        stun_turn_.reset(nullptr);
        white_list_.reset(nullptr);
      });

  if (!pjsip_->invoke<MPtr(&PJSIP::safe_bool_idiom)>()) return false;
  apply_configuration();
  return true;
}

void SIPPlugin::apply_configuration() {
  // trying to set port if configuration found
  if (config<MPtr(&InfoTree::branch_has_data)>("port")) {
    g_debug("SIP is trying to set port from configuration");
    auto port = config<MPtr(&InfoTree::branch_get_value)>("port");
    if (pmanage<MPtr(&PContainer::set<std::string>)>(port_id_, port.copy_as<std::string>())) {
      g_debug("sip has set port from configuration");
    } else {
      g_warning("sip failed setting port from configuration");
    }
  }

  // trying to set stun/turn from configuration
  std::string stun = config<MPtr(&InfoTree::branch_get_value)>("stun");
  std::string turn = config<MPtr(&InfoTree::branch_get_value)>("turn");
  std::string turn_user = config<MPtr(&InfoTree::branch_get_value)>("turn_user");
  std::string turn_pass = config<MPtr(&InfoTree::branch_get_value)>("turn_pass");
  if (!stun.empty()) {
    g_debug("SIP is trying to set STUN/TURN from configuration");
    if (PJStunTurn::set_stun_turn(
            stun.c_str(), turn.c_str(), turn_user.c_str(), turn_pass.c_str(), stun_turn_.get())) {
      g_debug("sip has set STUN/TURN from configuration");
    } else {
      g_warning("sip failed setting STUN/TURN from configuration");
    }
  }

  auto manager = manager_impl_.lock();
  if (!manager) return;
  quiddity_removal_cb_id_ = manager->register_removal_cb([this](const std::string& quiddity_name) {
    std::lock_guard<std::mutex> lock(exposed_quiddities_mutex_);
    auto it = std::find(exposed_quiddities_.begin(), exposed_quiddities_.end(), quiddity_name);
    if (it != exposed_quiddities_.end()) {
      exposed_quiddities_.erase(it);
    }
  });

  // trying to register if a user is given
  std::string user = config<MPtr(&InfoTree::branch_get_value)>("user");
  if (!user.empty()) {
    g_debug("SIP is trying to register from configuration");
    std::string pass = config<MPtr(&InfoTree::branch_get_value)>("pass");
    pjsip_->run([&]() { sip_presence_->register_account(user, pass); });
    if (sip_presence_->registered_) {
      g_debug("sip registered using configuration file");
    } else {
      g_warning("sip failed registration from configuration");
    }
  }
}

bool SIPPlugin::start_sip_transport() {
  if (-1 != transport_id_) {
    On_scope_exit { transport_id_ = -1; };
    if (pjsua_transport_close(transport_id_, PJ_FALSE) != PJ_SUCCESS) {
      g_warning("cannot close current transport");
      g_message("ERROR: (bug) cannot close current transport");
      return false;
    }
  }

  if (NetUtils::is_used(sip_port_)) {
    g_warning("SIP port cannot be bound (%u)", sip_port_);
    g_message("ERROR: SIP port is not available (%u)", sip_port_);
    return false;
  }
  pjsua_transport_config cfg;
  pjsua_transport_config_default(&cfg);
  cfg.port = sip_port_;
  pj_status_t status = pjsua_transport_create(PJSIP_TRANSPORT_UDP, &cfg, &transport_id_);
  if (status != PJ_SUCCESS) {
    g_warning("Error creating transport");
    g_message("ERROR: SIP port is not available (%u)", sip_port_);
    transport_id_ = -1;
    return false;
  }
  return true;
}

void SIPPlugin::expose_stream_to_quiddity(const std::string& quid_name,
                                          const std::string& shmpath) {
  auto manager = manager_impl_.lock();
  auto quid = Quiddity::string_to_quiddity_name(quid_name);
  std::unique_lock<std::mutex> lock(exposed_quiddities_mutex_);
  if (std::find(exposed_quiddities_.begin(), exposed_quiddities_.end(), quid) !=
      exposed_quiddities_.end())
    return;
  exposed_quiddities_.push_back(quid);
  lock.unlock();
  quid = manager->create("extshmsrc", quid);
  if (quid.empty()) {
    g_warning("Failed to create external shmdata quiddity for pjsip incoming stream.");
    return;
  }
  manager->props<MPtr(&PContainer::set_str_str)>(quid, "shmdata-path", shmpath);
}

void SIPPlugin::remove_exposed_quiddity(const std::string& quid_name) {
  auto manager = manager_impl_.lock();
  auto quid = string_to_quiddity_name(quid_name);

  std::unique_lock<std::mutex> lock(exposed_quiddities_mutex_);
  auto it = std::find(exposed_quiddities_.begin(), exposed_quiddities_.end(), quid);
  if (it == exposed_quiddities_.end()) return;
  exposed_quiddities_.erase(it);
  lock.unlock();

  manager->remove(quid);
}

}  // namespace switcher
