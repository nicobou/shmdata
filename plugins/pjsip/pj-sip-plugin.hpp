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

#ifndef __SWITCHER_PJSIP_PLUGIN_H__
#define __SWITCHER_PJSIP_PLUGIN_H__

#include <pjsua-lib/pjsua.h>
#include <atomic>
#include <memory>
#include "./pj-call.hpp"
#include "./pj-presence.hpp"
#include "./pj-sip.hpp"
#include "./pj-stun-turn.hpp"
#include "./pj-whitelist.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/threaded-wrapper.hpp"

namespace switcher {
class SIPPlugin : public Quiddity {
  friend PJCall;
  friend PJPresence;
  friend PJSIP;
  friend PJStunTurn;
  friend PJWhiteList;

 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(SIPPlugin);
  SIPPlugin(const std::string&);
  ~SIPPlugin();
  SIPPlugin(const SIPPlugin&) = delete;
  SIPPlugin& operator=(const SIPPlugin&) = delete;
  bool init();
  bool start();
  bool stop();

 private:
  std::unique_ptr<ThreadedWrapper<PJSIP>> pjsip_{};
  unsigned int sip_port_{5060};
  PContainer::prop_id_t port_id_;
  std::string dns_address_;
  PContainer::prop_id_t dns_address_id_{0};
  pjsua_transport_id transport_id_{-1};
  bool decompress_streams_{true};
  PContainer::prop_id_t decompress_streams_id_;
  std::unique_ptr<PJCall> sip_calls_{nullptr};
  std::unique_ptr<PJPresence> sip_presence_{nullptr};
  std::unique_ptr<PJStunTurn> stun_turn_{nullptr};
  std::unique_ptr<PJWhiteList> white_list_{nullptr};

  // singleton related members:
  bool i_m_the_one_{false};
  static std::atomic<unsigned short> sip_plugin_used_;
  static SIPPlugin* this_;

  bool start_sip_transport();
  void apply_configuration();
};

}  // namespace switcher
#endif
