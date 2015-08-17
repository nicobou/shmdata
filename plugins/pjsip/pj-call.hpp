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

#ifndef PLUGINS_PJSIP_PJ_CALL_H_
#define PLUGINS_PJSIP_PJ_CALL_H_

#include <pjsua-lib/pjsua.h>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "switcher/rtp-session.hpp"
#include "switcher/quiddity-manager.hpp"
#include "./pj-codec.hpp"

namespace switcher {
class PJSIP;
class PJCall {
  friend PJCodec;

 public:
  PJCall() = delete;
  explicit PJCall(PJSIP *sip_instance);
  ~PJCall();
  PJCall(const PJCall &) = delete;
  PJCall &operator=(const PJCall &) = delete;

 private:
  /* Media stream created when the call is active. */
  using media_t = struct media_stream {
    pj_uint16_t rtp_port{0};  // sending
    std::string shm_path_to_send {};
  };

  // This is a call structure that is created when the application starts
  // and only destroyed when the application quits.
  using call_t = struct call {
    pjsip_inv_session *inv {nullptr};
    std::vector<media_t> media{};
    std::string peer_uri {};
  };

 private:
  static pjmedia_endpt *med_endpt_;
  static pjsip_module mod_siprtp_;
  pj_str_t local_addr_ {nullptr, 0};
  std::vector<call_t> outgoing_call_{};
  std::mutex ocall_m_{};
  std::condition_variable ocall_cv_{};
  bool ocall_action_done_{false};
  bool is_calling_{false};
  bool is_hanging_up_{false};
  std::vector<call_t> incoming_call_{};
  std::vector<call_t> call_{};
  std::map<std::string, std::string> local_ips_{};
  PJSIP *sip_instance_;
  // internal rtp
  QuiddityManager::ptr manager_;
  // saving association between reception quids (httpsdpdec) and uris:
  std::map<std::string, std::string> quid_uri_{};
  data::Tree::ptr contact_shm_;
  uint starting_rtp_port_ {18900};
  pj_uint16_t next_port_to_attribute_{18900};  // Must be even
  uint port_range_{100};
  GParamSpec *starting_rtp_port_spec_ {nullptr};
  // sip functions
  static pj_bool_t on_rx_request(pjsip_rx_data *rdata);
  static void call_on_state_changed(pjsip_inv_session *inv,
                                    pjsip_event *e);
  static void call_on_forked(pjsip_inv_session * inv, pjsip_event *e);
  static void call_on_media_update(pjsip_inv_session *inv,
                                   pj_status_t status);
  static void call_on_rx_offer(pjsip_inv_session *inv,
                               const pjmedia_sdp_session *offer);
  static void process_incoming_call(pjsip_rx_data *rdata);
  static pj_status_t create_sdp_answer(pj_pool_t *pool,
                                       struct call *call,
                                       const std::vector<pjmedia_sdp_media *>&media_to_receive,
                                       pjmedia_sdp_session **p_sdp);
  static pj_status_t parse_SDP_from_incoming_request(pjsip_rx_data *rdata,
                                                     pjmedia_sdp_session *offer);
  void make_call(std::string contact_uri);
  void create_outgoing_sdp(pjsip_dialog *dlg,
                           call_t *call,
                           pjmedia_sdp_session **res);
  Quiddity::ptr retrieve_rtp_manager();
  static gboolean send_to(gchar *sip_url, void *user_data);
  static void set_starting_rtp_port(const gint value, void *user_data);
  static gint get_starting_rtp_port(void *user_data);
  void make_hang_up(std::string contact_uri);
  static gboolean hang_up(gchar *sip_url, void *user_data);
  static gboolean attach_shmdata_to_contact(const gchar *shmpath,
                                            const gchar *contact_uri,
                                            gboolean attach,
                                            void *user_data);
  void make_attach_shmdata_to_contact(const std::string &shmpath,
                                      const std::string &contact_uri,
                                      bool attach);
  static void internal_manager_cb(const std::string &/*subscriber_name */,
                                  const std::string &/*quiddity_name */,
                                  const std::string &signal_name,
                                  const std::vector<std::string> &params,
                                  void */*user_data */);
  static void on_inv_state_disconnected(struct call *call,
                                        pjsip_inv_session *inv,
                                        pjsua_buddy_id id);
  static void on_inv_state_confirmed(struct call *call,
                                     pjsip_inv_session *inv,
                                     pjsua_buddy_id id);
  static void on_inv_state_early(struct call *call,
                                 pjsip_inv_session *inv,
                                 pjsua_buddy_id id);
  static void on_inv_state_connecting(struct call *call,
                                      pjsip_inv_session *inv,
                                      pjsua_buddy_id id);
  static bool release_incoming_call(call_t *call, pjsua_buddy_id id);
  static bool release_outgoing_call(call_t *call, pjsua_buddy_id id);
  static void print_sdp(const pjmedia_sdp_session *local_sdp);
};

}  // namespace switcher
#endif
