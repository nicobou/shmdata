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
#include <map>
#include <mutex>
#include <condition_variable>
#include "switcher/rtp-session2.hpp"
#include "switcher/quiddity-manager.hpp"
#include "switcher/gst-shmdata-to-cb.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/shmdata-decoder.hpp"
#include "switcher/gst-pipeliner.hpp"
#include "./pj-sip-plugin.hpp"
#include "./pj-sip.hpp"
#include "./pj-codec.hpp"
#include "./pj-ice-stream-trans.hpp"
#include "./pj-media-endpt.hpp"

namespace switcher {
class SIPPlugin;
class PJCall {

 public:
  PJCall();
  ~PJCall() = default;
  PJCall(const PJCall &) = delete;
  PJCall &operator=(const PJCall &) = delete;

 private:
  /* Media stream created when the call is active. */
  using media_t = struct media_stream {
    pj_uint16_t rtp_port{0};  // sending  // FIXME remove this, using ICE only
    std::string shm_path_to_send{};
    RTPSender::id_t cb_id{0};
    pj_sockaddr	def_addr;
  };

  // This is a call structure that is created when the application starts
  // and only destroyed when the application quits.
  using call_t = struct call {
    pjsip_inv_session *inv {nullptr};
    // as receiver
    std::unique_ptr<PJICEStreamTrans> ice_trans_{};
    std::vector<std::unique_ptr<ShmdataWriter>> rtp_writers_{};
    // std::unique_ptr<GstPipeliner> pipeliner_{};
    // std::vector<std::unique_ptr<ShmdataDecoder>> rtp_decoders_{};
    std::unique_ptr<RtpSession2> recv_rtp_session_{};
    std::vector<std::unique_ptr<RTPReceiver>> rtp_receivers_{};
    // as sender
    std::unique_ptr<PJICEStreamTrans> ice_trans_send_{};
    // media
    std::vector<media_t> media{};
    std::string peer_uri{};
  };

 private:
  static pjsip_module mod_siprtp_;
  SIPPlugin *sip_plugin_;
  pj_str_t local_addr_ {nullptr, 0};
  bool is_calling_{false};
  bool is_hanging_up_{false};
  std::map<std::string, std::string> local_ips_{};
  std::mutex ocall_m_{};
  std::condition_variable ocall_cv_{};
  bool ocall_action_done_{false};
  // internal rtp
  QuiddityManager::ptr manager_;
  std::map<std::string, unsigned> reader_ref_count_{};
  // // saving association between reception quids (httpsdpdec) and uris:
  // std::map<std::string, std::string> quid_uri_{};
  InfoTree::ptr contact_shm_;
  uint starting_rtp_port_ {18900};
  pj_uint16_t next_port_to_attribute_{18900};  // Must be even
  uint port_range_{100};

  RtpSession2 rtp_session_{};
  std::unique_ptr<PJMediaEndpt> med_endpt_{nullptr};
  std::vector<call_t> incoming_call_{};
  std::vector<call_t> outgoing_call_{};
  std::map<std::string, std::unique_ptr<RTPSender>> readers_{};

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
  void make_hang_up(std::string contact_uri);
  static gboolean hang_up(gchar *sip_url, void *user_data);
  static gboolean attach_shmdata_to_contact(const gchar *shmpath,
                                            const gchar *contact_uri,
                                            gboolean attach,
                                            void *user_data);
  void make_attach_shmdata_to_contact(const std::string &shmpath,
                                      const std::string &contact_uri,
                                      bool attach);
  // static void internal_manager_cb(const std::string &/*subscriber_name */,
  //                                 const std::string &/*quiddity_name */,
  //                                 const std::string &signal_name,
  //                                 const std::vector<std::string> &params,
  //                                 void */*user_data */);
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

  static std::unique_ptr<PJICEStreamTrans> negociate_ice_transport(
      const pjmedia_sdp_session *remote_sdp,
      pj_pool_t *dlg_pool);
};

}  // namespace switcher
#endif
