/*
 * This file is part of switcher-myplugin.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef PLUGINS_PJSIP_PJ_CALL_H_
#define PLUGINS_PJSIP_PJ_CALL_H_

#include <pjsua-lib/pjsua.h>

#include <string>
#include <vector>

#include "./pj-codec.hpp"
#include "switcher/shmdata-any-writer.hpp"
#include "switcher/rtp-session.hpp"
#include "switcher/quiddity-manager.hpp"

#define MAX_CALLS 1024

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
  struct media_stream {
    unsigned call_index {0}; /* Call owner. */
    unsigned media_index {0};  /* Media index in call. */
    pjmedia_transport *transport {nullptr};  /* To send/recv RTP/RTCP */
    pj_bool_t active {PJ_FALSE};  /* Non-zero if is in call. */
    pjmedia_stream_info si;  /* Current stream info: */
    pjmedia_rtp_session out_sess;     /* outgoing RTP session */
    pjmedia_rtp_session in_sess;      /* incoming RTP session */
    pjmedia_rtcp_session rtcp;        /* incoming RTCP session (for stats). */
    // type + codec param
    std::string type {};              /* audio, video or appli */
    std::string extra_params {};
    // shmdata
    ShmdataAnyWriter::ptr shm {};     /* RTP, FIXME make RTCP shm */
    std::string shm_path_to_send {};
    media_stream(): si(), out_sess(), in_sess(), rtcp() {}
    media_stream(const media_stream&) = delete;
    media_stream& operator=(const media_stream&) = delete;
  };

  /* This is a call structure that is created when the application starts
   * and only destroyed when the application quits.
   */
  struct call {
    unsigned index {0};
    pjsip_inv_session *inv {nullptr};
    unsigned media_count {0};
    struct media_stream media[64];
    pj_time_val start_time {0, 0};
    pj_time_val response_time {0, 0};
    pj_time_val connect_time {0, 0};
    std::string peer_uri {};
    PJCall *instance {nullptr};
  };

  // Application's global variables
  typedef struct app {
    unsigned max_calls {256};
    unsigned uac_calls {0};
    pj_str_t local_addr {nullptr, 0};
    struct call call[MAX_CALLS];
  } app_t;

 private:
  static pjmedia_endpt *med_endpt_;
  static pjsip_module mod_siprtp_;
  static app_t app;
  PJSIP *sip_instance_;
  // internal rtp
  QuiddityManager::ptr manager_;

  // external rtp session quidity for sending
  // std::string rtp_session_name_ {};
  // GParamSpec *rtp_session_name_spec_ {nullptr};

  uint starting_rtp_port_ {18000};
  GParamSpec *starting_rtp_port_spec_ {nullptr};

  // sip functions
  static pj_bool_t on_rx_request(pjsip_rx_data *rdata);
  static void call_on_state_changed(pjsip_inv_session *inv,
                                    pjsip_event *e);
  static void call_on_forked(pjsip_inv_session * inv, pjsip_event *e);
  static void call_on_media_update(pjsip_inv_session *inv,
                                   pj_status_t status);
  static void process_incoming_call(pjsip_rx_data *rdata);
  void init_app();
  static pj_status_t create_sdp(pj_pool_t *pool,
                                struct call *call,
                                const std::vector <pjmedia_sdp_media *>&
                                media_to_receive,
                                pjmedia_sdp_session **p_sdp);
  static void on_rx_rtp(void *user_data, void *pkt, pj_ssize_t size);
  static void on_rx_rtcp(void *user_data, void *pkt, pj_ssize_t size);
  static
  pj_status_t parse_SDP_from_incoming_request(pjsip_rx_data *rdata,
                                              pjmedia_sdp_session *offer);
  static void print_sdp(const pjmedia_sdp_session *local_sdp);
  static pj_status_t stream_info_from_sdp(pjmedia_stream_info *si,
                                          pj_pool_t *pool,
                                          pjmedia_endpt *endpt,
                                          const pjmedia_sdp_session *
                                          local,
                                          const pjmedia_sdp_session *
                                          remote, unsigned stream_idx);
  static pj_status_t get_audio_codec_info_param(pjmedia_stream_info *si,
                                                pj_pool_t *pool,
                                                pjmedia_codec_mgr *mgr,
                                                const pjmedia_sdp_media *
                                                local_m,
                                                const pjmedia_sdp_media *
                                                rem_m);
  static void remove_from_sdp_media(pjmedia_sdp_media *sdp_media,
                                    unsigned fmt_pos);
  pj_status_t make_call(std::string contact_uri);
  std::string create_outgoing_sdp(struct call *call, std::string dst_uri);
  Quiddity::ptr retrieve_rtp_manager();
  static gboolean call_sip_url(gchar *sip_url, void *user_data);
  static void set_starting_rtp_port(const gint value, void *user_data);
  static gint get_starting_rtp_port(void *user_data);
  bool make_hang_up(std::string contact_uri);
  static gboolean hang_up(gchar *sip_url, void *user_data);
  static gboolean attach_shmdata_to_contact(gchar *shmpath,
                                            gchar *contact_uri,
                                            gboolean attach,
                                            void *user_data);
  void make_attach_shmdata_to_contact(std::string shmpath,
                                      std::string contact_uri,
                                      bool attach);
};
}  // namespace switcher

#endif  // PLUGINS_PJSIP_PJ_CALL_H_
