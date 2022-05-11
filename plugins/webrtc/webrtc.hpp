/*
 * This file is part of switcher.
 *
 * switcher-webrtc is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_WEBRTC_PLUGIN_H__
#define __SWITCHER_WEBRTC_PLUGIN_H__
// Define GST_USE_UNSTABLE_API otherwise compilation fail with the following message:
// "The WebRTC library from gst-plugins-bad is unstable API and may change in future."
// "You can define GST_USE_UNSTABLE_API to avoid this warning."
#define GST_USE_UNSTABLE_API

#include <atomic>
#include <chrono>
#include <memory>
#include <random>
#include <string>

#include <gst/webrtc/webrtc.h>
#include <json-glib/json-glib.h>
#include <libsoup/soup.h>

#include "switcher/gst/pipeliner.hpp"
#include "switcher/gst/unique-gst-element.hpp"
#include "switcher/infotree/json-serializer.hpp"
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/quiddity/startable.hpp"
#include "switcher/shmdata/follower.hpp"
#include "switcher/utils/scope-exit.hpp"
#include "switcher/utils/threaded-wrapper.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;

//! Webrtc Client
/*! 
 This quiddity implements a webrtc client able to communicate
 with multiple peers, that uses shmdatas as input and output
 streams.
*/
class Webrtc : public Quiddity, public Startable {
 public:
  Webrtc(quiddity::Config&&);
  ~Webrtc();

  //! Generate a random username.
  static std::string make_username();
  //! Convert a JsonObject to std::string.
  static std::string json_to_string(JsonObject*);

 private:
  enum class ShmType { Audio = 0, Video = 1 };
  using bundle_t = std::tuple<Webrtc*, const std::string, GstWebRTCSessionDescription*>;

  struct bundle_cleaner_t {
    void operator()(bundle_t* bundle) {
      if (!bundle) {
        return;
      }

      auto [webrtc, peer, sdp] = *bundle;

      if (sdp) {
        gst_webrtc_session_description_free(sdp);
      }
    }
  };
  using unique_bundle_p = std::unique_ptr<bundle_t, bundle_cleaner_t>;

  template <typename T>
  struct g_object_cleaner_t {
    void operator()(T* gobj) {
      if (gobj) {
        g_object_unref(gobj);
      }
    }
  };
  template <typename T>
  using unique_gobject = std::unique_ptr<T, g_object_cleaner_t<T>>;

  template <typename T>
  struct gst_object_cleaner_t {
    void operator()(T* obj) {
      if (obj) {
        gst_object_unref(obj);
      }
    }
  };
  template <typename T>
  using unique_gst = std::unique_ptr<T, gst_object_cleaner_t<T>>;
  using unique_gst_element = unique_gst<GstElement>;

  struct json_object_cleaner_t {
    void operator()(JsonObject* json) {
      if (json) {
        json_object_unref(json);
      }
    }
  };
  using unique_json_object = std::unique_ptr<JsonObject, json_object_cleaner_t>;

  using signal_handler_t = gulong;
  using handler_tuple_t = std::tuple<signal_handler_t, signal_handler_t>;
  using peer_data_t = std::tuple<handler_tuple_t, unique_bundle_p>;

  static const std::string kConnectionSpec;  //!< Shmdata specifications

  std::unique_ptr<gst::Pipeliner> pipeline_;
  std::mutex pipeline_mutex_{};  // <! avoid concurent access among start/stop and wss messages
  std::mutex swid_mutex_{};      // <! avoid concurent creation of claws
  unique_gobject<SoupWebsocketConnection> connection_;
  std::mutex connection_mutex_{};  // <! avoid concurent access of the connection
  std::unique_ptr<gst::GlibMainLoop> loop_;
  std::map<const std::string, peer_data_t> peers_;

  std::string audio_caps_{};
  std::string video_caps_{};
  std::string audio_shmpath_{};
  std::string video_shmpath_{};
  std::unique_ptr<shmdata::Follower> audio_shmsub_{nullptr};
  std::unique_ptr<shmdata::Follower> video_shmsub_{nullptr};

  ThreadedWrapper<> async_this_{};

  std::string signaling_server_{"wss://localhost:8443"};
  property::prop_id_t signaling_server_id_;

  std::string room_{};
  property::prop_id_t room_id_;

  std::string username_{make_username()};
  property::prop_id_t username_id_;

  std::string stun_server_{"stun://stun.stunprotocol.org:3478"};
  property::prop_id_t stun_server_id_;

  std::string turn_server_{};
  property::prop_id_t turn_server_id_;

  static constexpr const char* https_aliases[]{"wss", nullptr};
  static inline const std::string WEBCLIENT_MAGIC_ID{"SAT_WebRTC_Client_Web"};

  // type of message used to communicate with other clients
  static inline const std::string SAT_COMMAND_TYPE{"customCommand"};
  // command used to ask a web client to stop a call they initiated
  static inline const std::string STOP_CALL_COMMAND{"STOP CALL"};

  gulong wss_closed_handler_id_{0};
  gulong wss_message_handler_id_{0};

  std::atomic<bool> stopping_{false};
  std::mutex loop_lock_;

  static inline std::string make_caps(const std::string& type, const std::string& encoding, int n) {
    return "application/x-rtp,media=" + type + ",encoding-name=" + encoding +
           ",payload=" + std::to_string(n);
  }

  bool on_shmdata_connect(const std::string& shmpath, claw::sfid_t sfid);
  bool on_shmdata_disconnect(claw::sfid_t sfid);

  /*!
    Start the client. Join a room on the signaling server and contact every peer in the room.
  */
  bool start() final;
  static int start_source_cb(void* self);
  void async_connect_to_wss();

  /*!
    Stop the client. Disconnect from the signaling server and stop transmitting/receiving media.
  */
  bool stop() final;
  static int stop_source_cb(void* self);
  void async_disconnect_from_wss();

  bool register_for_signaling();

  bool join_room();

  bool call_peer(const std::string&);
  bool return_call(const std::string&);

  bool start_pipeline();
  bool add_peer_to_pipeline(const std::string&);
  bool remove_peer_from_pipeline(const std::string&);

  bool peer_message(const std::string&, const std::string&);
  bool peer_registered(const std::string&) const;

  bool send_sdp(const std::string&, GstWebRTCSessionDescription*);
  bool send_ice(const std::string&, guint, const std::string&);
  bool send_command(const std::string&, const std::string&, const std::string&);
  bool send_peer_msg(const std::string&, const std::string&);
  bool send_text(const std::string&);

  bool handle_remote_sdp(const std::string&, const std::string&, GstWebRTCSDPType);

  void pad_added(GstPad*, bundle_t*);
  static void on_pad_added(GstElement*, GstPad*, bundle_t*);
  void decodebin_pad_added(GstPad*, bundle_t* data);
  static void on_decodebin_pad_added(GstElement*, GstPad*, bundle_t*);
  void ice_candidate_collected(const guint, const gchar*, unique_bundle_p);
  static void on_ice_candidate_collected(GstElement*, const guint, const gchar*, bundle_t*);
  void offer_created(GstPromise*, bundle_t*);
  static void on_offer_created(GstPromise*, bundle_t*);
  void answer_created(GstPromise*, unique_bundle_p);
  static void on_answer_created(GstPromise*, bundle_t*);
  void local_description_set(GstPromise*, unique_bundle_p);
  static void on_local_description_set(GstPromise*, bundle_t*);
  void server_connected(SoupSession*, GAsyncResult*);
  static void on_server_connected(SoupSession*, GAsyncResult*, SoupMessage*);
  void connection_closed();
  static void on_connection_closed(SoupWebsocketConnection*, Webrtc*);
  void wss_message(GBytes*, SoupWebsocketDataType);
  static void on_wss_message(SoupWebsocketConnection*, SoupWebsocketDataType, GBytes*, Webrtc*);
  void remote_description_set(GstPromise*, unique_bundle_p);
  static void on_remote_description_set(GstPromise*, bundle_t*);
};

SWITCHER_DECLARE_PLUGIN(Webrtc);

}  // namespace quiddities
}  // namespace switcher
#endif
