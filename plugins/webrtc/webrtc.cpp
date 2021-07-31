/*
 * This file is part of switcher-webrtc.
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

#include "webrtc.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Webrtc,
                                     "webrtc",
                                     "WebRTC Client",
                                     "audio/video",
                                     "reader/writer",
                                     "Plugin implementing a simple WebRTC client",
                                     "LGPL",
                                     "Hantz-Carly F. Vius");

const std::string Webrtc::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "audio",
      "description": "Audio stream for participation in the Webrtc session",
      "can_do": ["audio/x-raw"]
    },
    {
      "label": "video",
      "description": "Video stream for participation in the Webrtc session",
      "can_do": ["video/x-raw"]
    }
  ],
"writer":
  [
    {
      "label": "audio",
      "description": "Audio stream received from the Webrtc session",
      "can_do": ["audio/x-raw"]
    },
    {
      "label": "video",
      "description": "Video stream received from the Webrtc session",
      "can_do": [ "video/x-raw" ]
    }
  ]
}
)");

Webrtc::Webrtc(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf),
               {kConnectionSpec,
                [this](const std::string& shmpath, claw::sfid_t sfid) {
                  return on_shmdata_connect(shmpath, sfid);
                },
                [this](claw::sfid_t sfid) { return on_shmdata_disconnect(sfid); }}),
      quiddity::Startable(this),
      pipeline_(std::make_unique<gst::Pipeliner>(nullptr, nullptr)),
      signaling_server_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "signaling_server",
          [this](const std::string& val) {
            signaling_server_ = val;
            return true;
          },
          [this]() { return signaling_server_; },
          "WebSocket Signaling Server",
          "Address of the signaling server,  e.g. wss://1.1.1.1:8443",
          signaling_server_)),
      room_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "room",
          [this](const std::string& val) {
            room_ = val;
            return true;
          },
          [this]() { return room_; },
          "Room",
          "Room to join on the signaling server",
          room_)),
      username_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "username",
          [this](const std::string& val) {
            username_ = val;
            return true;
          },
          [this]() { return username_; },
          "Username",
          "Username to use on the server",
          username_)) {}

Webrtc::~Webrtc() {
  connection_closed();
  std::lock_guard<std::mutex> pipe_lock(pipeline_mutex_);
  pipeline_->play(false);
}

std::string Webrtc::make_username() {
  static std::random_device rdevice_;
  static std::mt19937 generator_(rdevice_());
  static std::uniform_int_distribution<unsigned short> ushort_distribution(
      std::numeric_limits<unsigned short>::min(), std::numeric_limits<unsigned short>::max());

  return "unnamed-" + std::to_string(ushort_distribution(generator_));
}

bool Webrtc::stop() {
  debug("Webrtc::stop");
  using namespace std::chrono_literals;

  std::lock_guard<std::mutex> pipe_lock(pipeline_mutex_);
  if (stopping_) {
    warning("Webrtc::stop::Already stopping. Skipping");
    return true;
  } else {
    stopping_ = true;
  }

  if (!connection_) {
    error("Webrtc::stop::Websocket connection is uninitialized");
    return true;
  }

  {
    std::scoped_lock lock(loop_lock_);
    if (!loop_) {
      error("Webrtc::stop::Main loop is uninitialized");
      return false;
    }

    pipeline_ = std::make_unique<gst::Pipeliner>(nullptr, nullptr);

    std::unique_ptr<GSource, decltype(&g_source_unref)> source(g_idle_source_new(), g_source_unref);

    g_source_set_callback(source.get(), Webrtc::stop_source_cb, this, nullptr);

    g_source_attach(source.get(), loop_->get_main_context());
  }  // endlock

  SoupWebsocketState state = SOUP_WEBSOCKET_STATE_OPEN;
  g_object_get(connection_.get(), "state", &state, nullptr);
  for (auto start = std::chrono::high_resolution_clock::now();
       connection_ && std::chrono::high_resolution_clock::now() - start < 3000ms &&
       state != SOUP_WEBSOCKET_STATE_CLOSED;
       g_object_get(connection_.get(), "state", &state, nullptr),
            std::this_thread::sleep_for(1ms)) {
  }

  if (state != SOUP_WEBSOCKET_STATE_CLOSED) {
    std::string state_str;
    switch (state) {
      case SOUP_WEBSOCKET_STATE_OPEN:
        state_str = "OPEN";
        break;
      case SOUP_WEBSOCKET_STATE_CLOSING:
        state_str = "CLOSING";
        break;
      case SOUP_WEBSOCKET_STATE_CLOSED:
        g_assert_not_reached();
        break;
      default:
        state_str = "UNKNOWN";
        break;
    }
    warning("Webrtc::stop::Websocket disconnection timed out. Current state is [%]", state_str);
  }

  return state == SOUP_WEBSOCKET_STATE_CLOSED;
}

int Webrtc::stop_source_cb(void* self) {
  static_cast<Webrtc*>(self)->async_disconnect_from_wss();
  return 0;  // false
}

void Webrtc::async_disconnect_from_wss() {
  debug("Webrtc::async_disconnect_from_wss");

  std::lock_guard<std::mutex> pipe_lock(pipeline_mutex_);

  if (!connection_) {
    debug("Webrtc::disconnect::Connection is uninitialized. Nothing to do");
    return;
  }

  for (const auto& peer : peers_) {
    remove_peer_from_pipeline(peer.first);
  }
  pipeline_->play(false);
  pipeline_ = std::make_unique<gst::Pipeliner>(nullptr, nullptr);
  soup_websocket_connection_close(connection_.get(), 1000, "");
}

bool Webrtc::start() {
  debug("Webrtc::start");
  using namespace std::chrono_literals;

  if (stopping_) {
    warning("Webrtc::start::Unfinished stop task previously launched. Skipping start.");
    return false;
  }

  std::unique_ptr<GSource, decltype(&g_source_unref)> source(g_idle_source_new(), g_source_unref);

  g_source_set_callback(source.get(), Webrtc::start_source_cb, this, nullptr);

  {
    std::scoped_lock lock(loop_lock_);
    loop_ = std::make_unique<gst::GlibMainLoop>();
    g_source_attach(source.get(), loop_->get_main_context());
  }

  SoupWebsocketState state = SOUP_WEBSOCKET_STATE_CLOSED;
  for (auto start = std::chrono::high_resolution_clock::now();
       std::chrono::high_resolution_clock::now() - start < 3000ms && !connection_;
       std::this_thread::sleep_for(1ms)) {
  }

  if (connection_) {
    debug("Webrtc::start::Connection object successfully initialized");
    g_object_get(connection_.get(), "state", &state, nullptr);
  }

  if (state != SOUP_WEBSOCKET_STATE_OPEN) {
    std::string state_str;
    switch (state) {
      case SOUP_WEBSOCKET_STATE_OPEN:
        g_assert_not_reached();
        break;
      case SOUP_WEBSOCKET_STATE_CLOSING:
        state_str = "CLOSING";
        break;
      case SOUP_WEBSOCKET_STATE_CLOSED:
        state_str = "CLOSED";
        break;
      default:
        state_str = "UNKNOWN";
        break;
    }
    warning("Webrtc::start::Websocket connection not open. Current state is [%]", state_str);
  } else {
    debug("Webrtc::start::Connection state is: [OPEN]");
  }

  return state == SOUP_WEBSOCKET_STATE_OPEN;
}

int Webrtc::start_source_cb(void* self) {
  static_cast<Webrtc*>(self)->async_connect_to_wss();
  return 0;  // false
}

void Webrtc::async_connect_to_wss() {
  debug("Webrtc::async_connect_to_wss");

  {
    std::scoped_lock lock(loop_lock_);
    if (loop_) {
      g_main_context_push_thread_default(loop_->get_main_context());
    } else {
      error("Webrtc::async_connect_to_wss::Mainloop object is uninitialized");
      return;
    }
  }

  SoupSession* session = soup_session_new_with_options(SOUP_SESSION_SSL_STRICT,
                                                       false,
                                                       SOUP_SESSION_SSL_USE_SYSTEM_CA_FILE,
                                                       true,
                                                       SOUP_SESSION_HTTPS_ALIASES,
                                                       Webrtc::https_aliases,
                                                       nullptr);

  SoupMessage* message = soup_message_new(SOUP_METHOD_GET, signaling_server_.c_str());

  debug("Webrtc::async_connect_to_wss::Connecting...");
  soup_session_websocket_connect_async(session,
                                       message,
                                       nullptr,
                                       nullptr,
                                       nullptr,
                                       (GAsyncReadyCallback)G_CALLBACK(Webrtc::on_server_connected),
                                       this);
}

void Webrtc::on_connection_closed(SoupWebsocketConnection* connection G_GNUC_UNUSED, Webrtc* self) {
  self->connection_closed();
}

void Webrtc::connection_closed() {
  debug("Webrtc::connection_closed");
  g_signal_handler_disconnect(connection_.get(), wss_closed_handler_id_);
  g_signal_handler_disconnect(connection_.get(), wss_message_handler_id_);
  connection_.reset(nullptr);
  stopping_ = false;
}

void Webrtc::on_server_connected(SoupSession* session,
                                 GAsyncResult* res,
                                 SoupMessage* msg G_GNUC_UNUSED) {
  auto self = static_cast<Webrtc*>(g_async_result_get_user_data(res));
  self->server_connected(session, res);
}

void Webrtc::server_connected(SoupSession* session, GAsyncResult* res) {
  debug("Webrtc::server_connected");

  GError* err = nullptr;
  connection_.reset(soup_session_websocket_connect_finish(session, res, &err));

  if (err) {
    error("Webrtc::server_connected::[%]", err->message);
    g_error_free(err);
    return;
  }

  if (!connection_) {
    error("Webrtc::server_connected::Connection object is uninitialized");
    return;
  }

  debug("Webrtc::server_connected::Successfully connected to the server");

  wss_closed_handler_id_ =
      g_signal_connect(connection_.get(), "closed", G_CALLBACK(Webrtc::on_connection_closed), this);
  if (wss_closed_handler_id_ <= 0) {
    error(
        "Webrtc::server_connected::Couldn't subscribe to 'closed' signals on the websocket "
        "connection object");
    return;
  }

  wss_message_handler_id_ =
      g_signal_connect(connection_.get(), "message", G_CALLBACK(Webrtc::on_wss_message), this);
  if (wss_message_handler_id_ <= 0) {
    error(
        "Webrtc::server_connected::Couldn't subscribe to 'message' signals on the webscocket "
        "connection object");
    g_signal_handler_disconnect(connection_.get(), wss_closed_handler_id_);
    return;
  }

  register_for_signaling();
}

void Webrtc::on_wss_message(SoupWebsocketConnection* con G_GNUC_UNUSED,
                            SoupWebsocketDataType type,
                            GBytes* message,
                            Webrtc* self) {
  self->wss_message(message, type);
}

void Webrtc::wss_message(GBytes* message, SoupWebsocketDataType type) {
  debug("Webrtc::wss_message");

  if (stopping_) {
    warning("Webrtc::wss_message::Stop task launched. Ignoring messsage");
    return;
  }

  std::string text;
  switch (type) {
    case SOUP_WEBSOCKET_DATA_BINARY:
      error("Webrtc::wss_message::Unsupported type SOUP_WEBSOCKET_DATA_BINARY");
      return;
    case SOUP_WEBSOCKET_DATA_TEXT: {
      gsize size;
      auto data = static_cast<const gchar*>(g_bytes_get_data(message, &size));  // observer_ptr
      text = std::string(data, size);
      break;
    }
    default:
      error("Webrtc::wss_message::Ignoring message of unknown type [%]", text);
      return;
  }

  debug("Webrtc::wss_message::[%]", text);

  if (text == "HELLO") {
    debug("Webrtc::registered_for_signaling");
    join_room();
  } else if (stringutils::starts_with(text, "ROOM_")) {
    if (stringutils::starts_with(text, "ROOM_OK")) {
      debug("Webrtc::room_joined");
      if (!start_pipeline()) {
        error("Webrtc::wss_message::Couldn't start pipeline");
        return;
      }

      if (text != "ROOM_OK ") {
        auto parts = stringutils::split_string(text, " ");

        debug("Webrtc::wss_message::Found [%] users in the room", std::to_string(parts.size()));

        for (std::size_t idx = 1; idx < parts.size(); ++idx) {
          if (!call_peer(parts[idx])) {
            error("Webrtc::wss_message::Call to [%] failed", parts[idx]);
          }
        }
      } else {
        debug("Webrtc::wss_message::No other user in the room");
      }
    } else if (stringutils::starts_with(text, "ROOM_PEER_")) {
      if (stringutils::starts_with(text, "ROOM_PEER_MSG")) {
        auto parts = stringutils::split_string(text, " ", 3);
        if (!peer_registered(parts[1])) {
          error("Webrtc::wss_message::Ignoring message from unknown peer [%]", parts[1]);
          return;
        }

        if (!peer_message(parts[1], parts[2])) {
          error("Webrtc::wss_message::Failed to handle message [%] from [%]\n", parts[2], parts[1]);
          return;
        }
      } else if (stringutils::starts_with(text, "ROOM_PEER_JOINED")) {
        auto parts = stringutils::split_string(text, " ", 2);
        debug("Webrtc::wss_message::[%] joined the room", parts[1]);
        if (!add_peer_to_pipeline(parts[1])) {
          error("Webrtc::wss_message::Couldn't add peer [%] to the pipeline", parts[1]);
          return;
        }
      } else if (stringutils::starts_with(text, "ROOM_PEER_LEFT")) {
        auto parts = stringutils::split_string(text, " ", 2);
        debug("Webrtc::wss_message::[%] left the room", parts[1]);
        if (peer_registered(parts[1])) {
          remove_peer_from_pipeline(parts[1]);
        }
      } else {
        error("Webrtc::wss_message::Ignoring unparsable message [%]", text);
      }
    }
  }
}

bool Webrtc::peer_message(const std::string& peer, const std::string& message) {
  debug("Webrtc::peer_message");

  unique_gobject<JsonParser> parser(json_parser_new());
  if (!json_parser_load_from_data(parser.get(), message.c_str(), message.length(), nullptr)) {
    error("Webrtc::peer_message::Ignoring unparsable message [%] from [%]", message, peer);
    return false;
  }

  JsonNode* root = json_parser_get_root(parser.get());  // observer_ptr
  if (!JSON_NODE_HOLDS_OBJECT(root)) {
    error("Webrtc::peer_message::Ignoring unparsable (empty?) message [%] from [%]", message, peer);
    return false;
  }

  JsonObject* object = json_node_get_object(root);  // observer_ptr

  if (json_object_has_member(object, "sdp")) {
    JsonObject* child = json_object_get_object_member(object, "sdp");

    if (!json_object_has_member(child, "type")) {
      error("Webrtc::peer_message::SDP message received without {type}");
      return false;
    }

    auto type = std::string(json_object_get_string_member(child, "type"));
    auto sdp = std::string(json_object_get_string_member(child, "sdp"));

    debug("Webrtc::peer_message::sdp::[type: [%] - [description: [%]", type, sdp);

    if (type == "offer") {
      if (json_object_has_member(object, "satid") &&
          json_object_get_string_member(object, "satid") == WEBCLIENT_MAGIC_ID) {
        debug("Webrtc::peer_message::Incoming call from SAT Web client, returning call");
        return_call(peer);
      } else {
        if (!handle_remote_sdp(peer, sdp.c_str(), GST_WEBRTC_SDP_TYPE_OFFER)) {
          error("Webrtc::peer_message::Failed to handle remote SDP offer [%] from [%]",
                message,
                peer);
          return false;
        }
      }
    } else if (type == "answer") {
      if (!handle_remote_sdp(peer, sdp.c_str(), GST_WEBRTC_SDP_TYPE_ANSWER)) {
        error(
            "Webrtc::peer_message::Failed to handle remote SDP answer [%] from [%]", message, peer);
        return false;
      }
    } else {
      error("Webrtc::peer_message::Invalid sdp type");
      return false;
    }
  } else if (json_object_has_member(object, "ice")) {
    JsonObject* child = json_object_get_object_member(object, "ice");
    const gchar* candidate = json_object_get_string_member(child, "candidate");
    int mline = json_object_get_int_member(child, "sdpMLineIndex");

    debug("Webrtc::peer_message::ice:[mline: [%] - [candidate: [%]",
          std::to_string(mline),
          candidate);

    unique_gst<GstElement> webrtc(
        gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), peer.c_str()));
    if (!webrtc) {
      error("Webrtc::peer_message::Couldn't get webrtcbin for peer [%]", peer);
      return false;
    }

    g_signal_emit_by_name(webrtc.get(), "add-ice-candidate", mline, candidate);
  } else {
    error("Webrtc::peer_message::Ignoring unknown JSON message [%] from [%]", message, peer);
    return false;
  }

  return true;
}

bool Webrtc::return_call(const std::string& peer) {
  debug("Webrtc::return_call::[%]", peer);

  if (!send_command(peer, SAT_COMMAND_TYPE, STOP_CALL_COMMAND)) {
    error("Webrtc::return_call::Failed to send stop call command to peer [%]", peer);
    return false;
  }

  if (!remove_peer_from_pipeline(peer)) {
    error("Webrtc::return::Failed to remove peer [%] from pipeline", peer);
    return false;
  }

  return call_peer(peer);
}

bool Webrtc::handle_remote_sdp(const std::string& peer,
                               const std::string& text,
                               GstWebRTCSDPType type) {
  debug("Webrtc::handle_remote_sdp");

  GstSDPMessage* sdp;

  if (gst_sdp_message_new(&sdp) != GST_SDP_OK) {
    error("Webrtc::handle_remote_sdp::Couldn't create GstSDPMessage object");
    return false;
  }

  if (gst_sdp_message_parse_buffer((guint8*)text.c_str(), text.length(), sdp) != GST_SDP_OK) {
    error("Webrtc::handle_remote_sdp::Couldn't parse sdp message [%]", text);
    return false;
  }

  GstWebRTCSessionDescription* description = gst_webrtc_session_description_new(type, sdp);
  if (!description) {
    error("Webrtc::handle_remote_sdp::Couldn't create GstWebRTCSessionDescription object");
    return false;
  }

  unique_gst<GstElement> webrtc(
      gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), peer.c_str()));
  if (!webrtc) {
    error("Webrtc::handle_remote_sdp::Couldn't get webrtcbin for peer [%]", peer);
    return false;
  }

  auto self = type == GST_WEBRTC_SDP_TYPE_OFFER ? this : nullptr;
  GstPromise* promise =
      gst_promise_new_with_change_func((GstPromiseChangeFunc)on_remote_description_set,
                                       (gpointer) new bundle_t(self, peer, description),
                                       nullptr);
  g_signal_emit_by_name(webrtc.get(), "set-remote-description", description, promise);
  return true;
}

void Webrtc::on_remote_description_set(GstPromise* promise, bundle_t* data) {
  auto [self, peer, description] = *data;

  gst_webrtc_session_description_free(description);

  if (self) {
    self->remote_description_set(promise, unique_bundle_p(data));
  } else {
    gst_promise_unref(promise);
  }
}

void Webrtc::remote_description_set(GstPromise* promise, unique_bundle_p data) {
  debug("Webrtc::remote_description_set");

  if (stopping_) {
    warning("Webrtc::remote_description_set::Stop task launched. Ignoring.");
    return;
  }

  auto [self, peer, description] = *data;

  unique_gst<GstElement> webrtc(
      gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), peer.c_str()));
  if (!webrtc) {
    error("Webrtc::remote_description_set::Couldn't get webrtcbin for peer [%]", peer);
    return;
  }

  gst_promise_unref(promise);
  promise = gst_promise_new_with_change_func(
      (GstPromiseChangeFunc)on_answer_created, (gpointer)(data.release()), nullptr);

  g_signal_emit_by_name(webrtc.get(), "create-answer", nullptr, promise);
}

void Webrtc::on_answer_created(GstPromise* promise, bundle_t* data) {
  auto self = std::get<Webrtc*>(*data);
  self->answer_created(promise, unique_bundle_p(data));
}

void Webrtc::answer_created(GstPromise* promise, unique_bundle_p data) {
  debug("Webrtc::answer_created");

  if (stopping_) {
    warning("Webrtc::answer_created::Stop task launched. Ignoring.");
    return;
  }

  if (gst_promise_wait(promise) != GST_PROMISE_RESULT_REPLIED) {
    error("Webrtc::answer_created::No result set on promise");
    return;
  }

  const GstStructure* reply = gst_promise_get_reply(promise);  // observer_ptr

  GstWebRTCSessionDescription* sdp;
  gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &sdp, nullptr);
  std::get<GstWebRTCSessionDescription*>(*data) = sdp;

  auto peer = std::get<const std::string>(*data);
  unique_gst<GstElement> webrtc(
      gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), peer.c_str()));
  if (!webrtc) {
    error("Webrtc::answer_created::Couldn't get webrtcbin for peer [%]", peer);
    return;
  }

  gst_promise_unref(promise);
  promise = gst_promise_new_with_change_func(
      (GstPromiseChangeFunc)on_local_description_set, (gpointer)(data.release()), nullptr);

  debug("Webrtc::answer_created::Setting local description[%]", peer);
  g_signal_emit_by_name(webrtc.get(), "set-local-description", sdp, promise);
}

bool Webrtc::peer_registered(const std::string& peer) const {
  return peers_.find(peer) != peers_.end();
}


bool Webrtc::register_for_signaling() {
  debug("Webrtc::register_for_signaling");

  if (soup_websocket_connection_get_state(connection_.get()) != SOUP_WEBSOCKET_STATE_OPEN) {
    return false;
  }

  debug("Webrtc::register_for_signaling::Registering with server as [%]", username_);

  std::string hello = "HELLO " + username_;
  send_text(hello);
  return true;
}

bool Webrtc::join_room() {
  if (soup_websocket_connection_get_state(connection_.get()) != SOUP_WEBSOCKET_STATE_OPEN) {
    return false;
  }

  debug("Webrtc::join_room::Joining room: [%]", room_.c_str());

  std::string msg = "ROOM " + room_;
  send_text(msg);

  return true;
}


bool Webrtc::start_pipeline() {
  debug("Webrtc::start_pipeline");

  std::string pipe = "tee name=videotee ! queue ! fakesink " + video_ +
                     " ! videoconvert ! queue ! vp8enc ! rtpvp8pay ! queue ! " +
                     make_caps("video", "VP8", 96) + " ! videotee. " +
                     "tee name=audiotee ! queue ! fakesink " + audio_ +
                     " ! audioconvert ! audioresample ! queue ! opusenc ! rtpopuspay ! queue ! " +
                     make_caps("audio", "OPUS", 96) + " ! audiotee. ";

  GError* err = nullptr;
  GstElement* el = gst_parse_bin_from_description_full(
      pipe.c_str(), false, nullptr, GST_PARSE_FLAG_NO_SINGLE_ELEMENT_BINS, &err);

  if (err) {
    error("Webrtc::start_pipeline::[%]", err->message);
    g_error_free(err);
    return false;
  }

  std::lock_guard<std::mutex> pipe_lock(pipeline_mutex_);

  pipeline_ = std::make_unique<gst::Pipeliner>(nullptr, nullptr);
  if (!pipeline_) {
    error("Webrtc::start_pipeline::Failed to create pipeline object");
    return false;
  }

  g_object_set(G_OBJECT(pipeline_->get_pipeline()), "async-handling", true, nullptr);
  g_object_set(G_OBJECT(el), "async-handling", true, nullptr);

  if (!audio_shmpath_.empty()) {
    auto shmdataaudio = gst_bin_get_by_name(GST_BIN(el), "shmaudio");
    g_object_set(G_OBJECT(shmdataaudio), "socket-path", audio_shmpath_.c_str(), nullptr);
  }
  
  if (!video_shmpath_.empty()) {
    auto shmdatavideo = gst_bin_get_by_name(GST_BIN(el), "shmvideo");
    g_object_set(G_OBJECT(shmdatavideo), "socket-path", video_shmpath_.c_str(), nullptr);
  }



  if (auto added = gst_bin_add(GST_BIN(pipeline_->get_pipeline()), el); added) {
    debug("Webrtc::start_pipeline::Added streams to pipeline");
  } else {
    error("Webrtc::start_pipeline::Failed to add streams to pipeline");
    return false;
  }

  pipeline_->play(true);

  return true;
}

bool Webrtc::call_peer(const std::string& peer) {
  debug("Webrtc::call_peer::[%]", peer);

  add_peer_to_pipeline(peer);

  GstElement* webrtc = gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), peer.c_str());
  if (!webrtc) {
    error("Webrtc::call_peer::Couldn't get webrtcbin for peer [%]", peer);
    return false;
  }

  GstPromise* promise =
      gst_promise_new_with_change_func((GstPromiseChangeFunc)on_offer_created,
                                       (gpointer) new bundle_t(this, peer, nullptr),
                                       nullptr);

  debug("Webrtc::call_peer::Creating offer");
  g_signal_emit_by_name(webrtc, "create-offer", nullptr, promise);
  return true;
}

bool Webrtc::remove_peer_from_pipeline(const std::string& peer) {
  debug("Webrtc::remove_peer_from_pipeline:[%]", peer);

  if (!peer_registered(peer)) {
    debug("Webrtc::remove_peer_from_pipeline:[%]: is not registered", peer);
    return true;
  }

  unique_gst_element audio_tee(gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), "audiotee"));
  if (!audio_tee) {
    error("Webrtc::remove_peer_from_pipeline:[%]:Couldn't retrieve tee [audiotee]", peer);
    return false;
  }

  unique_gst_element video_tee(gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), "videotee"));
  if (!video_tee) {
    error("Webrtc::remove_peer_from_pipeline:[%]:Couldn't retrieve tee [videotee]", peer);
    return false;
  }

  unique_gst_element tee_bin(GST_ELEMENT(gst_element_get_parent(audio_tee.get())));
  if (!tee_bin) {
    tee_bin.reset(GST_ELEMENT(gst_element_get_parent(video_tee.get())));
    if (!tee_bin) {
      error("Webrtc::remove_peer_from_pipeline:[%]:Failed to retrieve the bin containing the tees", peer);
      return false;
    }
  }

  unique_gst_element webrtc(gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), peer.c_str()));
  if (!webrtc) {
    error("Webrtc::remove_peer_from_pipeline:[%]:Failed to retrieve webrtcbin element for peer",
          peer);
    return false;
  }

  gst_bin_remove(GST_BIN(tee_bin.get()), webrtc.get());

  std::string audio_qname = "audio-queue-" + std::string(peer);
  std::unique_ptr<GstElement, decltype(&gst_object_unref)> audio_queue(
      gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), audio_qname.c_str()), gst_object_unref);
  if (!audio_queue) {
    error("Webrtc::remove_peer_from_pipeline:[%]:Failed to retrieve audio queue", peer);
    return false;
  }

  unique_gst<GstPad> audio_sinkpad(gst_element_get_static_pad(audio_queue.get(), "sink"));
  if (!audio_sinkpad) {
    error("Webrtc::remove_peer_from_pipeline:[%]:Failed to retrieve static pad 'sink' from queue",
          peer);
    return false;
  }

  unique_gst<GstPad> audio_srcpad(gst_pad_get_peer(audio_sinkpad.get()));
  if (!audio_srcpad) {
    error(
        "Webrtc::remove_peer_from_pipeline:[%]:Failed to retrieve corresponding sink for static "
        "sink pad",
        peer);
    return false;
  }

  gst_element_set_state(GST_ELEMENT(audio_queue.get()), GST_STATE_NULL);
  gst_element_release_request_pad(audio_tee.get(), audio_srcpad.get());

  gst_bin_remove(GST_BIN(tee_bin.get()), audio_queue.get());

  std::string video_qname = "video-queue-" + std::string(peer);
  std::unique_ptr<GstElement, decltype(&gst_object_unref)> video_queue(
      gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), video_qname.c_str()), gst_object_unref);
  if (!video_queue) {
    error("Webrtc::remove_peer_from_pipeline:[%]:Failed to retrieve video queue", peer);
    return false;
  }

  unique_gst<GstPad> video_sinkpad(gst_element_get_static_pad(video_queue.get(), "sink"));
  if (!video_sinkpad) {
    error("Webrtc::remove_peer_from_pipeline:[%]:Failed to retrieve static pad 'sink' from queue",
          peer);
    return false;
  }

  unique_gst<GstPad> video_srcpad(gst_pad_get_peer(video_sinkpad.get()));
  if (!video_srcpad) {
    error(
        "Webrtc::remove_peer_from_pipeline:[%]:Failed to retrieve corresponding sink for static "
        "sink pad",
        peer);
    return false;
  }

  gst_element_set_state(GST_ELEMENT(video_queue.get()), GST_STATE_NULL);
  gst_element_release_request_pad(video_tee.get(), video_srcpad.get());

  gst_bin_remove(GST_BIN(tee_bin.get()), video_queue.get());

  auto&& [handlers, data] = peers_[peer];

  auto [first, second] = handlers;
  if (first) g_signal_handler_disconnect(webrtc.get(), first);
  if (second) g_signal_handler_disconnect(webrtc.get(), second);

  peers_.erase(peer);

  return true;
}

bool Webrtc::add_peer_to_pipeline(const std::string& peer) {
  debug("Webrtc::add_peer_to_pipeline");

  bool success = false;
  On_scope_exit {
    // remove peer from pipeline if we exit before the end
    if (!success) {
      remove_peer_from_pipeline(peer);
    }
  };

  if (peer_registered(peer)) {
    debug("Webrtc::add_peer_to_pipeline::[%] is already registered", peer);
    return true;
  }

  std::string audio_qname = "audio-queue-" + std::string(peer);
  GstElement* audio_queue = gst_element_factory_make("queue", audio_qname.c_str());  // observer_ptr

  std::string video_qname = "video-queue-" + std::string(peer);
  GstElement* video_queue = gst_element_factory_make("queue", video_qname.c_str());  // observer_ptr

  GstElement* webrtc = gst_element_factory_make("webrtcbin", peer.c_str());  // observer_ptr

  unique_gst<GstElement> audio_tee(
      gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), "audiotee"));
  if (!audio_tee) {
    error("Webrtc::add_peer_to_pipeline::Couldn't get tee: [audiotee] from pipeline");
    return false;
  }

  unique_gst<GstElement> video_tee(
      gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), "videotee"));
  if (!video_tee) {
    error("Webrtc::add_peer_to_pipeline::Couldn't get tee: [videotee] from pipeline");
    return false;
  }

  unique_gst<GstElement> tee_bin(GST_ELEMENT(gst_element_get_parent(video_tee.get())));
  if (!tee_bin) {
    tee_bin.reset(GST_ELEMENT(gst_element_get_parent(video_tee.get())));
    if (!tee_bin) {
      error("Webrtc::add_peer_to_pipeline::Couldn't get bin container the a/v tees");
      return false;
    }
  }

  // ---------------    -------------
  // | audio queue |--->|           |
  // ===============    | webrtcbin |
  // | video queue |--->|           |
  // ---------------    -------------
  gst_bin_add_many(GST_BIN(tee_bin.get()), audio_queue, video_queue, webrtc, nullptr);
  {
    unique_gst<GstPad> audio_srcpad(gst_element_get_static_pad(audio_queue, "src"));
    if (!audio_srcpad) {
      error("Webrtc::add_peer_to_pipeline::Couldn't get 'src' from queue [%]", audio_qname);
      return false;
    }

    unique_gst<GstPad> audio_sinkpad(gst_element_get_request_pad(webrtc, "sink_%u"));
    if (!audio_sinkpad) {
      error("Webrtc::add_peer_to_pipeline:[%]:Couldn't get 'sink_%u' pad from peer's webrtcbin", peer);
      return false;
    }

    if (gst_pad_link(audio_srcpad.get(), audio_sinkpad.get()) != GST_PAD_LINK_OK) {
      error("Webrtc::add_peer_to_pipeline::Failed to link audio pads for peer [%]", peer);
      return false;
    }

    unique_gst<GstPad> video_srcpad(gst_element_get_static_pad(video_queue, "src"));
    if (!video_srcpad) {
      error("Webrtc::add_peer_to_pipeline::Couldn't get 'src' from queue [%]", video_qname);
      return false;
    }

    unique_gst<GstPad> video_sinkpad(gst_element_get_request_pad(webrtc, "sink_%u"));
    if (!video_sinkpad) {
      error("Webrtc::add_peer_to_pipeline:[%]:Couldn't get 'sink_%u' pad from peer's webrtcbin", peer);
      return false;
    }

    if (gst_pad_link(video_srcpad.get(), video_sinkpad.get()) != GST_PAD_LINK_OK) {
      error("Webrtc::add_peer_to_pipeline::Failed to link video pads for peer [%]", peer);
      return false;
    }
  }

  {
    // -------------    ---------------
    // | audio tee |--->| audio queue |
    // -------------    ===============
    // | video tee |--->| video queue |
    // -------------    ---------------
    unique_gst<GstPad> audio_srcpad(gst_element_get_request_pad(audio_tee.get(), "src_%u"));
    if (!audio_srcpad) {
      error("Webrtc::add_peer_to_pipeline::Failed to retrieve 'src_%u' pad from [audiotee]");
      return false;
    }

    unique_gst<GstPad> audio_sinkpad(gst_element_get_static_pad(audio_queue, "sink"));
    if (!audio_sinkpad) {
      error("Webrtc::add_peer_to_pipeline::Failed to retrieve 'sink' pad from queue [%]",
            audio_qname);
      return false;
    }

    if (gst_pad_link(audio_srcpad.get(), audio_sinkpad.get()) != GST_PAD_LINK_OK) {
      error("Webrtc::add_peer_to_pipeline::Failed to link audio pads for peer [%]", peer);
      return false;
    }

    unique_gst<GstPad> video_srcpad(gst_element_get_request_pad(video_tee.get(), "src_%u"));
    if (!video_srcpad) {
      error("Webrtc::add_peer_to_pipeline::Failed to retrieve 'src_%u' pad from [videotee]");
      return false;
    }

    unique_gst<GstPad> video_sinkpad(gst_element_get_static_pad(video_queue, "sink"));
    if (!video_sinkpad) {
      error("Webrtc::add_peer_to_pipeline::Failed to retrieve 'sink' pad from queue [%]",
            video_qname);
      return false;
    }

    if (gst_pad_link(video_srcpad.get(), video_sinkpad.get()) != GST_PAD_LINK_OK) {
      error("Webrtc::add_peer_to_pipeline::Failed to link video pads for peer [%]", peer);
      return false;
    }
  }

  auto data = unique_bundle_p(new bundle_t(this, peer, nullptr));
  auto ice_signal_id = g_signal_connect(
      webrtc, "on-ice-candidate", G_CALLBACK(on_ice_candidate_collected), (gpointer)data.get());
  if (ice_signal_id <= 0) {
    error(
        "Webrtc::add_peer_to_pipeline::Couldn't subscribe to 'on-ice-candidate' signal on "
        "webrtcbin for peer [%]",
        peer);
    return false;
  }

  auto pad_signal_id =
      g_signal_connect(webrtc, "pad-added", G_CALLBACK(on_pad_added), (gpointer)data.get());
  if (pad_signal_id <= 0) {
    error(
        "Webrtc::add_peer_to_pipeline::Couldn't subscribe to 'pad-added' signal on webrtcbin for "
        "peer [%]",
        peer);
    g_signal_handler_disconnect(webrtc, ice_signal_id);
    return false;
  }

  peers_[peer] = peer_data_t(handler_tuple_t(ice_signal_id, pad_signal_id), std::move(data));

  if (!(gst_element_sync_state_with_parent(audio_queue) &&
        gst_element_sync_state_with_parent(video_queue) &&
        gst_element_sync_state_with_parent(webrtc))) {
    error("Webrtc::add_peer_to_pipeline::Failed to sync new elements' states with their parent");
    return false;
  }

  success = true;

  return true;
}

void Webrtc::on_ice_candidate_collected(GstElement* webrtc G_GNUC_UNUSED,
                                        const guint mlineindex,
                                        const gchar* candidate,
                                        bundle_t* data) {
  auto [self, peer, description] = *data;
  self->send_ice(peer, mlineindex, std::string(candidate));
}

void Webrtc::on_pad_added(GstElement* webrtc G_GNUC_UNUSED, GstPad* pad, bundle_t* data) {
  auto self = std::get<Webrtc*>(*data);
  self->pad_added(pad, data);
}

void Webrtc::pad_added(GstPad* pad, bundle_t* data) {
  debug("Webrtc::pad_added");

  if (stopping_) {
    warning("Webrtc::pad_added::Stop task launched. Ignoring.");
    return;
  }

  if (GST_PAD_DIRECTION(pad) != GST_PAD_SRC) {
    error("Webrtc::pad_added::Pad doesn't have direction GST_PAD_SRC");
    return;
  }

  GstElement* decodebin = gst_element_factory_make("decodebin", nullptr);  // observer_ptr
  g_signal_connect(decodebin, "pad-added", G_CALLBACK(on_decodebin_pad_added), data);

  unique_gst<GstElement> tee(gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), "videotee"));
  if (!tee) {
    error("Webrtc::pad_added:[%]:Failed to retrieve [videotee] from pipeline", GST_PAD_NAME(pad));
    return;
  }

  unique_gst<GstElement> tee_parent(GST_ELEMENT(gst_element_get_parent(tee.get())));
  if (!tee_parent) {
    error("Webrtc::pad_added:[%]:Failed to retrieve [videotee]'s parent", GST_PAD_NAME(pad));
    return;
  }

  gst_bin_add(GST_BIN(tee_parent.get()), decodebin);
  gst_element_sync_state_with_parent(decodebin);

  unique_gst<GstPad> sinkpad(gst_element_get_static_pad(decodebin, "sink"));
  gst_pad_link(pad, sinkpad.get());
}

void Webrtc::on_decodebin_pad_added(GstElement* decodebin G_GNUC_UNUSED,
                                    GstPad* pad,
                                    bundle_t* data) {
  auto self = std::get<Webrtc*>(*data);
  self->decodebin_pad_added(pad, data);
}

void Webrtc::decodebin_pad_added(GstPad* pad, bundle_t* data) {
  debug("Webrtc::pad_added");

  if (stopping_) {
    warning("Webrtc::decodebin_pad_added::Stop task launched. Ignoring.");
    return;
  }

  auto peer = std::get<const std::string>(*data);

  if (!gst_pad_has_current_caps(pad)) {
    error("Webrtc::decodebin_pad_added:[%]:Ignoring pad [%] with no caps", peer, GST_PAD_NAME(pad));
    return;
  }

  unique_gst<GstCaps> caps(gst_pad_get_current_caps(pad));
  const gchar* name = gst_structure_get_name(gst_caps_get_structure(caps.get(), 0));

  unique_gst_element tee(gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), "videotee"));
  if (!tee) {
    error("Webrtc::decodebin_pad_added:[%]:Failed to retrieve [videotee] from pipeline", peer);
    return;
  }

  unique_gst_element tee_parent(GST_ELEMENT(gst_element_get_parent(tee.get())));
  if (!tee_parent) {
    error("Webrtc::decodebin_pad_added:[%]:Failed to retrieve [videotee]'s parent", peer);
    return;
  }

  GstElement* converter;
  std::string suffix;
  if (stringutils::starts_with(name, "video")) {
    converter = gst_element_factory_make("videoconvert", nullptr);
    suffix = "webrtc-video";
  } else if (stringutils::starts_with(name, "audio")) {
    converter = gst_element_factory_make("audioconvert", nullptr);
    suffix = "webrtc-audio";
  } else {
    error("Webrtc::decodebin_pad_added::Ignoring unsupported pad [%]", GST_PAD_NAME(pad));
    return;
  }

  if (!converter) {
    error("Webrtc::decodebin_pad_added:[%]:Failed to create converter element", peer);
    return;
  }

  gst::UGstElem sink("shmdatasink");
  if (!sink.get_raw()) {
    error("Webrtc::decodebin_pad_added:[%]:Failed to create sink element", peer);
    return;
  }

  GstElement* pqueue = gst_element_factory_make("queue", nullptr);
  if (!pqueue) {
    error("Webrtc::decodebin_pad_added:[%]:Failed to make queue element", peer);
    return;
  }

  g_object_set(G_OBJECT(sink.get_raw()),
               "socket-path",
               claw_.get_shmpath_from_writer_label(suffix).c_str(),
               nullptr);
  auto extra_caps = get_quiddity_caps();
  g_object_set(G_OBJECT(sink.get_raw()), "extra-caps-properties", extra_caps.c_str(), nullptr);

  gst_bin_add_many(GST_BIN(tee_parent.get()), pqueue, converter, sink.get_raw(), nullptr);

  gst_element_sync_state_with_parent(pqueue);
  gst_element_sync_state_with_parent(converter);
  gst_element_sync_state_with_parent(sink.get_raw());

  gst_element_link_many(pqueue, converter, sink.get_raw(), nullptr);

  GstPad* qpad = gst_element_get_static_pad(pqueue, "sink");

  if (gst_pad_link(pad, qpad) != GST_PAD_LINK_OK) {
    error("Webrtc::pad_added:[%]:Failed to link pad to the queue's sink", peer);
    return;
  }
}

void Webrtc::on_offer_created(GstPromise* promise, bundle_t* data) {
  auto self = std::get<Webrtc*>(*data);
  self->offer_created(promise, data);
}

void Webrtc::offer_created(GstPromise* promise, bundle_t* data) {
  debug("Webrtc::offer_created");

  if (stopping_) {
    warning("Webrtc::offer_created::Stop task launched. Ignoring.");
    return;
  }

  if (gst_promise_wait(promise) != GST_PROMISE_RESULT_REPLIED) {
    error("Webrtc::offer_created::No result reply set on promise");
    return;
  }

  const GstStructure* reply = gst_promise_get_reply(promise);  // observer_ptr

  GstWebRTCSessionDescription* sdp;
  gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &sdp, nullptr);
  std::get<GstWebRTCSessionDescription*>(*data) = sdp;

  auto peer = std::get<const std::string>(*data);
  unique_gst<GstElement> webrtc(
      gst_bin_get_by_name(GST_BIN(pipeline_->get_pipeline()), peer.c_str()));
  if (!webrtc) {
    error("Webrtc::offer_created::Couldn't get webrtcbin for peer [%]", peer);
    return;
  }

  gst_promise_unref(promise);
  promise = gst_promise_new_with_change_func(
      (GstPromiseChangeFunc)on_local_description_set, (gpointer)data, nullptr);
  debug("Webrtc::offer_created::Setting local description[%]", peer);

  g_signal_emit_by_name(webrtc.get(), "set-local-description", sdp, promise);
}

void Webrtc::on_local_description_set(GstPromise* promise, bundle_t* data) {
  auto self = std::get<Webrtc*>(*data);
  self->local_description_set(promise, unique_bundle_p(data));
}

void Webrtc::local_description_set(GstPromise* promise, unique_bundle_p data) {
  debug("Webrtc::local_description_set");
  gst_promise_unref(promise);

  auto [self, peer, sdp] = *data;
  if (!send_sdp(peer, sdp)) {
    error("Webrtc::local_description_set::Failed to send sdp message to peer [%]", peer);
    return;
  }
}

bool Webrtc::send_sdp(const std::string& peer, GstWebRTCSessionDescription* description) {
  debug("Webrtc::send_sdp");

  const gchar* type;
  if (description->type == GST_WEBRTC_SDP_TYPE_OFFER) {
    type = "offer";
  } else if (description->type == GST_WEBRTC_SDP_TYPE_ANSWER) {
    type = "answer";
  } else {
    error("Webrtc::send_sdp::Ignoring unsupported sdp type [%]",
          gst_webrtc_sdp_type_to_string(description->type));
    return false;
  }

  debug("Webrtc::send_sdp::type::[%]", type);

  std::string text(gst_sdp_message_as_text(description->sdp));

  unique_json_object msg(json_object_new());

  unique_json_object sdp(json_object_new());
  json_object_set_string_member(sdp.get(), "type", type);
  json_object_set_string_member(sdp.get(), "sdp", text.c_str());

  json_object_set_object_member(msg.get(), "sdp", sdp.release());

  return send_peer_msg(peer, json_to_string(msg.get()).c_str());
}

bool Webrtc::send_ice(const std::string& peer, guint mlineindex, const std::string& candidate) {
  debug("Webrtc::send_ice::[%]", peer);
  unique_json_object ice(json_object_new());

  json_object_set_string_member(ice.get(), "candidate", g_strdup(candidate.c_str()));
  json_object_set_int_member(ice.get(), "sdpMLineIndex", mlineindex);

  unique_json_object msg(json_object_new());
  json_object_set_object_member(msg.get(), "ice", ice.release());

  return send_peer_msg(peer, json_to_string(msg.get()).c_str());
}

bool Webrtc::send_command(const std::string& peer,
                          const std::string& type,
                          const std::string& command) {
  unique_json_object command_obj(json_object_new());
  json_object_set_string_member(command_obj.get(), "type", type.c_str());
  json_object_set_string_member(command_obj.get(), "command", command.c_str());

  unique_json_object message(json_object_new());
  json_object_set_object_member(message.get(), "command", command_obj.get());

  return send_peer_msg(peer, json_to_string(message.get()));
}

bool Webrtc::send_peer_msg(const std::string& peer, const std::string& text) {
  debug("Webrtc::send_peer_msg");

  std::string msg = "ROOM_PEER_MSG " + std::string(peer) + " " + std::string(text);

  return send_text(msg);
}

bool Webrtc::send_text(const std::string& text) {
  debug("Webrtc::send_text");

  if (stopping_) {
    warning("Webrtc::send_text::Stop task launched. Ignoring.");
    return false;
  }

  debug("Webrtc::send_text::[%]", text);
  
  soup_websocket_connection_send_text(connection_.get(), g_strdup(text.c_str()));
  return true;
}

std::string Webrtc::json_to_string(JsonObject* object) {
  std::unique_ptr<JsonNode, decltype(&json_node_free)> root(
      json_node_init_object(json_node_alloc(), object), json_node_free);
  unique_gobject<JsonGenerator> generator(json_generator_new());
  json_generator_set_root(generator.get(), root.get());
  return json_generator_to_data(generator.get(), nullptr);
}

bool Webrtc::on_shmdata_connect(const std::string& shmpath, claw::sfid_t sfid) {
  debug("Webrtc::on_shmdata_connect");

  auto label = claw_.get_follower_label(sfid);

  if ("video" == label) {
    video_shmpath_ = shmpath;
    // video_shmsub_.reset();
    // video_shmsub_ = std::make_unique<shmdata::Follower>(
    //     this,
    //     video_shmpath_,
    //     nullptr,
    //     nullptr,
    //     //[this](const std::string& caps) {
    //     //  if (!video_caps_.empty() && video_caps_ != caps) {
    //     //    video_caps_ = caps;
    //     //    async_this_.run_async([this]() { on_shmdata_connect(video_caps_, ShmType::Video);
    //     });
    //     //    return;
    //     //  }
    //     //  video_caps_ = caps;
    //     //},
    //     nullptr,
    //     shmdata::Stat::kDefaultUpdateInterval,
    //     shmdata::Follower::Direction::reader,
    //     true);
  } else if ("audio" == label) {
    audio_shmpath_ = shmpath;
    // audio_shmsub_.reset();
    // audio_shmsub_ = std::make_unique<shmdata::Follower>(
    //     this,
    //     audio_shmpath_,
    //     nullptr,
    //     nullptr,
    //     //[this](const std::string& caps) {
    //     //  if (!audio_caps_.empty() && audio_caps_ != caps) {
    //     //    audio_caps_ = caps;
    //     //    async_this_.run_async([this]() { on_shmdata_connect(audio_caps_, ShmType::Audio);
    //     });
    //     //    return;
    //     //  }
    //     //  audio_caps_ = caps;
    //     //},
    //     nullptr,
    //     shmdata::Stat::kDefaultUpdateInterval,
    //     shmdata::Follower::Direction::reader,
    //     true);
  } else {
    return false;
  }

  return true;
}

bool Webrtc::on_shmdata_disconnect(claw::sfid_t sfid) {
  debug("Webrtc::on_shmdata_disconnect");
  auto label = claw_.get_follower_label(sfid);
  if ("video" == label) {
    video_shmsub_.reset();
    video_shmpath_.clear();
  } else if ("audio" == label) {
    audio_shmsub_.reset();
    audio_shmpath_.clear();
  } else {
    return false;
  }
  return true;
}

}  // namespace quiddities
}  // namespace switcher
