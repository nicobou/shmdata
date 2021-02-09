/*
 * This file is part of switcher-rtmp.
 *
 * switcher-rtmp is free software; you can redistribute it and/or
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

#include "./rtmp.hpp"

namespace switcher {
namespace quiddities {

SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    RTMP,
    "rtmp",
    "RTMP broadcaster",
    "audio/video",
    "reader",
    "Plugin for streaming audio/video to an RTMP server (Youtube, Twitch, etc.)",
    "LGPL",
    "Jérémie Soria/Francis Lecavalier");

RTMP::RTMP(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)),
      Startable(this),
      shmcntr_(static_cast<Quiddity*>(this)),
      stream_app_url_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "stream_app_url",
          [this](const std::string& val) {
            stream_app_url_ = val;
            return true;
          },
          [this]() { return stream_app_url_; },
          "Stream application URL",
          "Address of the RTMP server used to stream",
          stream_app_url_)),
      stream_key_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "stream_key",
          [this](const std::string& val) {
            stream_key_ = val;
            return true;
          },
          [this]() { return stream_key_; },
          "Stream key",
          "Key to access the RTMP server",
          stream_key_)),
      gst_pipeline_(std::make_unique<gst::Pipeliner>(nullptr, nullptr)) {
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return on_shmdata_connect(shmpath); },
      [this](const std::string& shmpath) { return on_shmdata_disconnect(shmpath); },
      [this]() { return on_shmdata_disconnect_all(); },
      [this](const std::string& caps) { return can_sink_caps(caps); },
      2);
}

RTMP::~RTMP() { stop(); }

bool RTMP::create_gst_pipeline() {
  if (!gst_pipeline_) gst_pipeline_ = std::make_unique<gst::Pipeliner>(nullptr, nullptr);

  std::string description =
      "flvmux streamable=true name=mux ! queue ! rtmpsink name=rtmpsink sync=false ";
  description +=
      "shmdatasrc socket-path=/tmp/fake name=shmvideo copy-buffers=true do-timestamp=true ! "
      "h264parse ! queue ! mux. ";
  description +=
      "shmdatasrc socket-path=/tmp/fake name=shmaudio copy-buffers=true do-timestamp=true ! "
      "audioconvert ! audioresample ! queue ! voaacenc bitrate=256000 ! queue ! mux.";

  GError* gerror = nullptr;
  auto bin = gst_parse_bin_from_description(description.c_str(), FALSE, &gerror);
  if (gerror != nullptr) {
    error("Error while creating Gst pipeline for RTMP: %", std::string(gerror->message));
    g_error_free(gerror);
    return false;
  }

  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  g_object_set(G_OBJECT(bin), "async-handling", TRUE, nullptr);

  auto shmdatavideo = gst_bin_get_by_name(GST_BIN(bin), "shmvideo");
  auto shmdataaudio = gst_bin_get_by_name(GST_BIN(bin), "shmaudio");
  auto rtmpsink = gst_bin_get_by_name(GST_BIN(bin), "rtmpsink");
  g_object_set(G_OBJECT(shmdatavideo), "socket-path", video_shmpath_.c_str(), nullptr);
  g_object_set(G_OBJECT(shmdataaudio), "socket-path", audio_shmpath_.c_str(), nullptr);
  g_object_set(
      G_OBJECT(rtmpsink), "location", (stream_app_url_ + "/" + stream_key_).c_str(), nullptr);

  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), bin);

  return true;
}

bool RTMP::start() {
  if (this->is_started()) {
    error("RTMP is already started.");
    return false;
  }
  if (video_shmpath_.empty()) {
    error("RTMP requires a video shmdata connection.");
    return false;
  }
  if (audio_shmpath_.empty()) {
    error("RTMP requires an audio shmdata connection.");
    return false;
  }
  if (stream_app_url_.empty()) {
    error("stream_app_url must not be empty");
    return false;
  }
  if (stream_key_.empty()) {
    error("stream_key must not be empty");
    return false;
  }

  if (!create_gst_pipeline()) {
    return false;
  }

  gst_pipeline_->play(true);

  pmanage<MPtr(&property::PBag::disable)>(stream_app_url_id_, Startable::disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(stream_key_id_, Startable::disabledWhenStartedMsg);

  return true;
}

bool RTMP::stop() {
  if (this->is_started()) {
    gst_pipeline_->play(false);
    gst_pipeline_.reset();
    pmanage<MPtr(&property::PBag::enable)>(stream_app_url_id_);
    pmanage<MPtr(&property::PBag::enable)>(stream_key_id_);
  }
  return true;
}

bool RTMP::on_shmdata_connect(const std::string& shmpath) {
  if (stringutils::ends_with(shmpath, "video-encoded")) {
    if (!video_shmpath_.empty()) follower_video_.reset();
    video_shmpath_ = shmpath;
    follower_video_ = std::make_unique<shmdata::Follower>(this,
                                                          video_shmpath_,
                                                          nullptr,
                                                          nullptr,
                                                          nullptr,
                                                          shmdata::Stat::kDefaultUpdateInterval,
                                                          shmdata::Follower::Direction::reader,
                                                          true);
  } else if (stringutils::ends_with(shmpath, "audio")) {
    if (!audio_shmpath_.empty()) follower_audio_.reset();
    audio_shmpath_ = shmpath;
    follower_audio_ = std::make_unique<shmdata::Follower>(this,
                                                          audio_shmpath_,
                                                          nullptr,
                                                          nullptr,
                                                          nullptr,
                                                          shmdata::Stat::kDefaultUpdateInterval,
                                                          shmdata::Follower::Direction::reader,
                                                          true);
  } else {
    warning("RTMP is only compatible with shmdatas containing H264-encoded video or raw audio.");
    return false;
  }

  if (this->is_started()) {
    stop();
    return start();
  }

  return true;
}

bool RTMP::on_shmdata_disconnect(const std::string& shmpath) {
  if (shmpath == video_shmpath_) {
    follower_video_.reset();
    video_shmpath_.clear();
  } else if (shmpath == audio_shmpath_) {
    follower_audio_.reset();
    audio_shmpath_.clear();
  } else {
    return false;
  }

  pmanage<MPtr(&property::PBag::set_str_str)>("started", "false");
  return true;
}

bool RTMP::on_shmdata_disconnect_all() {
  follower_video_.reset();
  follower_audio_.reset();
  video_shmpath_.clear();
  audio_shmpath_.clear();
  pmanage<MPtr(&property::PBag::set_str_str)>("started", "false");
  return true;
}

bool RTMP::can_sink_caps(const std::string& str_caps) {
  return stringutils::starts_with(str_caps, "audio/x-raw") ||
         stringutils::starts_with(str_caps, "video/x-h264");
}

}  // namespace quiddities
}  // namespace switcher
