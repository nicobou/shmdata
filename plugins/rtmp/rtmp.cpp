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
    "Plugin for streaming audio/video to an RTMP server (Youtube, Twitch, etc.)",
    "LGPL",
    "Jérémie Soria/Francis Lecavalier");

const std::string RTMP::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "video-encoded",
      "description": "Video stream",
      "can_do": ["video/x-h264"]
    },
    {
      "label": "audio",
      "description": "Audio stream",
      "can_do": ["audio/x-raw"]
    }
}
)");

RTMP::RTMP(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf),
               {kConnectionSpec,
                [this](const std::string& shmpath, claw::sfid_t sfid) {
                  return on_shmdata_connect(shmpath, sfid);
                },
                [this](claw::sfid_t sfid) { return on_shmdata_disconnect(sfid); }}),
      Startable(this),
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
      gst_pipeline_(std::make_unique<gst::Pipeliner>(nullptr, nullptr)) {}

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
    LOGGER_ERROR(this->logger,
                 "Error while creating Gst pipeline for RTMP: {}",
                 std::string(gerror->message));
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
    LOGGER_ERROR(this->logger, "RTMP is already started.");
    return false;
  }
  if (video_shmpath_.empty()) {
    LOGGER_ERROR(this->logger, "RTMP requires a video shmdata connection.");
    return false;
  }
  if (audio_shmpath_.empty()) {
    LOGGER_ERROR(this->logger, "RTMP requires an audio shmdata connection.");
    return false;
  }
  if (stream_app_url_.empty()) {
    LOGGER_ERROR(this->logger, "stream_app_url must not be empty");
    return false;
  }
  if (stream_key_.empty()) {
    LOGGER_ERROR(this->logger, "stream_key must not be empty");
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

bool RTMP::on_shmdata_connect(const std::string& shmpath, claw::sfid_t sfid) {
  auto label = claw_.get_follower_label(sfid);
  if ("video-encoded" == label) {
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
  } else if ("audio" == label) {
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
    LOGGER_WARN(
        this->logger,
        "RTMP is only compatible with shmdatas containing H264-encoded video or raw audio.");
    return false;
  }

  if (this->is_started()) {
    stop();
    return start();
  }

  return true;
}

bool RTMP::on_shmdata_disconnect(claw::sfid_t sfid) {
  auto label = claw_.get_follower_label(sfid);
  if ("video-encoded" == label) {
    follower_video_.reset();
    video_shmpath_.clear();
  } else if ("audio" == label) {
    follower_audio_.reset();
    audio_shmpath_.clear();
  } else {
    return false;
  }

  pmanage<MPtr(&property::PBag::set_str_str)>("started", "false");
  return true;
}

}  // namespace quiddities
}  // namespace switcher
