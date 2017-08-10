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
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_DECLARE_PLUGIN(RTMP);
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(RTMP,
                                     "rtmp",
                                     "RTMP plugin",
                                     "audio/video",
                                     "reader",
                                     "RTMP plugin for streaming audio/video to Youtube/Twitch/...",
                                     "LGPL",
                                     "Jérémie Soria");

RTMP::RTMP(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      shmcntr_(static_cast<Quiddity*>(this)),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)) {
  stream_app_url_id_ = pmanage<MPtr(&PContainer::make_string)>(
      "stream_app_url",
      [this](const std::string& val) {
        stream_app_url_ = val;
        if (!stream_key_.empty() && (!audio_shmpath_.empty() || !video_shmpath_.empty()))
          return create_gst_pipeline();
        return true;
      },
      [this]() { return stream_app_url_; },
      "Stream application URL",
      "RTMP address used to stream.",
      stream_app_url_);
  stream_key_id_ = pmanage<MPtr(&PContainer::make_string)>(
      "stream_key",
      [this](const std::string& val) {
        stream_key_ = val;
        if (!stream_app_url_.empty() && (!audio_shmpath_.empty() || !video_shmpath_.empty()))
          return create_gst_pipeline();
        return true;
      },
      [this]() { return stream_key_; },
      "Stream key",
      "Stream application-specific stream key needed to link to application account.",
      stream_key_);

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) {
        if (StringUtils::ends_with(shmpath, "video-encoded"))
          return on_shmdata_connect(shmpath, ShmType::VIDEO);
        else
          return on_shmdata_connect(shmpath, ShmType::AUDIO);
      },
      [this](const std::string& shmpath) {
        if (StringUtils::ends_with(shmpath, "video-encoded"))
          return on_shmdata_disconnect(ShmType::VIDEO);
        else
          return on_shmdata_disconnect(ShmType::AUDIO);
      },
      nullptr,
      [this](const std::string& caps) { return can_sink_caps(caps); },
      std::numeric_limits<unsigned int>::max());

}

bool RTMP::create_gst_pipeline() {
  shmaudio_sub_.reset();
  shmvideo_sub_.reset();
  gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);

  std::string dest = "rtmpsink";
  if (audio_shmpath_.empty() || video_shmpath_.empty()) {
    warning("Could not send stream because no video or audio is connected (rtmp).");
    dest = "fakesink";
  }

  if (stream_app_url_.empty() || stream_key_.empty()) {
    warning("Could not send stream because stream application URL or key is empty (rtmp).");
    dest = "fakesink";
  }

  std::string description =
      "flvmux streamable=true name=mux ! queue ! " + dest + " name=rtmpsink sync=false ";

  description +=
      "shmdatasrc socket-path=/tmp/fake name=shmvideo copy-buffers=true do-timestamp=true ! ";
  description += "h264parse ! queue ! mux. ";

  description +=
      "shmdatasrc socket-path=/tmp/fake name=shmaudio copy-buffers=true do-timestamp=true ! ";
  description += "audioconvert ! audioresample ! queue ! voaacenc bitrate=256000 ! queue ! mux.";

  GError* error = nullptr;
  auto bin = gst_parse_bin_from_description(description.c_str(), FALSE, &error);

  if (error) {
    warning("Failed to create GstBin from pipeline description (rtmp): %",
            std::string(error->message));
    return false;
  }

  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  g_object_set(G_OBJECT(bin), "async-handling", TRUE, nullptr);

  if (!video_shmpath_.empty()) {
    auto shmdatavideo = gst_bin_get_by_name(GST_BIN(bin), "shmvideo");
    g_object_set(G_OBJECT(shmdatavideo), "socket-path", video_shmpath_.c_str(), nullptr);

    shmvideo_sub_ = std::make_unique<GstShmdataSubscriber>(
        shmdatavideo,
        [this](const std::string& caps) {
          graft_tree(
              ".shmdata.reader." + video_shmpath_,
              ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
        },
        ShmdataStat::make_tree_updater(this, ".shmdata.reader." + video_shmpath_),
        [this]() { prune_tree(".shmdata.reader." + video_shmpath_); });
  }

  if (!audio_shmpath_.empty()) {
    auto shmdataaudio = gst_bin_get_by_name(GST_BIN(bin), "shmaudio");
    g_object_set(G_OBJECT(shmdataaudio), "socket-path", audio_shmpath_.c_str(), nullptr);
    shmaudio_sub_ = std::make_unique<GstShmdataSubscriber>(
        shmdataaudio,
        [this](const std::string& str_caps) {
          graft_tree(".shmdata.reader." + audio_shmpath_,
                     ShmdataUtils::make_tree(
                         str_caps, ShmdataUtils::get_category(str_caps), ShmdataStat()));
        },
        ShmdataStat::make_tree_updater(this, ".shmdata.reader." + audio_shmpath_),
        [this]() { prune_tree(".shmdata.reader." + audio_shmpath_); });
  }

  if (!audio_shmpath_.empty() && !video_shmpath_.empty() && !stream_app_url_.empty() &&
      !stream_key_.empty()) {
    auto rtmpsink = gst_bin_get_by_name(GST_BIN(bin), "rtmpsink");
    g_object_set(
        G_OBJECT(rtmpsink), "location", (stream_app_url_ + "/" + stream_key_).c_str(), nullptr);
  }

  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), bin);
  gst_pipeline_->play(true);

  return true;
}

bool RTMP::on_shmdata_connect(const std::string& shmpath, ShmType type) {
  switch (type) {
    case ShmType::AUDIO:
      if (!audio_shmpath_.empty()) shmaudio_sub_.reset();  // We disconnect any existing audio
      audio_shmpath_ = shmpath;
      break;
    case ShmType::VIDEO:
      if (!video_shmpath_.empty()) shmvideo_sub_.reset();  // We disconnect any existing video
      video_shmpath_ = shmpath;
      break;
  }

  // We create it but if video and audio were both disconnected the pipeline will just be reset.
  create_gst_pipeline();
  return true;
}

bool RTMP::on_shmdata_disconnect(ShmType type) {
  switch (type) {
    case ShmType::AUDIO:
      audio_shmpath_ = "";
      break;
    case ShmType::VIDEO:
      video_shmpath_ = "";
      break;
  }

  create_gst_pipeline();
  return true;
}

bool RTMP::can_sink_caps(std::string str_caps) {
  return StringUtils::starts_with(str_caps, "audio/x-raw") ||
         StringUtils::starts_with(str_caps, "video/x-h264");
}
};
