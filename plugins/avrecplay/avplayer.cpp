/*
 * This file is part of switcher-avrecplay.
 *
 * switcher-avrecplay is free software; you can redistribute it and/or
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

#include "avplayer.hpp"

#include <sys/stat.h>
#include "switcher/scope-exit.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_DECLARE_PLUGIN(AVPlayer);
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(AVPlayer,
                                     "avplayer",
                                     "Audio/video shmdata player",
                                     "audio/video",
                                     "writer",
                                     "Replays and controls a recorded shmdata audio/video file",
                                     "LGPL",
                                     "Jérémie Soria");

const std::string AVPlayer::kShmDestPath = "/tmp/avplayer/";

AVPlayer::AVPlayer(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      shmcntr_(static_cast<Quiddity*>(this)),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)) {
  pmanage<MPtr(&PContainer::make_string)>(
      "playpath",
      [this](const std::string& val) {
        if (val.empty()) {
          warning("Empty folder provided for shmdata recorder.");
          message("ERROR: Empty folder provided for shmdata recorder.");
          return false;
        }

        struct stat st;
        if (stat(val.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
          warning("The specified folder does not exist (avplayer).");
          message("ERROR: The specified folder does not exist (avplayer).");
          return false;
        }

        playpath_ = val;
        return true;
      },
      [this]() { return playpath_; },
      "Source folder",
      "Location of the folder from which the player will read recorded shmdata files",
      playpath_);

  pmanage<MPtr(&PContainer::make_bool)>("paused",
                                        [this](const bool& val) {
                                          pause_ = val;
                                          gst_pipeline_->play(pause_);
                                          return true;
                                        },
                                        [this]() { return false; },
                                        "Paused",
                                        "Toggle paused status of the stream",
                                        pause_);

  struct stat st;
  if (stat(AVPlayer::kShmDestPath.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
    if (-1 == mkdir(AVPlayer::kShmDestPath.c_str(), S_IRWXU | S_IRUSR | S_IWUSR)) {
      warning("The shmdata destination folder does not exist and could not be created (avplayer).");
      message(
          "ERROR: The shmdata destination folder does not exist and could not be created "
          "(avplayer).");
      is_valid_ = false;
      return;
    }
  }

  init_startable(this);
}

bool AVPlayer::start() {
  gst_pipeline_ = std::make_unique<GstPipeliner>(
      [this](GstMessage* msg) { return this->bus_async(msg); },
      /*[this](GstMessage* msg) { return this->bus_sync(msg); }*/ nullptr);
  std::vector<std::string> playlist;
  DIR* dirp = opendir(playpath_.c_str());
  struct dirent* dp;
  while ((dp = readdir(dirp)) != NULL) {
    playlist.push_back(std::string(dp->d_name));
  }
  closedir(dirp);

  // Create a pipeline to read all the files and write in a shmdatasink.
  std::string description;
  int i = 0;  // File index of written shmdata
  for (auto& file : playlist) {
    if (file == "." || file == "..") continue;
    auto to_play =
        std::make_unique<ShmFile>(AVPlayer::kShmDestPath + "avplayer" + std::to_string(i),
                                  playpath_ + "/" + file,
                                  "shmsink_" + file);

    description += std::string("filesrc") + (i == 0 ? " do-timestamp=true " : " ") + "location=" +
                   to_play->filepath_ + " ! decodebin ! shmdatasink name=" + to_play->sink_name_ +
                   " socket-path=" + to_play->shmpath_ + " ";
    files_list_.push_back(std::move(to_play));
    ++i;
  }

  GError* error = nullptr;
  avplay_bin_ = gst_parse_bin_from_description(description.c_str(), FALSE, &error);
  bool success = true;
  On_scope_exit {
    if (!success) gst_object_unref(avplay_bin_);
  };

  if (error) {
    warning("Could not create shmdata player: %", std::string(error->message));
    g_error_free(error);
    return false;
  }

  for (auto& file : files_list_) {
    file->sink_element_ = gst_bin_get_by_name(GST_BIN(avplay_bin_), file->sink_name_.c_str());
    if (!file->sink_element_) continue;

    file->shmsink_sub_ = std::make_unique<GstShmdataSubscriber>(
        file->sink_element_,
        [ this, shmpath = file->shmpath_ ](const std::string& caps) {
          graft_tree(
              ".shmdata.writer." + shmpath,
              ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
        },
        ShmdataStat::make_tree_updater(this, ".shmdata.writer." + file->shmpath_),
        [ this, shmpath = file->shmpath_ ]() { prune_tree(".shmdata.writer." + shmpath); });
  }

  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  g_object_set(G_OBJECT(avplay_bin_), "async-handling", TRUE, nullptr);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), avplay_bin_);
  gst_pipeline_->play(true);
  return true;
}
bool AVPlayer::stop() {
  position_task_.reset();
  gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);
  pmanage<MPtr(&PContainer::remove)>(position_id_);
  position_id_ = 0;
  track_duration_ = 0;
  pause_ = false;
  files_list_.clear();
  return true;
}

GstBusSyncReply AVPlayer::bus_async(GstMessage* msg) {
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_EOS: {
      th_->run_async([this]() { pmanage<MPtr(&PContainer::set_str_str)>("started", "false"); });
    }

    case GST_MESSAGE_STATE_CHANGED: {
      if (position_id_ == 0) {
        gst_element_query_duration(
            gst_pipeline_->get_pipeline(), GST_FORMAT_TIME, &track_duration_);

        if (track_duration_ != 0) {
          position_id_ =
              pmanage<MPtr(&PContainer::make_int)>("track_position",
                                                   [this](const int& pos) {
                                                     std::lock_guard<std::mutex> lock(seek_mutex_);
                                                     position_ = pos;
                                                     seek_called_ = true;
                                                     gst_pipeline_->play(false);
                                                     return true;
                                                   },
                                                   [this]() { return position_; },
                                                   "Track position",
                                                   "Current position of the track",
                                                   0,
                                                   0,
                                                   track_duration_ / GST_SECOND);
          position_task_ = std::make_unique<PeriodicTask<>>(
              [this]() {
                gint64 position;
                gst_element_query_position(
                    gst_pipeline_->get_pipeline(), GST_FORMAT_TIME, &position);
                position_ = static_cast<int>(position / GST_SECOND);
                pmanage<MPtr(&PContainer::notify)>(position_id_);

              },
              std::chrono::milliseconds(500));
        }
      }
      std::lock_guard<std::mutex> lock(seek_mutex_);
      if (seek_called_) {
        seek_called_ = false;
        gst_element_seek_simple(gst_pipeline_->get_pipeline(),
                                GST_FORMAT_TIME,
                                GST_SEEK_FLAG_FLUSH,
                                position_ * GST_SECOND);
        gst_pipeline_->play(true);
      }
      break;
    }
    default:
      break;
  }
  return GST_BUS_PASS;
}
}
