/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#include "./timelapse.hpp"
#include <limits>
#include "switcher/file-utils.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Timelapse,
                                     "timelapse",
                                     "Timelapse",
                                     "video",
                                     "reader",
                                     "Make an image timelapse from raw video stream",
                                     "LGPL",
                                     "Nicolas Bouillot");

Timelapse::Timelapse(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      img_dir_id_(pmanage<MPtr(&PContainer::make_string)>(
          "imgdir",
          [this](const std::string& val) {
            img_dir_ = val;
            if (!img_dir_.empty() && img_dir_.back() != '/') img_dir_ += '/';
            auto file_prepared = FileUtils::prepare_writable_dir(val);
            if (!file_prepared.first) {
              warning("error preparing % directory for writing: %", val, file_prepared.second);
              return false;
            }
            updated_config_.store(true);
            return true;
          },
          [this]() { return img_dir_; },
          "Image Directory",
          "Directory where to store jpeg files to be produced. If empty, the "
          "path will be <video_shmdata_path>.jpg",
          img_dir_)),
      img_name_id_(pmanage<MPtr(&PContainer::make_string)>(
          "imgname",
          [this](const std::string& val) {
            img_name_ = val;
            updated_config_.store(true);
            return true;
          },
          [this]() { return img_name_; },
          "Image Name",
          "Name of the jpeg files to be produced. You can use printf format "
          "for numbering files (for instance %05d). If empty, the name will "
          "take the input shmdata name with option file number and jpg "
          "extension",
          img_name_)),
      num_files_id_(pmanage<MPtr(&PContainer::make_bool)>("num_files",
                                                          [this](const bool& num_files) {
                                                            num_files_ = num_files;
                                                            updated_config_.store(true);
                                                            return true;
                                                          },
                                                          [this]() { return num_files_; },
                                                          "Number Files",
                                                          "Automatically number produced files",
                                                          num_files_)),
      notify_last_file_id_(
          pmanage<MPtr(&PContainer::make_bool)>("notify_last_files",
                                                [this](const bool& notify) {
                                                  notify_last_file_ = notify;
                                                  return true;
                                                },
                                                [this]() { return notify_last_file_; },
                                                "Notify last file produced",
                                                "Update last file property with produced jpg file",
                                                notify_last_file_)),
      framerate_id_(
          pmanage<MPtr(&PContainer::make_fraction)>("framerate",
                                                    [this](const Fraction& val) {
                                                      framerate_ = val;
                                                      updated_config_.store(true);
                                                      return true;
                                                    },
                                                    [this]() { return framerate_; },
                                                    "Framerate",
                                                    "Number of image to be produced by seconds",
                                                    framerate_,
                                                    1,
                                                    1,  // min num/denom
                                                    60,
                                                    5)),  // max num/denom
      max_files_id_(pmanage<MPtr(&PContainer::make_unsigned_int)>(
          "maxfiles",
          [this](unsigned int val) {
            max_files_ = val;
            updated_config_.store(true);
            return true;
          },
          [this]() { return max_files_; },
          "Max files",
          "Maximum number of files simultaneously present on disk",
          max_files_,
          0,
          4294967295)),
      jpg_quality_id_(
          pmanage<MPtr(&PContainer::make_unsigned_int)>("quality",
                                                        [this](unsigned int val) {
                                                          jpg_quality_ = val;
                                                          updated_config_.store(true);
                                                          return true;
                                                        },
                                                        [this]() { return jpg_quality_; },
                                                        "JPEG quality",
                                                        "Quality of the produced jpeg image",
                                                        jpg_quality_,
                                                        0,
                                                        100)),
      last_image_id_(pmanage<MPtr(&PContainer::make_string)>("last_image",
                                                             nullptr,
                                                             [this]() { return last_image_; },
                                                             "Last image written",
                                                             "Path of the last jpeg file written",
                                                             last_image_)),
      width_id_(pmanage<MPtr(&PContainer::make_unsigned_int)>("width",
                                                              [this](unsigned int val) {
                                                                width_ = val;
                                                                updated_config_.store(true);
                                                                return true;
                                                              },
                                                              [this]() { return width_; },
                                                              "Width",
                                                              "Width of the scaled image",
                                                              width_,
                                                              0,
                                                              8192)),
      height_id_(pmanage<MPtr(&PContainer::make_unsigned_int)>("height",
                                                               [this](unsigned int val) {
                                                                 height_ = val;
                                                                 updated_config_.store(true);
                                                                 return true;
                                                               },
                                                               [this]() { return height_; },
                                                               "Height",
                                                               "Height of the scaled image",
                                                               height_,
                                                               0,
                                                               8192)),
      relaunch_task_(
          [this]() {
            if (updated_config_.exchange(false)) {
              std::unique_lock<std::mutex> lock(timelapse_mtx_);
              std::vector<std::string> shmdatas;
              for (auto& it : timelapse_) shmdatas.push_back(it.first);
              timelapse_.clear();
              for (auto& it : shmdatas) start_timelapse(it);
            }
          },
          std::chrono::milliseconds(200)),
      shmcntr_(static_cast<Quiddity*>(this)),
      timelapse_config_{std::string(), std::string()} {
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string& shmpath) { return this->on_shmdata_disconnect(shmpath); },
      [this]() { return this->on_shmdata_disconnect_all(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      std::numeric_limits<unsigned int>::max());
}

bool Timelapse::on_shmdata_disconnect(const std::string& shmpath) {
  std::unique_lock<std::mutex> lock(timelapse_mtx_);
  pmanage<MPtr(&PContainer::enable)>(img_name_id_);
  if (!stop_timelapse(shmpath)) return false;
  if (timelapse_.size() == 1) pmanage<MPtr(&PContainer::enable)>(img_name_id_);
  return true;
}

bool Timelapse::on_shmdata_disconnect_all() {
  std::unique_lock<std::mutex> lock(timelapse_mtx_);
  pmanage<MPtr(&PContainer::enable)>(img_name_id_);
  timelapse_.clear();
  return true;
}

bool Timelapse::on_shmdata_connect(const std::string& shmpath) {
  std::unique_lock<std::mutex> lock(timelapse_mtx_);
  if (timelapse_.size() == 1) {
    pmanage<MPtr(&PContainer::disable)>(img_name_id_, ShmdataConnector::disabledWhenConnectedMsg);
    img_name_.clear();
  }
  return start_timelapse(shmpath);
}

bool Timelapse::stop_timelapse(const std::string& shmpath) {
  auto timelapse = timelapse_.find(shmpath);
  if (timelapse_.end() == timelapse) return true;
  timelapse_.erase(timelapse);
  return true;
}

bool Timelapse::start_timelapse(const std::string& shmpath) {
  {
    auto timelapse = timelapse_.find(shmpath);
    if (timelapse_.end() != timelapse) timelapse_.erase(timelapse);
  }
  auto img_path = img_dir_;
  auto img_from_shmpath =
      img_dir_.empty() ? shmpath : shmpath.substr(shmpath.find_last_of("/") + 1);
  img_path += img_name_.empty() ? img_from_shmpath : img_name_;
  if (num_files_ && std::string::npos == img_path.find('%'))
    img_path += "_%d.jpg";
  else if (img_name_.empty())
    img_path += ".jpg";
  timelapse_config_ = GstVideoTimelapseConfig(shmpath, img_path);
  timelapse_config_.framerate_num_ = framerate_.numerator();
  timelapse_config_.framerate_denom_ = framerate_.denominator();
  timelapse_config_.width_ = width_;
  timelapse_config_.height_ = height_;
  timelapse_config_.jpg_quality_ = jpg_quality_;
  timelapse_config_.max_files_ = max_files_;
  auto new_timelapse = std::make_unique<GstVideoTimelapse>(
      timelapse_config_,
      [this, shmpath](const std::string& caps) {
        graft_tree(".shmdata.reader." + shmpath,
                   ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.reader." + shmpath),
      nullptr,
      [this, shmpath](std::string&& file_name) {
        if (!notify_last_file_) return;
        {
          auto lock = pmanage<MPtr(&PContainer::get_lock)>(last_image_id_);
          last_image_ = file_name;
        }
        pmanage<MPtr(&PContainer::notify)>(last_image_id_);
      });
  if (!new_timelapse.get()) return false;
  timelapse_[shmpath] = std::move(new_timelapse);
  return true;
}

bool Timelapse::can_sink_caps(const std::string& caps) {
  // assuming timelapse_ is internally using videoconvert as first caps
  // negotiating gst element:
  return GstUtils::can_sink_caps("videoconvert", caps);
}

}  // namespace switcher
