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

#include "switcher/std2.hpp"
#include "switcher/shmdata-utils.hpp"
#include "./timelapse.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    Timelapse,
    "timelapse",
    "Timelapse",
    "video",
    "reader",
    "Make an image timelapse from raw video stream",
    "LGPL",
    "Nicolas Bouillot");

Timelapse::Timelapse(const std::string &):
    shmcntr_(static_cast<Quiddity *>(this)),
    timelapse_config_{std::string(), std::string()},
    img_path_id_(pmanage<MPtr(&PContainer::make_string)>(
        "imgpath",
        [this](const std::string &val){
          img_path_ = val;
          updated_config_.store(true);
          return true;
        },
        [this](){return img_path_;},
        "Image Path",
        "Path for the jpeg files to be produced. if empty, the path will be <video_shmdata_path>%05d.jpg",
        img_path_)),
    framerate_id_(pmanage<MPtr(&PContainer::make_fraction)>(
        "framerate",
        [this](const Fraction &val){
          framerate_ = val;
          updated_config_.store(true);
          return true;
        },
        [this](){return framerate_;},
        "Framerate",
        "Number of image to be produced by seconds",
        framerate_,
        1, 1,  // min num/denom
        60, 5)),  // max num/denom
    max_files_id_(pmanage<MPtr(&PContainer::make_unsigned_int)>(
        "maxfiles",
        [this](unsigned int val){
          max_files_ = val;
          updated_config_.store(true);
          return true;
        },
        [this](){return max_files_;},
        "Max files",
        "Maximum number of files simultaneously present on disk",
        max_files_, 0, 4294967295)),
    jpg_quality_id_(pmanage<MPtr(&PContainer::make_unsigned_int)>(
        "quality",
        [this](unsigned int val){
          jpg_quality_ = val;
          updated_config_.store(true);
          return true;
        },
        [this](){return jpg_quality_;},
        "JPEG quality",
        "Quality of the produced jpeg image",
        jpg_quality_, 0, 100)),
    last_image_id_(pmanage<MPtr(&PContainer::make_string)>(
        "last_image",
        nullptr,
        [this](){return last_image_;},
        "Last image written",
        "Path of the last jpeg file written",
        last_image_)),
    width_id_(pmanage<MPtr(&PContainer::make_unsigned_int)>(
        "width_",
        [this](unsigned int val){
          width_ = val;
          updated_config_.store(true);
          return true;
        },
        [this](){return width_;},
        "Width",
        "Width of the scaled image",
        width_, 0, 8192)),
    height_id_(pmanage<MPtr(&PContainer::make_unsigned_int)>(
        "height",
        [this](unsigned int val){
          height_ = val;
          updated_config_.store(true);
          return true;
        },
        [this](){return height_;},
        "Height",
        "Height of the scaled image",
        height_, 0, 8192)){
    }

bool Timelapse::init() {
  shmcntr_.install_connect_method(
      [this](const std::string &shmpath){return this->on_shmdata_connect(shmpath);},
      [this](const std::string &){return this->on_shmdata_disconnect();},
      [this](){return this->on_shmdata_disconnect();},
      [this](const std::string &caps){return this->can_sink_caps(caps);},
      1);
  return true;
}

bool Timelapse::on_shmdata_disconnect() {
  return stop_timelapse();
}

bool Timelapse::on_shmdata_connect(const std::string &shmpath) {
  shmpath_ = shmpath;
  return start_timelapse();
}

bool Timelapse::stop_timelapse(){
  std::unique_lock<std::mutex> lock(timelapse_mtx_);
  if (!timelapse_)
    return true;
  timelapse_.reset();
  return true;
}

bool Timelapse::start_timelapse(){
  std::unique_lock<std::mutex> lock(timelapse_mtx_);
  timelapse_.reset();
  auto img_path = img_path_.empty() ? shmpath_ + "%05d.jpg" : img_path_;
  if (std::string::npos == img_path.find('%'))
    img_path += "_%d.jpg";
  timelapse_config_ =
      GstVideoTimelapseConfig(shmpath_,
                              img_path);
  timelapse_config_.framerate_num_ = framerate_.numerator();
  timelapse_config_.framerate_denom_ = framerate_.denominator();
  timelapse_config_.width_ = width_;
  timelapse_config_.height_ = height_;
  timelapse_config_.jpg_quality_ = jpg_quality_;
  timelapse_config_.max_files_ = max_files_;
  timelapse_ = std2::make_unique<GstVideoTimelapse>(
      timelapse_config_,
      [this](const std::string &caps) {
        graft_tree(".shmdata.reader." + timelapse_config_.orig_shmpath_,
                   ShmdataUtils::make_tree(caps,
                                           ShmdataUtils::get_category(caps),
                                           0));
      },
      [this](GstShmdataSubscriber::num_bytes_t byte_rate){
        graft_tree(".shmdata.reader." + timelapse_config_.orig_shmpath_ + ".byte_rate",
                   InfoTree::make(std::to_string(byte_rate)));
      },
      nullptr,
      [this](std::string &&file_name){
        { auto lock = pmanage<MPtr(&PContainer::get_lock)>(last_image_id_);
          last_image_ = file_name;
        }
        if (updated_config_.load()){
          updated_config_.store(false);
          fut_ = std::async(
              std::launch::async,
              [this](){start_timelapse();});
        }
        pmanage<MPtr(&PContainer::notify)>(last_image_id_);
      });
  if (!*timelapse_.get())
    return false;
  return true;
}

bool Timelapse::can_sink_caps(const std::string &caps) {
  // assuming timelapse_ is internally using videoconvert as first caps negotiating gst element: 
  return GstUtils::can_sink_caps("videoconvert", caps);
}

}  // namespace switcher
