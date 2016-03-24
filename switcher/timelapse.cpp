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
        [this](const std::string &val){img_path_ = val; return true;},
        [this](){return img_path_;},
        "Image Path",
        "Path for the jpeg files to be produced. if empty, the path will be <video_shmdata_path>%05d.jpg",
        img_path_)),
    framerate_id_(pmanage<MPtr(&PContainer::make_fraction)>(
        "framerate",
        [this](const Fraction &val){
          framerate_ = val; return true;
        },
        [this](){return framerate_;},
        "Framerate",
        "Number of image to be produced by seconds",
        framerate_,
        1, 1,  // min num/denom
        60, 5)),  // max num/denom
    last_image_id_(pmanage<MPtr(&PContainer::make_string)>(
        "last_image",
        nullptr,
        [this](){return last_image_;},
        "Last image written",
        "Path of the last jpeg file written",
        last_image_)),
    scale_id_(pmanage<MPtr(&PContainer::make_bool)>(
        "scale",
        [this](bool val){
          scale_ = val;
          pmanage<MPtr(&PContainer::enable)>(width_id_, scale_);
          pmanage<MPtr(&PContainer::enable)>(height_id_, scale_);
          return true;
        },
        [this](){return scale_;},
        "Scale image",
        "scale image width and height",
        scale_)),
    width_id_(pmanage<MPtr(&PContainer::make_unsigned_int)>(
        "width_",
        [this](unsigned int val){width_ = val; return true;},
        [this](){return width_;},
        "Width",
        "Width of the scaled image",
        width_, 0, 8192)),
    height_id_(pmanage<MPtr(&PContainer::make_unsigned_int)>(
        "height",
        [this](unsigned int val){height_ = val; return true;},
        [this](){return height_;},
        "Height",
        "Height of the scaled image",
        height_, 0, 8192)){
      pmanage<MPtr(&PContainer::enable)>(width_id_, scale_);
      pmanage<MPtr(&PContainer::enable)>(height_id_, scale_);
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
  if (!timelapse_)
    return true;
  timelapse_.reset();
  pmanage<MPtr(&PContainer::enable)>(img_path_id_, true);
  pmanage<MPtr(&PContainer::enable)>(framerate_id_, true);
  pmanage<MPtr(&PContainer::enable)>(scale_id_, true);
  pmanage<MPtr(&PContainer::enable)>(width_id_, scale_);
  pmanage<MPtr(&PContainer::enable)>(height_id_, scale_);
  return true;
}

bool Timelapse::on_shmdata_connect(const std::string &shmpath) {
  timelapse_config_ =
      GstVideoTimelapseConfig(shmpath,
                              img_path_.empty() ? shmpath + "%05d.jpg" : img_path_);
  timelapse_config_.framerate_num_ = framerate_.numerator();
  timelapse_config_.framerate_denom_ = framerate_.denominator();
  timelapse_config_.scale_ = scale_;
  timelapse_config_.width_ = width_;
  timelapse_config_.height_ = height_;
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
        pmanage<MPtr(&PContainer::notify)>(last_image_id_);
      });
  if (!*timelapse_.get())
    return false;
  pmanage<MPtr(&PContainer::enable)>(framerate_id_, false);
  pmanage<MPtr(&PContainer::enable)>(img_path_id_, false);
  pmanage<MPtr(&PContainer::enable)>(scale_id_, false);
  pmanage<MPtr(&PContainer::enable)>(width_id_, false);
  pmanage<MPtr(&PContainer::enable)>(height_id_, false);
  return true;
}

bool Timelapse::can_sink_caps(const std::string &caps) {
  // assuming timelapse_ is internally using videoconvert as first caps negotiating gst element: 
  return GstUtils::can_sink_caps("videoconvert", caps);
}

}  // namespace switcher
