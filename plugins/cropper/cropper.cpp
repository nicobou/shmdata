/*
 * This file is part of switcher-cropper.
 *
 * switcher-cropper is free software; you can redistribute it and/or
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

#include "cropper.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Cropper,
                                     "cropper",
                                     "Video Cropper",
                                     "video",
                                     "reader/writer",
                                     "Plugin for cropping video sources",
                                     "LGPL",
                                     "Francis Lecavalier");

const std::string Cropper::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "video",
      "description": "Video stream to crop",
      "can_do": ["video/x-raw"]
    }
  ],
"writer":
  [
    {
      "label": "video",
      "description": "Croped video stream",
      "can_do": ["video/x-raw"]
    }
  ]
}
)");

Cropper::Cropper(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf),
               {kConnectionSpec,
                [this](const std::string& shmpath, claw::sfid_t sfid) {
                  return on_shmdata_connect(shmpath);
                },
                [this](claw::sfid_t sfid) { return on_shmdata_disconnect(); }}),
      gst_pipeline_(std::make_unique<gst::Pipeliner>(nullptr, nullptr)),
      left_id_(pmanage<MPtr(&property::PBag::make_int)>(
          "left",
          [this](const int val) {
            left_ = val;
            if (gst_pipeline_ != nullptr) {
              async_this_.run_async([this]() { create_pipeline(); });
            }
            return true;
          },
          [this]() { return left_; },
          "Left Crop",
          "Pixels to crop on the left side",
          0,
          0,
          4096)),
      right_id_(pmanage<MPtr(&property::PBag::make_int)>(
          "right",
          [this](const int val) {
            right_ = val;
            if (gst_pipeline_ != nullptr) {
              async_this_.run_async([this]() { create_pipeline(); });
            }
            return true;
          },
          [this]() { return right_; },
          "Right Crop",
          "Pixels to crop on the right side",
          0,
          0,
          4096)),
      top_id_(pmanage<MPtr(&property::PBag::make_int)>(
          "top",
          [this](const int val) {
            top_ = val;
            if (gst_pipeline_ != nullptr) {
              async_this_.run_async([this]() { create_pipeline(); });
            }
            return true;
          },
          [this]() { return top_; },
          "Top Crop",
          "Pixels to crop at the top",
          0,
          0,
          4096)),
      bottom_id_(pmanage<MPtr(&property::PBag::make_int)>(
          "bottom",
          [this](const int val) {
            bottom_ = val;
            if (gst_pipeline_ != nullptr) {
              async_this_.run_async([this]() { create_pipeline(); });
            }
            return true;
          },
          [this]() { return bottom_; },
          "Bottom Crop",
          "Pixels to crop at the bottom",
          0,
          0,
          4096)) {
  shmpath_cropped_ = claw_.get_shmpath_from_writer_label("video");
}

bool Cropper::on_shmdata_connect(const std::string& shmpath) {
  if (shmpath.empty()) {
    error("shmpath must not be empty");
    return false;
  }
  if (shmpath == shmpath_cropped_) {
    error("cropper cannot connect to itself");
    return false;
  }
  shmpath_to_crop_ = shmpath;
  shmsrc_sub_.reset();
  shmsink_sub_.reset();
  if (!create_pipeline()) {
    return false;
  }
  shmsrc_sub_ = std::make_unique<shmdata::Follower>(
      this,
      shmpath_to_crop_,
      nullptr,
      [this](const std::string& caps) {
        if (!cur_caps_.empty() && cur_caps_ != caps) {
          cur_caps_ = caps;
          debug(
              "cropper restarting shmdata connection "
              "because of updated caps (%)",
              cur_caps_);
          async_this_.run_async([this]() { on_shmdata_connect(shmpath_to_crop_); });

          return;
        }
        cur_caps_ = caps;
      },
      nullptr,
      shmdata::Stat::kDefaultUpdateInterval,
      shmdata::Follower::Direction::reader,
      true);
  shmsink_sub_ = std::make_unique<shmdata::Follower>(this,
                                                     shmpath_cropped_,
                                                     nullptr,
                                                     nullptr,
                                                     nullptr,
                                                     shmdata::Stat::kDefaultUpdateInterval,
                                                     shmdata::Follower::Direction::writer,
                                                     true);
  return true;
}

bool Cropper::on_shmdata_disconnect() {
  if (shmpath_to_crop_.empty()) {
    return false;
  }
  cur_caps_.clear();
  shmsink_sub_.reset();
  shmsrc_sub_.reset();
  gst_pipeline_->play(false);
  gst_pipeline_ = nullptr;
  shmpath_to_crop_ = "";
  return true;
}

bool Cropper::remake_elements() {
  if (!gst::UGstElem::renew(shmsrc_) || !gst::UGstElem::renew(queue_element_) ||
      !gst::UGstElem::renew(cropper_element_) || !gst::UGstElem::renew(scaler_element_) ||
      !gst::UGstElem::renew(shmsink_, {"socket-path", "sync", "extra-caps-properties"})) {
    error("cropper could not renew GStreamer elements");
    return false;
  }
  return true;
}

bool Cropper::create_pipeline() {
  if (gst_pipeline_ != nullptr) {
    gst_pipeline_->play(false);
  }
  gst_pipeline_ = std::make_unique<gst::Pipeliner>(nullptr, nullptr);
  if (!remake_elements()) {
    return false;
  };
  g_object_set(G_OBJECT(shmsrc_.get_raw()), "socket-path", shmpath_to_crop_.c_str(), nullptr);
  g_object_set(G_OBJECT(cropper_element_.get_raw()),
               "left",
               left_,
               "right",
               right_,
               "top",
               top_,
               "bottom",
               bottom_,
               nullptr);
  auto extra_caps = get_quiddity_caps();
  g_object_set(G_OBJECT(shmsink_.get_raw()),
               "socket-path",
               shmpath_cropped_.c_str(),
               "sync",
               false,
               "extra-caps-properties",
               extra_caps.c_str(),
               nullptr);
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   shmsrc_.get_raw(),
                   queue_element_.get_raw(),
                   cropper_element_.get_raw(),
                   scaler_element_.get_raw(),
                   shmsink_.get_raw(),
                   nullptr);
  gst_element_link_many(shmsrc_.get_raw(),
                        queue_element_.get_raw(),
                        cropper_element_.get_raw(),
                        scaler_element_.get_raw(),
                        shmsink_.get_raw(),
                        nullptr);
  if (!static_cast<bool>(gst_pipeline_.get())) {
    return false;
  }
  gst_pipeline_->play(true);
  return true;
}

}  // namespace quiddities
}  // namespace switcher
