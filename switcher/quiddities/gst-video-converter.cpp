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

#include "./gst-video-converter.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GstVideoConverter,
                                     "videoconvert",
                                     "Video converter",
                                     "video",
                                     "writer/reader",
                                     "Convert pixel format of raw video stream",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string GstVideoConverter::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "video",
      "description": "Video stream to convert",
      "can_do": ["video/x-raw"]
    }
  ],
"writer":
  [
    {
      "label": "video",
      "description": "Converted video stream",
      "can_do": ["video/x-raw"]
    }
  ]
}
)");

GstVideoConverter::GstVideoConverter(quiddity::Config&& conf)
    : Quiddity(
          std::forward<quiddity::Config>(conf),
          {kConnectionSpec,
           [this](const std::string& shmpath, claw::sfid_t) { return on_shmdata_connect(shmpath); },
           [this](claw::sfid_t) { return on_shmdata_disconnect(); }}),
      video_format_(
          gst::utils::get_gst_element_capability_as_list("videoconvert", "format", GST_PAD_SRC), 0),
      video_format_id_(pmanage<MPtr(&property::PBag::make_selection<>)>(
          "pixel_format",
          [this](const quiddity::property::IndexOrName& val) {
            video_format_.select(val);
            return true;
          },
          [this]() { return video_format_.get(); },
          "Convert to selected pixel format",
          "Pixel format to convert into",
          video_format_)) {
  shmpath_converted_ = claw_.get_shmpath_from_writer_label("video");
}

bool GstVideoConverter::on_shmdata_disconnect() {
  shmsink_sub_.reset();
  shmsrc_sub_.reset();
  converter_.reset();
  pmanage<MPtr(&property::PBag::enable)>(video_format_id_);
  return true;
}

bool GstVideoConverter::on_shmdata_connect(const std::string& shmpath) {
  if (shmpath == shmpath_converted_) {
    message("ERROR:videoconverter cannot connect to itself");
    return false;
  }
  shmpath_to_convert_ = shmpath;
  converter_.reset();
  auto extra_caps = get_quiddity_caps();
  converter_ = std::make_unique<gst::PixelFormatConverter>(
      shmpath_to_convert_, shmpath_converted_, video_format_.get_attached(), extra_caps);
  if (!static_cast<bool>(*converter_.get())) return false;
  shmsink_sub_ =
      std::make_unique<shmdata::GstTreeUpdater>(this,
                                                converter_->get_shmsink(),
                                                shmpath_converted_,
                                                shmdata::GstTreeUpdater::Direction::writer);
  shmsrc_sub_ =
      std::make_unique<shmdata::GstTreeUpdater>(this,
                                                converter_->get_shmsrc(),
                                                shmpath_to_convert_,
                                                shmdata::GstTreeUpdater::Direction::reader);
  pmanage<MPtr(&property::PBag::disable)>(video_format_id_,
                                          property::PBag::disabledWhenConnectedMsg);
  return true;
}

}  // namespace quiddities
}  // namespace switcher
