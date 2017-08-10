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
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GstVideoConverter,
                                     "videoconvert",
                                     "Video converter",
                                     "video",
                                     "writer/reader",
                                     "Convert pixel format of raw video stream",
                                     "LGPL",
                                     "Nicolas Bouillot");

GstVideoConverter::GstVideoConverter(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      video_format_(
          GstUtils::get_gst_element_capability_as_list("videoconvert", "format", GST_PAD_SRC), 0),
      video_format_id_(
          pmanage<MPtr(&PContainer::make_selection<>)>("Pixel format",
                                                       [this](const IndexOrName& val) {
                                                         video_format_.select(val);
                                                         return true;
                                                       },
                                                       [this]() { return video_format_.get(); },
                                                       "Convert to selected pixel format",
                                                       "Pixel format to convert into",
                                                       video_format_)),
      shmcntr_(static_cast<Quiddity*>(this)) {
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);
  shmpath_converted_ = make_file_name("video");
}

bool GstVideoConverter::on_shmdata_disconnect() {
  prune_tree(".shmdata.writer." + shmpath_converted_);
  prune_tree(".shmdata.reader." + shmpath_to_convert_);
  shmsink_sub_.reset();
  shmsrc_sub_.reset();
  converter_.reset(nullptr);
  pmanage<MPtr(&PContainer::enable)>(video_format_id_);
  return true;
}

bool GstVideoConverter::on_shmdata_connect(const std::string& shmpath) {
  if (shmpath == shmpath_converted_) {
    message("ERROR:videoconverter cannot connect to itself");
    return false;
  }
  shmpath_to_convert_ = shmpath;
  converter_.reset(nullptr);
  converter_ = std::make_unique<GstPixelFormatConverter>(
      shmpath_to_convert_, shmpath_converted_, video_format_.get_attached());
  if (!static_cast<bool>(*converter_.get())) return false;
  shmsink_sub_ = std::make_unique<GstShmdataSubscriber>(
      converter_->get_shmsink(),
      [this](const std::string& caps) {
        graft_tree(".shmdata.writer." + shmpath_converted_,
                   ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.writer." + shmpath_converted_));
  shmsrc_sub_ = std::make_unique<GstShmdataSubscriber>(
      converter_->get_shmsrc(),
      [this](const std::string& caps) {
        graft_tree(".shmdata.reader." + shmpath_to_convert_,
                   ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.reader." + shmpath_to_convert_));
  pmanage<MPtr(&PContainer::disable)>(video_format_id_, ShmdataConnector::disabledWhenConnectedMsg);
  return true;
}

bool GstVideoConverter::can_sink_caps(const std::string& caps) {
  return GstPixelFormatConverter::can_sink_caps(caps);
}

}  // namespace switcher
