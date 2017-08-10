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

#ifndef __SWITCHER_GST_VIDEO_CONVERTER_H__
#define __SWITCHER_GST_VIDEO_CONVERTER_H__

#include <memory>
#include "switcher/gst-pixel-format-converter.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"

namespace switcher {
class GstVideoConverter : public Quiddity {
 public:
  GstVideoConverter(QuiddityConfiguration&&);
  ~GstVideoConverter() = default;
  GstVideoConverter(const GstVideoConverter&) = delete;
  GstVideoConverter& operator=(const GstVideoConverter&) = delete;

 private:
  std::string shmpath_to_convert_{};
  std::string shmpath_converted_{};
  Selection<> video_format_;
  PContainer::prop_id_t video_format_id_;
  // registering connect/disconnect/can_sink_caps:
  ShmdataConnector shmcntr_;
  std::unique_ptr<GstShmdataSubscriber> shmsrc_sub_{nullptr};
  std::unique_ptr<GstShmdataSubscriber> shmsink_sub_{nullptr};

  std::unique_ptr<GstPixelFormatConverter> converter_{nullptr};
  bool on_shmdata_disconnect();
  bool on_shmdata_connect(const std::string& shmdata_sochet_path);
  bool can_sink_caps(const std::string& caps);
};

}  // namespace switcher
#endif
