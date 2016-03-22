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

#ifndef __SWITCHER_TIMELAPSE_H__
#define __SWITCHER_TIMELAPSE_H__

#include <memory>
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/gst-video-timelapse.hpp"
#include "switcher/fraction.hpp"

namespace switcher {
class Timelapse: public Quiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(Timelapse);
  Timelapse(const std::string &);
  ~Timelapse() = default;
  Timelapse(const Timelapse &) = delete;
  Timelapse &operator=(const Timelapse &) = delete;

 private:
  // registering connect/disconnect/can_sink_caps:
  ShmdataConnector shmcntr_;
  GstVideoTimelapseConfig timelapse_config_; 
  std::unique_ptr<GstVideoTimelapse> timelapse_{nullptr};
  // images path 
  std::string img_path_;
  PContainer::prop_id_t img_path_id_;
  // framerate
  Fraction framerate_{1,1};
  PContainer::prop_id_t framerate_id_;
  
  bool init() final;
  bool on_shmdata_disconnect();
  bool on_shmdata_connect(const std::string &shmdata_sochet_path);
  bool can_sink_caps(const std::string &caps);
};

}  // namespace switcher
#endif