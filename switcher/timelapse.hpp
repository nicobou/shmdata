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
#include <atomic>
#include <mutex>
#include <future>
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
  // vid shmpath
  std::string shmpath_{};
  // images path 
  std::string img_path_;
  PContainer::prop_id_t img_path_id_;
  // framerate
  Fraction framerate_{1,1};
  PContainer::prop_id_t framerate_id_;
  // max files
  unsigned int max_files_{10};
  PContainer::prop_id_t max_files_id_;
  // image quality
  unsigned int jpg_quality_{85};
  PContainer::prop_id_t jpg_quality_id_;
  // last image 
  std::string last_image_{};
  PContainer::prop_id_t last_image_id_;
  // scaling image
  unsigned int width_{0};
  PContainer::prop_id_t width_id_;
  unsigned int height_{0};
  PContainer::prop_id_t height_id_;

  // making dynamically configurable timelapse
  std::atomic_bool updated_config_{false};
  std::mutex timelapse_mtx_{};
  std::future<void> fut_{};
  
  bool init() final;
  bool on_shmdata_disconnect();
  bool on_shmdata_connect(const std::string &shmdata_sochet_path);
  bool can_sink_caps(const std::string &caps);
  bool start_timelapse();
  bool stop_timelapse();
};

}  // namespace switcher
#endif
