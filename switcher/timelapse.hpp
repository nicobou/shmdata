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

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include "switcher/fraction.hpp"
#include "switcher/gst-video-timelapse.hpp"
#include "switcher/periodic-task.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"

namespace switcher {
class Timelapse : public Quiddity {
 public:
  Timelapse(QuiddityConfiguration&&);
  ~Timelapse() = default;
  Timelapse(const Timelapse&) = delete;
  Timelapse& operator=(const Timelapse&) = delete;

 private:
  // images path
  std::string img_dir_;
  PContainer::prop_id_t img_dir_id_;
  std::string img_name_;
  PContainer::prop_id_t img_name_id_;
  // numerate files
  bool num_files_{true};
  PContainer::prop_id_t num_files_id_;
  // bool notify last file
  bool notify_last_file_{false};
  PContainer::prop_id_t notify_last_file_id_;
  // framerate
  Fraction framerate_{1, 1};
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

  // tracking parameter changes and update timelapse pipeline
  PeriodicTask<> relaunch_task_;

  // registering connect/disconnect/can_sink_caps:
  ShmdataConnector shmcntr_;
  GstVideoTimelapseConfig timelapse_config_;
  std::map<std::string, std::unique_ptr<GstVideoTimelapse>> timelapse_{};

  bool on_shmdata_disconnect(const std::string& shmdata_sochet_path);
  bool on_shmdata_connect(const std::string& shmdata_sochet_path);
  bool on_shmdata_disconnect_all();
  bool can_sink_caps(const std::string& caps);
  bool start_timelapse(const std::string& shmpath);
  bool stop_timelapse(const std::string& shmpath);
};

}  // namespace switcher
#endif
