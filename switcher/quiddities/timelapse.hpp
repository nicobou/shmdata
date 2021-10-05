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
#include "../gst/video-timelapse.hpp"
#include "../quiddity/property/fraction.hpp"
#include "../quiddity/quiddity.hpp"
#include "../utils/periodic-task.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class Timelapse : public Quiddity {
 public:
  Timelapse(quiddity::Config&&);

 private:
  static const std::string kConnectionSpec;  //!< Shmdata specifications
  std::string img_dir_;                      //!< images path
  property::prop_id_t img_dir_id_;
  std::string img_name_;
  property::prop_id_t img_name_id_;
  bool num_files_{true};  //!< numerate files
  property::prop_id_t num_files_id_;
  bool notify_last_file_{false};  //!< bool notify last file
  property::prop_id_t notify_last_file_id_;
  property::Fraction framerate_{1, 1};  //!< framerate
  property::prop_id_t framerate_id_;
  unsigned int max_files_{10};  //!< max files
  property::prop_id_t max_files_id_;
  unsigned int jpg_quality_{85};  //!< image quality
  property::prop_id_t jpg_quality_id_;
  std::string last_image_{};  //!< last image
  property::prop_id_t last_image_id_;
  unsigned int width_{0};  //!< scaling image
  property::prop_id_t width_id_;
  unsigned int height_{0};
  property::prop_id_t height_id_;
  std::atomic_bool updated_config_{false};  //!< making dynamically configurable timelapse
  std::mutex timelapse_mtx_{};

  PeriodicTask<> relaunch_task_;  //!< tracking parameter changes and update timelapse pipeline

  gst::VideoTimelapseConfig timelapse_config_;
  std::map<claw::sfid_t, std::unique_ptr<gst::VideoTimelapse>> timelapse_{};

  bool on_shmdata_disconnect(claw::sfid_t sfid);
  bool on_shmdata_connect(const std::string& shmpath, claw::sfid_t sfid);
  bool start_timelapse(const std::string& shmpath, claw::sfid_t sfid);
  bool stop_timelapse(claw::sfid_t sfid);
};

}  // namespace quiddities
}  // namespace switcher
#endif
