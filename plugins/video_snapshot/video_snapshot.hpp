/*
 * This file is part of switcher-video-snapshot.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_VIDEO_SNAPSHOT_H__
#define __SWITCHER_VIDEO_SNAPSHOT_H__

#include "switcher/gst/pipeliner.hpp"
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/shmdata/connector.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class VideoSnapshot : public Quiddity {
 public:
  using on_new_file_t = std::function<void(std::string&&)>;
  VideoSnapshot(quiddity::Config&&);

 private:
  // Take Snaphot property
  property::prop_id_t snap_id_;

  // last image
  std::string last_image_{};
  property::prop_id_t last_image_id_;

  // images path
  std::string img_dir_{"/tmp/"};
  property::prop_id_t img_dir_id_;
  std::string img_name_{};
  property::prop_id_t img_name_id_;

  // numerate files
  bool num_files_{true};
  property::prop_id_t num_files_id_;

  // image quality
  unsigned int jpg_quality_{85};
  property::prop_id_t jpg_quality_id_;

  // registering connect/disconnect/can_sink_caps:
  shmdata::Connector shmcntr_;

  // gst pipeline
  std::mutex mtx_{};
  std::unique_ptr<gst::Pipeliner> gst_pipeline_;
  GstElement* valve_{nullptr};

  void make_gst_pipeline(const std::string& shmpath);
  void on_new_file(const std::string& filename);
  bool on_shmdata_disconnect();
  bool on_shmdata_connect(const std::string& shmdata_socket_path);
  bool can_sink_caps(const std::string& caps);
};

SWITCHER_DECLARE_PLUGIN(VideoSnapshot);
}  // namespace quiddities
}  // namespace switcher
#endif  // __SWITCHER_VIDEO_SNAPSHOT_H__
