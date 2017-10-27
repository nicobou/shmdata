/*
 * This file is part of posture.
 *
 * posture is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_POSTURE_MERGE_H__
#define __SWITCHER_POSTURE_MERGE_H__

#include <deque>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "./posture.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"

namespace switcher {
class PostureMerge : public Quiddity, public StartableQuiddity {
 public:
  PostureMerge(QuiddityConfiguration&&);
  ~PostureMerge();
  PostureMerge(const PostureMerge&) = delete;
  PostureMerge& operator=(const PostureMerge&) = delete;

  bool start();
  bool stop();

 private:
  ShmdataConnector shmcntr_;

  std::unique_ptr<posture::CalibrationReader> calibration_reader_{nullptr};
  std::string calibration_path_{"default.kvc"};
  bool compress_cloud_{false};
  bool reload_calibration_{false};
  bool save_cloud_{false};
  bool downsample_{false};
  double downsample_resolution_{0.1};
  PContainer::prop_id_t downsample_resolution_id_{0};
  unsigned int source_id_{0};
  std::shared_ptr<posture::PointCloudMerger> merger_{nullptr};
  std::mutex mutex_{};

  std::mutex connect_mutex_{};
  unsigned int shmreader_id_{0};
  std::map<std::string, std::unique_ptr<ShmdataFollower>> cloud_readers_{};
  std::map<int, std::string> cloud_readers_caps_{};

  std::unique_ptr<ShmdataWriter> cloud_writer_{nullptr};

  std::map<int, std::vector<char>> stock_{};
  std::mutex stock_mutex_{};

  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string shmName);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);
};

SWITCHER_DECLARE_PLUGIN(PostureMerge);
}  // namespace switcher
#endif
