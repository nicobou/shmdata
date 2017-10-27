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

#ifndef __SWITCHER_POSTURE_MESHMERGE_H__
#define __SWITCHER_POSTURE_MESHMERGE_H__

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "./posture.hpp"
#include "./posture_worker.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"

namespace switcher {
class PostureMeshMerge : public Quiddity, public StartableQuiddity {
 public:
  PostureMeshMerge(QuiddityConfiguration&&);
  ~PostureMeshMerge();
  PostureMeshMerge(const PostureMeshMerge&) = delete;
  PostureMeshMerge& operator=(const PostureMeshMerge&) = delete;

  bool start();
  bool stop();

 private:
  ShmdataConnector shmcntr_;

  std::unique_ptr<posture::CalibrationReader> calibration_reader_{nullptr};
  std::string calibration_path_{"default.kvc"};
  bool reload_calibration_{false};
  bool apply_calibration_{true};

  unsigned int source_id_{0};
  std::shared_ptr<posture::MeshMerger> merger_{nullptr};
  std::mutex mutex_{};
  std::mutex updateMutex_{};
  Worker worker_{};

  std::mutex connect_mutex_{};
  unsigned int shmreader_id_{0};
  std::map<std::string, std::unique_ptr<ShmdataFollower>> mesh_readers_{};
  std::map<int, std::string> mesh_readers_caps_{};
  std::unique_ptr<ShmdataWriter> mesh_writer_{};


  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string shmName);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);
};

SWITCHER_DECLARE_PLUGIN(PostureMeshMerge);
}  // namespace switcher
#endif
