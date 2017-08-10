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

#ifndef __SWITCHER_POSTURE_DETECT_H__
#define __SWITCHER_POSTURE_DETECT_H__

#include <deque>
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
class PostureDetect : public Quiddity, public StartableQuiddity {
 public:
  PostureDetect(QuiddityConfiguration&&);
  ~PostureDetect();
  PostureDetect(const PostureDetect&) = delete;
  PostureDetect& operator=(const PostureDetect&) = delete;

  bool start();
  bool stop();

 private:
  ShmdataConnector shmcntr_;
  bool compress_cloud_{false};

  std::shared_ptr<posture::Detect> detect_{nullptr};
  std::mutex mutex_{};

  std::unique_ptr<ShmdataFollower> reader_{nullptr};
  std::string reader_caps_{};
  std::unique_ptr<ShmdataWriter> cloud_writer_{nullptr};
  std::unique_ptr<ShmdataWriter> mesh_writer_{nullptr};


  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string shmName);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);
};

SWITCHER_DECLARE_PLUGIN(PostureDetect);
}  // namespace switcher
#endif
