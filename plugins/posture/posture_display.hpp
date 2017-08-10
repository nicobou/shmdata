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

#ifndef __SWITCHER_POSTURE_DISPLAY_H__
#define __SWITCHER_POSTURE_DISPLAY_H__

#include <memory>
#include <mutex>
#include <string>

#include "./posture.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"

namespace switcher {
class PostureDisplay : public Quiddity {
 public:
  PostureDisplay(QuiddityConfiguration&&);
  ~PostureDisplay();
  PostureDisplay(const PostureDisplay&) = delete;
  PostureDisplay& operator=(const PostureDisplay&) = delete;

 private:
  ShmdataConnector shmcntr_;

  std::unique_ptr<ShmdataFollower> reader_{nullptr};
  std::string reader_caps_{};
  std::unique_ptr<posture::Display> display_{nullptr};
  std::mutex display_mutex_{};


  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string /*unused*/);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);
};

SWITCHER_DECLARE_PLUGIN(PostureDisplay);
}  // namespace switcher
#endif
