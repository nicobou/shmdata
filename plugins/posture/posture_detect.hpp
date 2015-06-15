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
#include "switcher/segment.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/custom-property-helper.hpp"

namespace switcher {
class PostureDetect : public Quiddity, public Segment, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureDetect);
  PostureDetect(const std::string &);
  ~PostureDetect();
  PostureDetect(const PostureDetect &) = delete;
  PostureDetect &operator=(const PostureDetect &) = delete;

  bool start();
  bool stop();

 private:
  CustomPropertyHelper::ptr custom_props_;
  bool compress_cloud_ {false};

  std::shared_ptr<posture::Detect> detect_ {nullptr};
  std::mutex mutex_ {};

  ShmdataAnyWriter::ptr cloud_writer_ {nullptr};
  ShmdataAnyWriter::ptr mesh_writer_ {nullptr};
  std::deque<std::shared_ptr<std::vector<unsigned char>>> shmwriter_queue_ {};

  bool init() final;

  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string shmName);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);

  static void free_sent_buffer(void* data);
  void check_buffers();
};

SWITCHER_DECLARE_PLUGIN(PostureDetect);
}  // namespace switcher

#endif
