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

#ifndef __SWITCHER_POSTURE_SOLIDIFY_H__
#define __SWITCHER_POSTURE_SOLIDIFY_H__

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
class PostureSolidify : public Quiddity, public Segment, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureSolidify);
  PostureSolidify();
  ~PostureSolidify();
  PostureSolidify(const PostureSolidify &) = delete;
  PostureSolidify &operator=(const PostureSolidify &) = delete;

  bool start();
  bool stop();

 private:
  CustomPropertyHelper::ptr custom_props_;
  int marching_cubes_resolution_ {16};
  bool save_mesh_ {false};
  GParamSpec *marching_cubes_resolution_prop_ {nullptr};
  GParamSpec *save_mesh_prop_ {nullptr};

  std::shared_ptr<posture::Solidify> solidify_ {nullptr};
  std::mutex mutex_ {};

  ShmdataAnyWriter::ptr mesh_writer_ {nullptr};
  std::deque<std::shared_ptr<std::vector<unsigned char>>> shmwriter_queue_ {};

  bool init() final;

  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string /*unused*/);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);

  static int get_marching_cubes_resolution(void *user_data);
  static void set_marching_cubes_resolution(const int res, void *user_data);
  static int get_save_mesh(void *user_data);
  static void set_save_mesh(const int save, void *user_data);

  static void free_sent_buffer(void* data);
  void check_buffers();
};

SWITCHER_DECLARE_PLUGIN(PostureSolidify);
}  // namespace switcher

#endif
