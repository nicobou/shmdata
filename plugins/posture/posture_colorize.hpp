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

#ifndef __SWITCHER_POSTURE_COLORIZE_H__
#define __SWITCHER_POSTURE_COLORIZE_H__

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "./posture.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/segment.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/custom-property-helper.hpp"

namespace switcher {
class PostureColorize : public Quiddity, public Segment, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureColorize);
  PostureColorize();
  ~PostureColorize();
  PostureColorize(const PostureColorize &) = delete;
  PostureColorize &operator=(const PostureColorize &) = delete;

  bool start();
  bool stop();

 private:
  CustomPropertyHelper::ptr custom_props_;
  std::string calibration_path_ {"default.kvc"};
  double face_sort_resolution_ {0.01};

  GParamSpec *calibration_path_prop_ {nullptr};
  GParamSpec *face_sort_resolution_prop_ {nullptr};

  std::shared_ptr<posture::Colorize> colorize_ {nullptr};
  std::mutex mutex_ {};

  bool has_input_mesh_ {false};
  int mesh_index_ {-1};
  std::map<unsigned int, unsigned int> shm_index_ {};
  std::vector<unsigned char> mesh_ {};
  std::vector<std::vector<unsigned char>> images_ {};
  std::vector<std::vector<unsigned int>> dims_ {};
  unsigned int source_id_ {0};

  ShmdataAnyWriter::ptr mesh_writer_ {nullptr};
  ShmdataAnyWriter::ptr tex_writer_ {nullptr};
  std::deque<std::shared_ptr<std::vector<unsigned char>>> shmwriter_queue_ {};

  bool init() final;

  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string /*unused*/);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);
  bool check_image_caps(std::string caps, unsigned int& width, unsigned int& height, unsigned int& channels);

  static const gchar *get_calibration_path(void *user_data);
  static void set_calibration_path(const gchar *name, void *user_data);
  static double get_face_sort_resolution(void *user_data);
  static void set_face_sort_resolution(double resolution, void *user_data);

  static void free_sent_buffer(void* data);
  void check_buffers();  
};

SWITCHER_DECLARE_PLUGIN(PostureColorize);
}  // namespace switcher

#endif
