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
#include "./posture_worker.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/custom-property-helper.hpp"

namespace switcher {
class PostureColorize : public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureColorize);
  PostureColorize(const std::string &);
  ~PostureColorize();
  PostureColorize(const PostureColorize &) = delete;
  PostureColorize &operator=(const PostureColorize &) = delete;

  bool start();
  bool stop();

 private:
  CustomPropertyHelper::ptr custom_props_;
  ShmdataConnector shmcntr_;

  std::string calibration_path_ {"default.kvc"};
  bool compute_tex_coords_ {false};
  bool compress_mesh_ {true};

  GParamSpec *calibration_path_prop_ {nullptr};
  GParamSpec *compute_tex_coords_prop_ {nullptr};
  GParamSpec *compress_mesh_prop_ {nullptr};

  Worker worker_ {};

  std::shared_ptr<posture::Colorize> colorize_ {nullptr};
  std::mutex mutex_ {};
  std::mutex imageMutex_ {};

  bool has_input_mesh_ {false};
  int mesh_index_ {-1};
  std::map<unsigned int, unsigned int> shm_index_ {};
  //std::vector<unsigned char> mesh_ {};
  std::vector<std::vector<unsigned char>> images_ {};
  std::vector<std::vector<unsigned int>> dims_ {};
  unsigned int source_id_ {0};

  std::mutex connect_mutex_ {};
  std::map<std::string, std::unique_ptr<ShmdataFollower>> shmdata_readers_ {};
  std::map<int, std::string> shmdata_reader_caps_ {};
  unsigned int shmdata_reader_id_ {0};

  std::unique_ptr<ShmdataWriter> mesh_writer_ {nullptr};
  std::unique_ptr<ShmdataWriter> tex_writer_ {nullptr};

  unsigned int prev_width_ {0}, prev_height_ {0}; // Used to check the texture size did not change

  bool init() final;

  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string /*unused*/);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);
  bool check_image_caps(std::string caps, unsigned int& width, unsigned int& height, unsigned int& channels);

  static const gchar *get_calibration_path(void *user_data);
  static void set_calibration_path(const gchar *name, void *user_data);
  static int get_compute_tex_coords(void *user_data);
  static void set_compute_tex_coords(const int compute, void *user_data);
  static int get_compress_mesh(void *user_data);
  static void set_compress_mesh(const int compress, void *user_data);
};

SWITCHER_DECLARE_PLUGIN(PostureColorize);
}  // namespace switcher

#endif