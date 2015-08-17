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
#include "switcher/shmdata-writer.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/startable-quiddity.hpp"
#include "switcher/custom-property-helper.hpp"

namespace switcher {
class PostureMerge : public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureMerge);
  PostureMerge(const std::string &);
  ~PostureMerge();
  PostureMerge(const PostureMerge &) = delete;
  PostureMerge &operator=(const PostureMerge &) = delete;

  bool start();
  bool stop();

 private:
  CustomPropertyHelper::ptr custom_props_;
  ShmdataConnector shmcntr_;

  std::string calibration_path_ {"default.kvc"};
  std::string devices_path_ {"devices.xml"};
  bool compress_cloud_ {false};
  bool reload_calibration_ {false};
  bool save_cloud_ {false};
  bool downsample_ {false};
  double downsample_resolution_ {0.1};

  GParamSpec *calibration_path_prop_ {nullptr};
  GParamSpec *devices_path_prop_ {nullptr};
  GParamSpec *compress_cloud_prop_ {nullptr};
  GParamSpec *reload_calibration_prop_ {nullptr};
  GParamSpec *save_cloud_prop_ {nullptr};
  GParamSpec *downsample_prop_ {nullptr};
  GParamSpec *downsample_resolution_prop_ {nullptr};

  unsigned int source_id_ {0};
  std::shared_ptr<posture::PointCloudMerger> merger_ {nullptr};
  std::mutex mutex_ {};

  std::mutex connect_mutex_ {};
  unsigned int shmreader_id_ {0};
  std::map<std::string, std::unique_ptr<ShmdataFollower>> cloud_readers_ {};
  std::map<int, std::string> cloud_readers_caps_ {};

  std::unique_ptr<ShmdataWriter> cloud_writer_ {nullptr};

  std::map<int, std::vector<char>> stock_ {};
  std::mutex stock_mutex_ {};

  bool init() final;

  bool connect(std::string shmdata_socket_path);
  bool disconnect(std::string shmName);
  bool disconnect_all();
  bool can_sink_caps(std::string caps);

  static const gchar *get_calibration_path(void *user_data);
  static void set_calibration_path(const gchar *name, void *user_data);
  static const gchar *get_devices_path(void *user_data);
  static void set_devices_path(const gchar *name, void *user_data);
  static int get_compress_cloud(void *user_data);
  static void set_compress_cloud(const int compress, void *user_data);
  static int get_reload_calibration(void *user_data);
  static void set_reload_calibration(const int reload, void *user_data);
  static int get_save_cloud(void *user_data);
  static void set_save_cloud(const int save, void *user_data);

  static int get_downsample_active(void *user_data);
  static void set_downsample_active(const int active, void *user_data);
  static double get_downsampling_resolution(void *user_data);
  static void set_downsampling_resolution(const double resolution, void *user_data);  
};

SWITCHER_DECLARE_PLUGIN(PostureMerge);
}  // namespace switcher

#endif
