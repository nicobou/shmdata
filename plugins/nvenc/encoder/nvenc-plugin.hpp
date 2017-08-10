/*
 * This file is part of switcher-nvenc.
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

#ifndef __SWITCHER_NVENC_PLUGIN_H__
#define __SWITCHER_NVENC_PLUGIN_H__

#include <cuda.h>
#include <memory>
#include "./nvenc-encode-session.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/threaded-wrapper.hpp"

namespace switcher {
class NVencPlugin : public Quiddity {
 public:
  NVencPlugin(QuiddityConfiguration&&);
  ~NVencPlugin() = default;
  NVencPlugin(const NVencPlugin&) = delete;
  NVencPlugin& operator=(const NVencPlugin&) = delete;


 private:
  std::unique_ptr<ThreadedWrapper<NVencES>> es_;
  Selection<> devices_{{"none"}, 0};
  bool bitrate_from_preset_{false};
  PContainer::prop_id_t default_preset_id_;
  uint32_t bitrate_{5000000};
  PContainer::prop_id_t bitrate_id_;
  PContainer::prop_id_t devices_id_{0};
  std::vector<int> devices_nv_ids_{};
  Selection<> codecs_{{"none"}, 0};
  std::vector<std::pair<std::string, GUID>> codecs_guids_{};
  PContainer::prop_id_t codecs_id_{0};
  Selection<> presets_{{"none"}, 0};
  std::vector<std::pair<std::string, GUID>> presets_guids_{};
  PContainer::prop_id_t presets_id_{0};
  Selection<> profiles_{{"none"}, 0};
  std::vector<std::pair<std::string, GUID>> profiles_guids_{};
  PContainer::prop_id_t profiles_id_{0};
  int max_width_{0};
  PContainer::prop_id_t max_width_id_{0};
  int max_height_{0};
  PContainer::prop_id_t max_height_id_{0};
  std::vector<std::pair<std::string, NV_ENC_BUFFER_FORMAT>> video_formats_{};
  std::unique_ptr<ShmdataWriter> shmw_{nullptr};
  std::unique_ptr<ShmdataFollower> shm_{nullptr};
  ShmdataConnector shmcntr_;
  void update_device();
  void update_codec();
  void update_preset();
  void update_profile();
  void update_max_width_height();
  void update_input_formats();

  bool on_shmdata_disconnect();
  bool on_shmdata_connect(const std::string& shmdata_sochet_path);
  bool can_sink_caps(const std::string& caps);
  void on_shmreader_data(void* data, size_t data_size);
  void on_shmreader_server_connected(const std::string& data_descr);
};

SWITCHER_DECLARE_PLUGIN(NVencPlugin);

}  // namespace switcher
#endif
