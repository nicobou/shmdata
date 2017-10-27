/*
 * This file is part of switcher-nvdec.
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

#include "./nvdec-decode-session.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/threaded-wrapper.hpp"

namespace switcher {
class NVdecPlugin : public Quiddity {
 public:
  NVdecPlugin(QuiddityConfiguration&&);
  ~NVdecPlugin() = default;
  NVdecPlugin(const NVdecPlugin&) = delete;
  NVdecPlugin& operator=(const NVdecPlugin&) = delete;


 private:
  static const std::array<const char*, 5> kSupportedCodecs;

  std::unique_ptr<ShmdataWriter> shmwriter_{nullptr};
  std::unique_ptr<ShmdataFollower> shmfollower_{nullptr};
  ShmdataConnector shmcntr_;
  std::unique_ptr<ThreadedWrapper<NVencDS>> ds_{nullptr};
  Selection<> devices_{{"none"}, 0};
  PContainer::prop_id_t devices_id_{0};
  std::vector<int> devices_nv_ids_{};
  cudaVideoCodec video_codec_{cudaVideoCodec_NumCodecs};
  std::string caps_{};
  size_t writer_size_{0};

  static cudaVideoCodec convert_video_type_to_cuda(const std::string& content_type,
                                                   const std::string& codec,
                                                   guint mpeg_version);

  bool on_shmdata_disconnect();
  bool on_shmdata_connect(const std::string& shmdata_sochet_path);
  bool can_sink_caps(const std::string& caps);
  void on_shmreader_data(void* data, size_t data_size);
  void on_shmreader_server_connected(const std::string& data_descr);
  void on_shmreader_server_disconnected();
};

SWITCHER_DECLARE_PLUGIN(NVdecPlugin);

}  // namespace switcher
#endif
