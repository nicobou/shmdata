/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_FILEDECODER_H__
#define __SWITCHER_FILEDECODER_H__

#include <memory>
#include <unordered_map>
#include "../gst/decodebin-to-shmdata.hpp"
#include "../gst/pipeliner.hpp"
#include "../quiddity/quiddity.hpp"
#include "../shmdata/gst-shm-tree-updater.hpp"
#include "../utils/counter-map.hpp"
#include "../utils/periodic-task.hpp"

namespace switcher {
namespace quiddities {
class FileDecoder : public Quiddity {
 public:
  FileDecoder(quid::Config&&);

 private:
  bool load_file(const std::string& path);
  void configure_shmdatasink(GstElement* element,
                             const std::string& media_type,
                             const std::string& media_label);
  // internals
  std::condition_variable media_loaded_cond_{};
  std::mutex media_loaded_mutex_{};
  bool media_loaded_{false};
  CounterMap counter_{};
  std::vector<std::unique_ptr<GstShmTreeUpdater>> shm_subs_{};
  GstElement* filesrc_{nullptr};
  // custom properties
  std::string location_{};
  PContainer::prop_id_t location_id_;
  bool play_{false};
  PContainer::prop_id_t play_id_;
  double cur_pos_{0};
  PContainer::prop_id_t cur_pos_id_;
  bool loop_{false};
  PContainer::prop_id_t loop_id_;
  double speed_{1.0};
  PContainer::prop_id_t speed_id_;
  std::unique_ptr<gst::DecodebinToShmdata> decodebin_{nullptr};
  std::unique_ptr<gst::Pipeliner> gst_pipeline_{nullptr};
  std::unique_ptr<PeriodicTask<>> position_task_{};
};

}  // namespace quiddities
}  // namespace switcher
#endif
