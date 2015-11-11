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
#include "switcher/quiddity.hpp"
#include "switcher/threaded-wrapper.hpp"
#include "./nvenc-encode-session.hpp"

namespace switcher {
class NVencPlugin: public Quiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(NVencPlugin);
  NVencPlugin(const std::string &);
  ~NVencPlugin() = default;
  NVencPlugin(const NVencPlugin &) = delete;
  NVencPlugin &operator=(const NVencPlugin &) = delete;

  bool init() final;
 private:
  std::unique_ptr<ThreadedWrapper<NVencES>> es_{};
  Selection devices_{{"none"}, 0};
  std::vector<int> devices_nv_ids_{};
  Selection codecs_{{"none"}, 0};
  std::vector<std::pair<std::string, GUID>> codecs_guids_{};
  PContainer::prop_id_t codecs_id_{0};
  Selection presets_{{"none"}, 0};
  std::vector<std::pair<std::string, GUID>> presets_guids_{};
  PContainer::prop_id_t presets_id_{0};
  Selection profiles_{{"none"}, 0};
  std::vector<std::pair<std::string, GUID>> profiles_guids_{};
  PContainer::prop_id_t profiles_id_{0};
  void update_device();
  void update_codec();
  void update_preset();
  void update_profile();
};

SWITCHER_DECLARE_PLUGIN(NVencPlugin);

}  // namespace switcher
#endif
