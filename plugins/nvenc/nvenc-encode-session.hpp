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

#ifndef __SWITCHER_NVENC_ENCODE_SESSION_H__
#define __SWITCHER_NVENC_ENCODE_SESSION_H__

#include <cstdint>  // uint32_t
#include <vector>
#include <memory>
#include <utility>
#include "switcher/safe-bool-idiom.hpp"
#include "./nvenc-api.hpp"
#include "./cuda-context.hpp"
#include "./nvenc-buffers.hpp"

namespace switcher {
class NVencES: public SafeBoolIdiom {
 public:
  NVencES(uint32_t device_id);
  NVencES(): NVencES (0){}
  ~NVencES();
  NVencES(const NVencES &) = delete;
  NVencES(NVencES &&) = delete;
  NVencES &operator=(const NVencES &) = delete;
  NVencES &operator=(NVencES &&) = delete;

  std::vector<std::pair<std::string, GUID>> get_supported_codecs();  
  std::vector<std::pair<std::string, GUID>> get_presets(GUID encodeGUID);
  std::vector<std::pair<std::string, GUID>> get_profiles(GUID encodeGUID);
  std::vector<std::pair<std::string, NV_ENC_BUFFER_FORMAT>> get_input_formats(GUID encodeGUID);

  std::pair<int,int> get_max_width_height(GUID encodeGUID);
  bool safe_bool_idiom() const {return nullptr != encoder_;}
  bool initialize_encoder(GUID encodeGuid, GUID presetGuid,
                          uint32_t width, uint32_t height,
                          NV_ENC_BUFFER_FORMAT format);

  // these three following methods must be invoked in sequence:
  bool copy_to_next_input_buffer(void *data, size_t size);
  bool encode_current_input();
  bool process_encoded_frame(std::function<void(void *, uint32_t)> fun);

 private:
  NVencAPI api_{};
  void *encoder_{nullptr};
  NV_ENC_INITIALIZE_PARAMS init_params_;
  CudaContext cu_ctx_;
  std::unique_ptr<NVencBuffers> buffers_{nullptr};
  static bool is_same(GUID g1, GUID g2);
};

}  // namespace switcher
#endif
