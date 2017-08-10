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
#include <memory>
#include <utility>
#include <vector>
#include "./nvenc-api.hpp"
#include "./nvenc-buffers.hpp"
#include "cuda/cuda-context.hpp"
#include "switcher/logged.hpp"
#include "switcher/safe-bool-idiom.hpp"

namespace switcher {
class NVencES : public Logged, public SafeBoolIdiom {
 public:
  NVencES(uint32_t device_id, BaseLogger* log);
  NVencES(BaseLogger* log) : NVencES(0, log) {}
  NVencES() = delete;
  ~NVencES();
  NVencES(const NVencES&) = delete;
  NVencES(NVencES&&) = delete;
  NVencES& operator=(const NVencES&) = delete;
  NVencES& operator=(NVencES&&) = delete;

  using named_guid_t = std::vector<std::pair<std::string, GUID>>;
  using input_format_t = std::vector<std::pair<std::string, NV_ENC_BUFFER_FORMAT>>;
  named_guid_t get_supported_codecs();
  named_guid_t get_presets(GUID encodeGUID);
  named_guid_t get_profiles(GUID encodeGUID);
  input_format_t get_input_formats(GUID encodeGUID);
  std::pair<int, int> get_max_width_height(GUID encodeGUID);
  bool safe_bool_idiom() const { return nullptr != encoder_; }
  bool initialize_encoder(GUID encodeGuid,
                          GUID presetGuid,
                          GUID profileGuid,
                          uint32_t bitrate,
                          uint32_t width,
                          uint32_t height,
                          uint32_t frameRateNum,
                          uint32_t framerateDen,
                          NV_ENC_BUFFER_FORMAT format);
  // these three following methods must be invoked in sequence:
  bool copy_to_next_input_buffer(void* data, size_t size);
  bool encode_current_input();
  bool process_encoded_frame(std::function<void(void*, uint32_t)> fun);

 private:
  uint32_t device_id_;
  NV_ENC_PRESET_CONFIG preset_config_;
  NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS params_;
  NV_ENC_CAPS_PARAM caps_params_{NV_ENC_CAPS_PARAM_VER, NV_ENC_CAPS_WIDTH_MAX, {}};
  static const size_t kArraySize{64};
  GUID encode_GUID_;
  uint32_t codecs_num_{0};
  GUID codecs_guids_[kArraySize];
  uint32_t presets_num_{0};
  GUID preset_GUID_;
  GUID presets_guids_[kArraySize];
  GUID profile_GUID_;
  uint32_t profiles_num_{0};
  GUID profiles_guids_[kArraySize];
  GUID input_formats_GUID_;
  uint32_t input_formats_num_{0};
  NV_ENC_BUFFER_FORMAT buf_formats_[kArraySize];
  GUID max_width_height_GUID_;
  int max_width_{0};
  int max_height_{0};
  NVencAPI api_{};
  void* encoder_{nullptr};
  NV_ENC_INITIALIZE_PARAMS init_params_;
  CudaContext cu_ctx_;
  std::unique_ptr<NVencBuffers> buffers_{nullptr};
  static bool is_same(const GUID& g1, const GUID& g2);
};

}  // namespace switcher
#endif
