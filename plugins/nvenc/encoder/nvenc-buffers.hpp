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

#ifndef __SWITCHER_NVENC_BUFFER_H__
#define __SWITCHER_NVENC_BUFFER_H__

#include <algorithm>
#include <array>
#include <functional>
#include "./nvenc-api.hpp"
#include "switcher/safe-bool-idiom.hpp"

namespace switcher {
class NVencBuffers : public SafeBoolIdiom {
 public:
  NVencBuffers(void* encoder, uint32_t width, uint32_t height, NV_ENC_BUFFER_FORMAT format);
  ~NVencBuffers();
  NVencBuffers(const NVencBuffers&) = delete;
  NVencBuffers(NVencBuffers&&) = delete;
  NVencBuffers& operator=(const NVencBuffers&) = delete;
  NVencBuffers& operator=(NVencBuffers&&) = delete;

  bool copy_to_next_input_buffer(void* data, size_t size);
  bool encode_current_input();
  bool process_encoded_frame(std::function<void(void*, uint32_t)> fun);

 private:
  NV_ENC_CREATE_INPUT_BUFFER input_buf_param_;
  NV_ENC_CREATE_BITSTREAM_BUFFER output_buf_param_;
  NV_ENC_LOCK_INPUT_BUFFER lock_input_buffer_params_;
  NV_ENC_PIC_PARAMS enc_pic_params_;
  NV_ENC_LOCK_BITSTREAM lock_bitstream_data_;
  void* encoder_;  // must remain valid during NVencBuffers lifetime
  uint32_t width_;
  uint32_t height_;
  NV_ENC_BUFFER_FORMAT format_;
  static const unsigned int kNumBuf{48};
  std::array<NV_ENC_INPUT_PTR, kNumBuf> input_bufs_{{}};
  unsigned int cur_buf_{0};
  NV_ENC_INPUT_PTR next_input_{nullptr};
  uint32_t timestamp_{0};
  std::array<NV_ENC_OUTPUT_PTR, kNumBuf> output_bufs_{{}};
  NV_ENC_INPUT_PTR next_output_{nullptr};
  bool safe_bool_idiom() const;
};

}  // namespace switcher
#endif
