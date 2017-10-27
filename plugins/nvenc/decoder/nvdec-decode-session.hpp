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

#ifndef __SWITCHER_NVENC_DECODE_SESSION_H__
#define __SWITCHER_NVENC_DECODE_SESSION_H__

#include <nvcuvid.h>
#include <cstdint>  // uint32_t
#include <memory>
#include <utility>
#include <vector>
#include "cuda/cuda-context.hpp"
#include "switcher/logged.hpp"
#include "switcher/safe-bool-idiom.hpp"

namespace switcher {
class NVencDS : public Logged, public SafeBoolIdiom {
 public:
  NVencDS(uint32_t device_id, cudaVideoCodec codec, BaseLogger* log);
  NVencDS(const NVencDS&) = delete;
  NVencDS(NVencDS&&) = delete;
  NVencDS() = delete;
  NVencDS& operator=(const NVencDS&) = delete;
  NVencDS& operator=(NVencDS&&) = delete;
  ~NVencDS();
  bool safe_bool_idiom() const { return (nullptr != video_decoder_ && nullptr != video_parser_); }
  void parse_data(std::function<void(CUvideoparser)> parse_func) { parse_func(video_parser_); }

  void process_decoded(std::function<void(const unsigned char* data_decoded,
                                          unsigned int data_width,
                                          unsigned int data_height,
                                          unsigned int pitch,
                                          bool& scaled)> post_process_func) {
    post_process_func(
        (const unsigned char*)bitstream_, frame_width_, frame_height_, pitch_, scaled_);
  }

  static int CUDAAPI HandleVideoSequence(void* pUserData, CUVIDEOFORMAT* pFormat);
  static int CUDAAPI HandlePictureDecode(void* pUserData, CUVIDPICPARAMS* pPicParams);
  static int CUDAAPI HandlePictureDisplay(void* pUserData, CUVIDPARSERDISPINFO* pPicParams);

 private:
  static const unsigned long kNumDecodeSurfaces{8};
  CUvideodecoder video_decoder_{nullptr};
  CUVIDDECODECREATEINFO video_info_;
  cudaVideoCreateFlags video_flags{};
  CUvideoparser video_parser_{nullptr};
  CudaContext cu_ctx_;
  void* bitstream_{nullptr};
  unsigned int frame_width_{0};
  unsigned int frame_height_{0};
  unsigned int pitch_{0};
  bool scaled_{false};
};
}  // namespace switcher
#endif
