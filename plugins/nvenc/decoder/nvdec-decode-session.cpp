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
#include "./nvdec-decode-session.hpp"
#include <glib.h>  // log
#include <cmath>
#include <cstring>  // memset
#include "switcher/scope-exit.hpp"

#include <iostream>
namespace switcher {

NVencDS::NVencDS(uint32_t device_id, cudaVideoCodec codec, BaseLogger* log)
    : Logged(log), cu_ctx_(device_id, log) {
  On_scope_exit {
    if (!safe_bool_idiom()) warning("NV decoder session initialization failed");
  };

  memset(&video_info_, 0, sizeof(CUVIDDECODECREATEINFO));

  // Arbitrary initial values, will change when receiving data.
  video_info_.CodecType = codec;
  video_info_.ulWidth = 4096;
  video_info_.ulHeight = 4096;
  video_info_.ChromaFormat = cudaVideoChromaFormat_420;

  video_info_.ulNumDecodeSurfaces = kNumDecodeSurfaces;
  video_info_.OutputFormat = cudaVideoSurfaceFormat_NV12;
  video_info_.DeinterlaceMode = cudaVideoDeinterlaceMode_Adaptive;
  video_info_.ulCreationFlags = cudaVideoCreate_PreferCUVID;
  video_info_.ulNumOutputSurfaces = kNumDecodeSurfaces;

  // No scaling
  video_info_.ulTargetWidth = video_info_.ulWidth;
  video_info_.ulTargetHeight = video_info_.ulHeight;

  // Create the decoder
  if (CUDA_SUCCESS != cuvidCreateDecoder(&video_decoder_, &video_info_)) {
    video_decoder_ = nullptr;
    return;
  }

  {
    CUVIDPARSERPARAMS video_parser_params;
    memset(&video_parser_params, 0, sizeof(CUVIDPARSERPARAMS));
    video_parser_params.CodecType = codec;
    video_parser_params.ulMaxNumDecodeSurfaces = kNumDecodeSurfaces;
    video_parser_params.pUserData = this;

    // Needed so that the parser will push frames out to the decoder as quickly as it can.
    video_parser_params.ulMaxDisplayDelay = 1;

    // Called before decoding frames and / or whenever there is a format change
    video_parser_params.pfnSequenceCallback = HandleVideoSequence;
    // Called when a picture is ready to be decoded (decode order)
    video_parser_params.pfnDecodePicture = HandlePictureDecode;
    // Called whenever a picture is ready to be displayed (display order)
    video_parser_params.pfnDisplayPicture = HandlePictureDisplay;

    // Create the parser
    if (CUDA_SUCCESS != cuvidCreateVideoParser(&video_parser_, &video_parser_params)) {
      video_parser_ = nullptr;
      return;
    }
  }
}

NVencDS::~NVencDS() {
  if (video_parser_ && CUDA_SUCCESS != cuvidDestroyVideoParser(video_parser_))
    warning("Error while destroying NV video parser (nvdec)");

  if (video_decoder_ && CUDA_SUCCESS != cuvidDestroyDecoder(video_decoder_))
    warning("Error while destroying NV video decoder (nvdec)");

  if (bitstream_) {
    cuMemFreeHost(bitstream_);
  }
}

int CUDAAPI NVencDS::HandleVideoSequence(void* user_data, CUVIDEOFORMAT* format) {
  NVencDS* ds = static_cast<NVencDS*>(user_data);

  if ((format->codec != ds->video_info_.CodecType) ||
      (format->coded_width != ds->video_info_.ulWidth) ||
      (format->coded_height != ds->video_info_.ulHeight) ||
      (format->chroma_format != ds->video_info_.ChromaFormat)) {
    if (ds->video_decoder_) {
      auto status = cuvidDestroyDecoder(ds->video_decoder_);
      ds->video_decoder_ = nullptr;
      if (status != CUDA_SUCCESS) return 0;
    }
    ds->video_info_.ulWidth = format->coded_width;
    ds->video_info_.ulHeight = format->coded_height;
    ds->video_info_.CodecType = format->codec;
    ds->video_info_.ChromaFormat = format->chroma_format;

    // FIXME: Scaling in case it's a 16:9 format
    // See post on NVIDIA forum at https://goo.gl/HQRkCz
    if (std::abs(static_cast<float>(ds->video_info_.ulWidth) /
                     static_cast<float>(ds->video_info_.ulHeight) -
                 16.0f / 9.0f) < 0.001f) {
      if (ds->video_info_.ulWidth <= 1280) {
        ds->video_info_.ulTargetWidth = 1280;
        ds->video_info_.ulTargetHeight = 720;
      } else if (ds->video_info_.ulWidth <= 2048) {
        ds->video_info_.ulTargetWidth = 2048;
        ds->video_info_.ulTargetHeight = 1152;
      } else if (ds->video_info_.ulWidth <= 3072) {
        ds->video_info_.ulTargetWidth = 3072;
        ds->video_info_.ulTargetHeight = 1728;
      } else if (ds->video_info_.ulWidth <= 4096) {
        ds->video_info_.ulTargetWidth = 4096;
        ds->video_info_.ulTargetHeight = 2304;
      }

      ds->scaled_ = true;

    } else {
      ds->video_info_.ulTargetWidth = ds->video_info_.ulWidth;
      ds->video_info_.ulTargetHeight = ds->video_info_.ulHeight;
    }

    ds->frame_width_ = ds->video_info_.ulTargetWidth;
    ds->frame_height_ = ds->video_info_.ulTargetHeight;

    // If the format changed, we re-create the buffer to be safe.
    if (ds->bitstream_) {
      cuMemFreeHost(ds->bitstream_);
      ds->bitstream_ = nullptr;
    }

    // Create the decoder
    if (CUDA_SUCCESS != cuvidCreateDecoder(&ds->video_decoder_, &ds->video_info_)) {
      ds->video_decoder_ = nullptr;
      return 0;
    }
  }

  return 1;
}

int CUDAAPI NVencDS::HandlePictureDecode(void* user_data, CUVIDPICPARAMS* picture_params) {
  NVencDS* ds = static_cast<NVencDS*>(user_data);
  cuvidDecodePicture(ds->video_decoder_, picture_params);
  return 1;
}

int CUDAAPI NVencDS::HandlePictureDisplay(void* user_data, CUVIDPARSERDISPINFO* picture_params) {
  NVencDS* ds = static_cast<NVencDS*>(user_data);
  CUdeviceptr device_pointer;
  CUVIDPROCPARAMS vpp;
  memset(&vpp, 0, sizeof(vpp));
  vpp.progressive_frame = picture_params->progressive_frame;
  vpp.top_field_first = picture_params->top_field_first;

  // Called to gain access to the device memory pointer and get frame informations
  cuvidMapVideoFrame(
      ds->video_decoder_, picture_params->picture_index, &device_pointer, &ds->pitch_, &vpp);

  unsigned int decoded_size =
      ds->pitch_ * ds->video_info_.ulTargetHeight * 3 / 2;  // Only NV12 for now, 12bpp.
  if (ds->bitstream_ == nullptr) {
    cuMemAllocHost(&ds->bitstream_, decoded_size);
  }

  // Copy the data from the GPU to host memory
  cuMemcpyDtoH(ds->bitstream_, device_pointer, decoded_size);

  cuvidUnmapVideoFrame(ds->video_decoder_, device_pointer);

  return 1;
}
}  // namespace switcher
