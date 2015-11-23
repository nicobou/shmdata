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

#include <glib.h>  // log
//#include <gst/gst.h>
#include <cstring> // memset
#include <iostream> // TODO remove
#include "./nvenc-buffers.hpp"

namespace switcher {
NVencBuffers::NVencBuffers(void *encoder,
                           uint32_t width,
                           uint32_t height,
                           NV_ENC_BUFFER_FORMAT format):
    encoder_(encoder),
    width_(width),
    height_(height),
    format_(format){
  // input buffers
  for (auto &it: input_bufs_){
    NV_ENC_CREATE_INPUT_BUFFER buf;
    buf.version = NV_ENC_CREATE_INPUT_BUFFER_VER;
    buf.width = width;  // FIXME find puissnce de 2 superieure a width
    buf.height = height;
    buf.memoryHeap = NV_ENC_MEMORY_HEAP_SYSMEM_CACHED;
    buf.bufferFmt = format;
    if (NV_ENC_SUCCESS != NVencAPI::api.nvEncCreateInputBuffer (encoder_, &buf)){
      g_warning("nvenc cannot create input buffer");
      return;
    }
    it = buf.inputBuffer;
  }
  // output buffers
  for (auto &it: output_bufs_){
    NV_ENC_CREATE_BITSTREAM_BUFFER buf;
    buf.version = NV_ENC_CREATE_BITSTREAM_BUFFER_VER;
    /* 20 MB should be large enough to hold most output frames.
     * NVENC will automatically increase this if it's not enough. */
    buf.size = 20 * 1024 * 1024;
    buf.memoryHeap = NV_ENC_MEMORY_HEAP_SYSMEM_CACHED;
    if (NV_ENC_SUCCESS != NVencAPI::api.nvEncCreateBitstreamBuffer (encoder_, &buf)){
      g_warning("nvenc cannot create output buffer");
      return;
    }
    it = buf.bitstreamBuffer;
  }
}

NVencBuffers::~NVencBuffers(){
  // input buffers
  for (auto &it: input_bufs_)
    if (nullptr != it &&
        NV_ENC_SUCCESS != NVencAPI::api.nvEncDestroyInputBuffer (encoder_, it))
      g_warning("nvenc cannot destroy input buffer");
  // output buffers
  for (auto &it: output_bufs_)
    if (nullptr != it &&
        NV_ENC_SUCCESS != NVencAPI::api.nvEncDestroyBitstreamBuffer (encoder_, it))
      g_warning("nvenc cannot destroy output buffer");
}

bool NVencBuffers::safe_bool_idiom() const {
  return
      input_bufs_.end() == std::find(input_bufs_.begin(), input_bufs_.end(),
                                     nullptr)
      && output_bufs_.end() == std::find(output_bufs_.begin(), output_bufs_.end(),
                                         nullptr);
}

void convertYUVpitchtoYUV444(unsigned char *yuv_luma, unsigned char *yuv_cb, unsigned char *yuv_cr,
    unsigned char *surf_luma, unsigned char *surf_cb, unsigned char *surf_cr, int width, int height, int srcStride, int dstStride)
{
    int h;

    for (h = 0; h < height; h++)
    {
        memcpy(surf_luma + dstStride * h, yuv_luma + srcStride * h, width);
        memcpy(surf_cb + dstStride * h, yuv_cb + srcStride * h, width);
        memcpy(surf_cr + dstStride * h, yuv_cr + srcStride * h, width);
    }
}

bool NVencBuffers::copy_to_next_input_buffer(void *data, size_t size){
  if (nullptr != next_input_)
    return false;
  NV_ENC_LOCK_INPUT_BUFFER lockInputBufferParams;
  memset(&lockInputBufferParams, 0, sizeof(lockInputBufferParams));
  lockInputBufferParams.version = NV_ENC_LOCK_INPUT_BUFFER_VER;
  
  lockInputBufferParams.inputBuffer = input_bufs_[cur_buf_];
  if (NV_ENC_SUCCESS != NVencAPI::api.nvEncLockInputBuffer(encoder_, &lockInputBufferParams))
    return false;
  // uint32_t lockedPitch = lockInputBufferParams.pitch;
  // unsigned char *pInputSurface = (unsigned char *)lockInputBufferParams.bufferDataPtr;
  if (format_ == NV_ENC_BUFFER_FORMAT_NV12_PL){
    guint8 *src = (guint8 *)data;
    guint src_stride = width_;
    guint8 *dest = (guint8 *) lockInputBufferParams.bufferDataPtr;
    guint dest_stride = lockInputBufferParams.pitch;
    for (guint y = 0; y < height_; ++y) {
      memcpy (dest, src, width_);
      dest += dest_stride;
      src += src_stride;
    }
    for (guint y = 0; y < height_ / 2; ++y) {
      memcpy (dest, src, width_);
      dest += dest_stride;
      src += src_stride;
    }
  } else {
    // HERE
    // if (size != width_ * height_ * 2)
    //   g_print("coucou size %lu, width %u, height %u \n", size, width_, height_);
    guint8 *src = (guint8 *)data;
    guint src_stride = width_;
    guint8 *dest = (guint8 *) lockInputBufferParams.bufferDataPtr;
    guint dest_stride = lockInputBufferParams.pitch;
    for (guint y = 0; y < height_; ++y) {
      memcpy (dest, src, width_);
      dest += dest_stride;
      src += src_stride;
    }
    for (guint y = 0; y < height_; ++y) {
      memcpy (dest, src, width_/2);
      dest += dest_stride;
      src += src_stride;
    }
    //     unsigned char *pInputSurfaceCb =
    //     pInputSurface + (height_ * lockedPitch);
    // unsigned char *pInputSurfaceCr =
    //     pInputSurfaceCb + (height_ * lockedPitch);
    // convertYUVpitchtoYUV444((unsigned char *)data,
    //                         (unsigned char *)data + size/3,
    //                         (unsigned char *)data + size * 2 / 3,
    //                         pInputSurface,
    //                         pInputSurfaceCb,
    //                         pInputSurfaceCr,
    //                         width_,
    //                         height_,
    //                         width_,
    //                         lockedPitch);
  }
  if (NV_ENC_SUCCESS !=
      NVencAPI::api.nvEncUnlockInputBuffer(encoder_, input_bufs_[cur_buf_]))
    return false;
  next_input_ = input_bufs_[cur_buf_];
  next_output_ = output_bufs_[cur_buf_];
  cur_buf_ = (cur_buf_ + 1) % num_buf_;
  return true;  
}

bool NVencBuffers::encode_current_input(){
  if (nullptr == next_input_)
    return false;
  NV_ENC_PIC_PARAMS encPicParams;
  memset(&encPicParams, 0, sizeof(encPicParams));
  encPicParams.version = NV_ENC_PIC_PARAMS_VER;
  encPicParams.inputBuffer = next_input_;
  encPicParams.bufferFmt = format_;
  encPicParams.inputWidth = width_;
  encPicParams.inputHeight = height_;
  encPicParams.inputPitch = width_;
  encPicParams.outputBitstream = next_output_;
  encPicParams.inputTimeStamp = timestamp_;
  encPicParams.pictureStruct = NV_ENC_PIC_STRUCT_FRAME;
  encPicParams.encodePicFlags = NV_ENC_PIC_FLAG_OUTPUT_SPSPPS;
  NVENCSTATUS nvStatus = NVencAPI::api.nvEncEncodePicture(encoder_, &encPicParams);
  if (nvStatus != NV_ENC_SUCCESS) {
    return false;
  }
  ++timestamp_;
  return true;
}

bool NVencBuffers::process_encoded_frame(std::function<void(void *, uint32_t)> fun){
 if (nullptr == next_output_)
    return false;
  NV_ENC_LOCK_BITSTREAM lockBitstreamData;
  memset(&lockBitstreamData, 0, sizeof(lockBitstreamData));
  lockBitstreamData.version = NV_ENC_LOCK_BITSTREAM_VER;
  lockBitstreamData.outputBitstream = next_output_;
  lockBitstreamData.doNotWait = false;
  NVENCSTATUS nvStatus = NVencAPI::api.nvEncLockBitstream(encoder_, &lockBitstreamData);
  if (nvStatus == NV_ENC_SUCCESS) {
    fun(lockBitstreamData.bitstreamBufferPtr, lockBitstreamData.bitstreamSizeInBytes);
    nvStatus = NVencAPI::api.nvEncUnlockBitstream(encoder_, next_output_);
  }
  next_input_ = nullptr;
  next_output_ = nullptr;
  return true;
}

}  // namespace switcher
