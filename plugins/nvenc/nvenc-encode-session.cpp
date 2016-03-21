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
#include <cstring>  // memset
#include "./nvenc-encode-session.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/std2.hpp"

namespace switcher {
NVencES::NVencES(uint32_t device_id
                 //, uint32_t width, uint32_t height,  NV_ENC_BUFFER_FORMAT format
                 ):
    cu_ctx_(device_id){
  On_scope_exit{
    if (!safe_bool_idiom())
      g_warning("NV encoder session initialization failed");
  };
  if (!cu_ctx_)
    return;
  NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS params;
  params.version = NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER;
  params.apiVersion = NVENCAPI_VERSION;
  params.device = cu_ctx_.cuda_ctx_;
  params.deviceType = NV_ENC_DEVICE_TYPE_CUDA;
  if (NV_ENC_SUCCESS != NVencAPI::api.nvEncOpenEncodeSessionEx(&params, &encoder_)) {
      encoder_ = nullptr;
      return;
  }
}

NVencES::~NVencES(){
  if (safe_bool_idiom() && !(NV_ENC_SUCCESS == NVencAPI::api.nvEncDestroyEncoder(encoder_)))
    g_warning("BUG! (destroying NV encoder session)");
}

std::vector<std::pair<std::string, GUID>> NVencES::get_supported_codecs(){
  std::vector<std::pair<std::string, GUID>> res;
  // supported codecs
  uint32_t i, num = 0;
  GUID guids[16];
  if (NV_ENC_SUCCESS != NVencAPI::api.nvEncGetEncodeGUIDs (encoder_, guids, 16, &num)){
    g_warning("issue with nvEncGetEncodeGUIDs");
    return res;
  }
  for (i = 0; i < num; ++i) {
    if (is_same(guids[i], NV_ENC_CODEC_H264_GUID)){
      res.push_back(std::make_pair(std::string("H264"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_CODEC_HEVC_GUID)) {
      res.push_back(std::make_pair(std::string("HEVC"), guids[i]));
    } else
      g_warning("unknown codec GUID from nvenc");
  }
  return res;
}

std::vector<std::pair<std::string, GUID>> NVencES::get_presets(GUID encodeGUID){
  std::vector<std::pair<std::string, GUID>> res;
  // supported codecs
  uint32_t i, num = 0;
  GUID guids[16];
  if (NV_ENC_SUCCESS !=
      NVencAPI::api.nvEncGetEncodePresetGUIDs(encoder_, encodeGUID,
                                              guids, 16, &num)){
    g_warning("issue with nvEncGetEncodePresetGUIDs");
    return res;
  }
  for (i = 0; i < num; ++i) {
    if (is_same(guids[i], NV_ENC_PRESET_DEFAULT_GUID)){
      res.push_back(std::make_pair(std::string("Default"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_PRESET_HP_GUID)) {
      res.push_back(std::make_pair(std::string("HP"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_PRESET_HQ_GUID)) {
      res.push_back(std::make_pair(std::string("HQ"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_PRESET_BD_GUID)) {
      res.push_back(std::make_pair(std::string("BD"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_PRESET_LOW_LATENCY_DEFAULT_GUID)) {
      res.push_back(std::make_pair(std::string("Low Latency default"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_PRESET_LOW_LATENCY_HQ_GUID)) {
      res.push_back(std::make_pair(std::string("Low Latency HQ"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_PRESET_LOW_LATENCY_HP_GUID)) {
      res.push_back(std::make_pair(std::string("Low Latency HP"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_PRESET_LOSSLESS_DEFAULT_GUID)) {
      res.push_back(std::make_pair(std::string("Lossless default"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_PRESET_LOSSLESS_HP_GUID)) {
      res.push_back(std::make_pair(std::string("Lossless HP"), guids[i]));
    } else
      g_warning("unknown preset GUID from nvenc");
  }
  return res;
}

std::vector<std::pair<std::string, GUID>> NVencES::get_profiles(GUID encodeGUID){
  std::vector<std::pair<std::string, GUID>> res;
  // supported codecs
  uint32_t i, num = 0;
  GUID guids[16];
  NVencAPI::api.nvEncGetEncodeProfileGUIDs (encoder_, encodeGUID, guids, 16, &num);
  for (i = 0; i < num; ++i) {
    if (is_same(guids[i], NV_ENC_CODEC_PROFILE_AUTOSELECT_GUID)){
      res.push_back(std::make_pair(std::string("Autoselect"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_H264_PROFILE_BASELINE_GUID)) {
      res.push_back(std::make_pair(std::string("Baseline"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_H264_PROFILE_MAIN_GUID)) {
      res.push_back(std::make_pair(std::string("Main (H264)"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_H264_PROFILE_HIGH_GUID)) {
      res.push_back(std::make_pair(std::string("High"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_H264_PROFILE_HIGH_444_GUID)) {
      res.push_back(std::make_pair(std::string("High 444"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_H264_PROFILE_STEREO_GUID)) {
      res.push_back(std::make_pair(std::string("Stereo"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_H264_PROFILE_SVC_TEMPORAL_SCALABILTY)) {
      res.push_back(std::make_pair(std::string("SVC temporal scalability"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_H264_PROFILE_CONSTRAINED_HIGH_GUID)) {
      res.push_back(std::make_pair(std::string("Constrained high"), guids[i]));
    } else if (is_same(guids[i], NV_ENC_HEVC_PROFILE_MAIN_GUID)) {
      res.push_back(std::make_pair(std::string("Main (HEVC)"), guids[i]));
    } else
      g_warning("unknown profile GUID from nvenc");
  }
  return res;
}

std::vector<std::pair<std::string, NV_ENC_BUFFER_FORMAT>>
    NVencES::get_input_formats(GUID encodeGUID){
  std::vector<std::pair<std::string, NV_ENC_BUFFER_FORMAT>> res;
  // supported codecs
  uint32_t i, num = 0;
  NV_ENC_BUFFER_FORMAT buf_format[64];
  NVencAPI::api.nvEncGetInputFormats (encoder_, encodeGUID, buf_format, 16, &num);
  for (i = 0; i < num; ++i) {
    if (NV_ENC_BUFFER_FORMAT_UNDEFINED == buf_format[i]) {
      //res.push_back(std::make_pair(std::string("Undefined"), buf_format[i]));
      g_warning("nvEncGetInputFormats gives NV_ENC_BUFFER_FORMAT_UNDEFINED (?)");
    } else if (NV_ENC_BUFFER_FORMAT_NV12_PL == buf_format[i]) {
      res.push_back(std::make_pair(std::string("NV12_PL"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_NV12_TILED16x16 == buf_format[i]) {
      res.push_back(std::make_pair(std::string("NV12_TILED16x16"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_NV12_TILED64x16 == buf_format[i]) {
      res.push_back(std::make_pair(std::string("NV12_TILED64x16"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_YV12_PL == buf_format[i]) {
      res.push_back(std::make_pair(std::string("YV12_PL"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_YV12_TILED16x16 == buf_format[i]) {
      res.push_back(std::make_pair(std::string("YV12_TILED16x16"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_YV12_TILED64x16 == buf_format[i]) {
      res.push_back(std::make_pair(std::string("YV12_TILED64x16"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_IYUV_PL == buf_format[i]) {
      res.push_back(std::make_pair(std::string("IYUV_PL"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_IYUV_TILED16x16 == buf_format[i]) {
      res.push_back(std::make_pair(std::string("IYUV_TILED16x16"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_IYUV_TILED64x16 == buf_format[i]) {
      res.push_back(std::make_pair(std::string("IYUV_TILED64x16"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_YUV444_PL == buf_format[i]) {
      res.push_back(std::make_pair(std::string("YUV444_PL"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_YUV444_TILED16x16 == buf_format[i]) {
      res.push_back(std::make_pair(std::string("YUV444_TILED16x16"), buf_format[i]));
    } else if (NV_ENC_BUFFER_FORMAT_YUV444_TILED64x16 == buf_format[i]) {
      res.push_back(std::make_pair(std::string("YUV444_TILED64x16"), buf_format[i]));
    } else
      g_warning("unknown input format from nvenc");
  }
  return res;
}

std::pair<int,int> NVencES::get_max_width_height(GUID encodeGUID){
  NV_ENC_CAPS_PARAM params =
      { NV_ENC_CAPS_PARAM_VER,
        NV_ENC_CAPS_WIDTH_MAX,
        {}};
  int width=0, height = 0;
  NVencAPI::api.nvEncGetEncodeCaps(encoder_, encodeGUID, &params, &width);
  params.capsToQuery = NV_ENC_CAPS_HEIGHT_MAX;
  NVencAPI::api.nvEncGetEncodeCaps(encoder_, encodeGUID, &params, &height);
  return std::make_pair(width, height);
}

bool NVencES::is_same(GUID g1, GUID g2){
  return (g1.Data1 == g2.Data1 && g1.Data2 == g2.Data2 && g1.Data3 == g2.Data3
          && g1.Data4[0] == g2.Data4[0] && g1.Data4[1] == g2.Data4[1]
          && g1.Data4[2] == g2.Data4[2] && g1.Data4[3] == g2.Data4[3]
          && g1.Data4[4] == g2.Data4[4] && g1.Data4[5] == g2.Data4[5]
          && g1.Data4[6] == g2.Data4[6] && g1.Data4[7] == g2.Data4[7]);
}

bool NVencES::initialize_encoder(GUID encodeGuid, GUID presetGuid,
                                 uint32_t width, uint32_t height,
                                 uint32_t frameRateNum, uint32_t framerateDen,
                                 NV_ENC_BUFFER_FORMAT format){
  memset(&init_params_, 0, sizeof(init_params_));
  init_params_.version = NV_ENC_INITIALIZE_PARAMS_VER;
  init_params_.encodeGUID = encodeGuid;
  init_params_.presetGUID = presetGuid;
  init_params_.encodeWidth = width;
  init_params_.encodeHeight = height;
  init_params_.frameRateNum = frameRateNum;
  init_params_.frameRateDen = framerateDen;
  init_params_.enablePTD = 1; // enable the Picture Type Decision is be
                              // taken by the NvEncodeAPI interface.

  NV_ENC_PRESET_CONFIG preset_config;
  memset(&preset_config, 0, sizeof(preset_config));
  preset_config.version = NV_ENC_PRESET_CONFIG_VER;
  preset_config.presetCfg.version = NV_ENC_CONFIG_VER;
  if (NV_ENC_SUCCESS != NVencAPI::api.nvEncGetEncodePresetConfig(
          encoder_,
          encodeGuid,
          presetGuid,
          &preset_config)){
    g_warning("nvenc cannot get encode preset config");
  } else {
    preset_config.presetCfg.version = NV_ENC_CONFIG_VER;
    preset_config.presetCfg.profileGUID = NV_ENC_CODEC_PROFILE_AUTOSELECT_GUID;
    preset_config.presetCfg.encodeCodecConfig.h264Config.level = NV_ENC_LEVEL_AUTOSELECT; 
    preset_config.presetCfg.encodeCodecConfig.h264Config.chromaFormatIDC = 1;

    // preset_config.presetCfg.encodeCodecConfig.h264Config.outputBufferingPeriodSEI = 1;
    // preset_config.presetCfg.encodeCodecConfig.h264Config.outputPictureTimingSEI = 1;
    // preset_config.presetCfg.encodeCodecConfig.h264Config.outputAUD = 1;
    // preset_config.presetCfg.encodeCodecConfig.h264Config.outputFramePackingSEI = 1;
    // preset_config.presetCfg.encodeCodecConfig.h264Config.outputRecoveryPointSEI = 1;
    preset_config.presetCfg.encodeCodecConfig.h264Config.repeatSPSPPS = 1;
      // if (GST_VIDEO_INFO_FORMAT (info) == GST_VIDEO_FORMAT_Y444) {
    // GST_DEBUG_OBJECT (h264enc, "have Y444 input, setting config accordingly");
    // preset_config.presetCfg.encodeCodecConfig.
    //     h264Config.separateColourPlaneFlag = 1;
    //preset_config.presetCfg.encodeCodecConfig.h264Config.chromaFormatIDC = 3;
    //preset_config.presetCfg.encodeCodecConfig.h264Config.outputAUD = 1;
    //}
     // if (GST_VIDEO_INFO_IS_INTERLACED (info)) {
     //   if (GST_VIDEO_INFO_INTERLACE_MODE (info) ==
     //       GST_VIDEO_INTERLACE_MODE_INTERLEAVED
     //       || GST_VIDEO_INFO_INTERLACE_MODE (info) ==
     //       GST_VIDEO_INTERLACE_MODE_MIXED) {
         // preset_config.presetCfg.frameFieldMode =
         //     NV_ENC_PARAMS_FRAME_FIELD_MODE_FIELD;
     //   }
     // }
     init_params_.encodeConfig = &preset_config.presetCfg;
  }
  
  NVENCSTATUS status = NVencAPI::api.nvEncInitializeEncoder(encoder_, &init_params_);
  if (NV_ENC_SUCCESS != status) {
    g_warning("encode session initialization failed");
    return false;
  }
  
  buffers_ = std2::make_unique<NVencBuffers>(encoder_, width, height, format);
  return true;
}

bool NVencES::copy_to_next_input_buffer(void *data, size_t size){
  if (!buffers_)
    return false;
  return buffers_->copy_to_next_input_buffer(data, size);
}
bool NVencES::encode_current_input(){
  if (!buffers_)
    return false;
  return buffers_->encode_current_input();
}

bool NVencES::process_encoded_frame(std::function<void(void *, uint32_t)> fun){
  if (!buffers_)
    return false;
  return buffers_->process_encoded_frame(fun);

}

}  // namespace switcher
