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

#include "./nvenc-encode-session.hpp"
#include <glib.h>   // log
#include <cstring>  // memset
#include "switcher/scope-exit.hpp"

namespace switcher {
NVencES::NVencES(uint32_t device_id,
                 //, uint32_t width, uint32_t height,  NV_ENC_BUFFER_FORMAT format
                 BaseLogger* log)
    : Logged(log), device_id_(device_id), cu_ctx_(device_id, log) {
  On_scope_exit {
    if (!safe_bool_idiom()) warning("NV encoder session initialization failed");
  };
  if (!cu_ctx_) return;
  params_.version = NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER;
  params_.apiVersion = NVENCAPI_VERSION;
  params_.device = cu_ctx_.cuda_ctx_;
  params_.deviceType = NV_ENC_DEVICE_TYPE_CUDA;
  if (NV_ENC_SUCCESS != NVencAPI::api.nvEncOpenEncodeSessionEx(&params_, &encoder_)) {
    // from nvidia doc: If the creation of encoder session fails, the client must call
    // ::NvEncDestroyEncoder API before exiting
    NVencAPI::api.nvEncDestroyEncoder(encoder_);
    encoder_ = nullptr;
    return;
  }
}

NVencES::~NVencES() {
  buffers_.reset();
  if (safe_bool_idiom() && !(NV_ENC_SUCCESS == NVencAPI::api.nvEncDestroyEncoder(encoder_)))
    warning("BUG! (destroying NV encoder session)");
}

NVencES::named_guid_t NVencES::get_supported_codecs() {
  named_guid_t res;
  // supported codecs
  if (NV_ENC_SUCCESS !=
      NVencAPI::api.nvEncGetEncodeGUIDs(encoder_, codecs_guids_, kArraySize, &codecs_num_)) {
    warning("issue with nvEncGetEncodeGUIDs");
    return res;
  }
  for (uint32_t i = 0; i < codecs_num_; ++i) {
    if (is_same(codecs_guids_[i], NV_ENC_CODEC_H264_GUID)) {
      res.push_back(std::make_pair(std::string("H264"), NV_ENC_CODEC_H264_GUID));
    } else if (is_same(codecs_guids_[i], NV_ENC_CODEC_HEVC_GUID)) {
      res.push_back(std::make_pair(std::string("HEVC"), NV_ENC_CODEC_HEVC_GUID));
    }
  }
  return res;
}

NVencES::named_guid_t NVencES::get_presets(GUID encodeGUID) {
  named_guid_t res;
  preset_GUID_ = encodeGUID;
  // supported codecs
  if (NV_ENC_SUCCESS !=
      NVencAPI::api.nvEncGetEncodePresetGUIDs(
          encoder_, preset_GUID_, presets_guids_, kArraySize, &presets_num_)) {
    warning("issue with nvEncGetEncodePresetGUIDs");
    return res;
  }
  for (uint32_t i = 0; i < presets_num_; ++i) {
    if (is_same(presets_guids_[i], NV_ENC_PRESET_DEFAULT_GUID)) {
      res.push_back(std::make_pair(std::string("Default"), NV_ENC_PRESET_DEFAULT_GUID));
    } else if (is_same(presets_guids_[i], NV_ENC_PRESET_HP_GUID)) {
      res.push_back(std::make_pair(std::string("HP"), NV_ENC_PRESET_HP_GUID));
    } else if (is_same(presets_guids_[i], NV_ENC_PRESET_HQ_GUID)) {
      res.push_back(std::make_pair(std::string("HQ"), NV_ENC_PRESET_HQ_GUID));
    } else if (is_same(presets_guids_[i], NV_ENC_PRESET_BD_GUID)) {
      res.push_back(std::make_pair(std::string("BD"), NV_ENC_PRESET_BD_GUID));
    } else if (is_same(presets_guids_[i], NV_ENC_PRESET_LOW_LATENCY_DEFAULT_GUID)) {
      res.push_back(std::make_pair(std::string("Low Latency default"),
                                   NV_ENC_PRESET_LOW_LATENCY_DEFAULT_GUID));
    } else if (is_same(presets_guids_[i], NV_ENC_PRESET_LOW_LATENCY_HQ_GUID)) {
      res.push_back(
          std::make_pair(std::string("Low Latency HQ"), NV_ENC_PRESET_LOW_LATENCY_HQ_GUID));
    } else if (is_same(presets_guids_[i], NV_ENC_PRESET_LOW_LATENCY_HP_GUID)) {
      res.push_back(
          std::make_pair(std::string("Low Latency HP"), NV_ENC_PRESET_LOW_LATENCY_HP_GUID));
    } else if (is_same(presets_guids_[i], NV_ENC_PRESET_LOSSLESS_DEFAULT_GUID)) {
      res.push_back(
          std::make_pair(std::string("Lossless default"), NV_ENC_PRESET_LOSSLESS_DEFAULT_GUID));
    } else if (is_same(presets_guids_[i], NV_ENC_PRESET_LOSSLESS_HP_GUID)) {
      res.push_back(std::make_pair(std::string("Lossless HP"), NV_ENC_PRESET_LOSSLESS_HP_GUID));
    } else
      debug("unknown preset GUID from nvenc");
  }
  return res;
}

NVencES::named_guid_t NVencES::get_profiles(GUID encodeGUID) {
  named_guid_t res;
  profile_GUID_ = encodeGUID;
  // supported codecs
  NVencAPI::api.nvEncGetEncodeProfileGUIDs(
      encoder_, profile_GUID_, profiles_guids_, kArraySize, &profiles_num_);
  for (uint32_t i = 0; i < profiles_num_; ++i) {
    if (is_same(profiles_guids_[i], NV_ENC_CODEC_PROFILE_AUTOSELECT_GUID)) {
      res.push_back(
          std::make_pair(std::string("Autoselect"), NV_ENC_CODEC_PROFILE_AUTOSELECT_GUID));
    } else if (is_same(profiles_guids_[i], NV_ENC_H264_PROFILE_BASELINE_GUID)) {
      res.push_back(std::make_pair(std::string("Baseline"), NV_ENC_H264_PROFILE_BASELINE_GUID));
    } else if (is_same(profiles_guids_[i], NV_ENC_H264_PROFILE_MAIN_GUID)) {
      res.push_back(std::make_pair(std::string("Main (H264)"), NV_ENC_H264_PROFILE_MAIN_GUID));
    } else if (is_same(profiles_guids_[i], NV_ENC_H264_PROFILE_HIGH_GUID)) {
      res.push_back(std::make_pair(std::string("High"), NV_ENC_H264_PROFILE_HIGH_GUID));
    } else if (is_same(profiles_guids_[i], NV_ENC_H264_PROFILE_HIGH_444_GUID)) {
      res.push_back(std::make_pair(std::string("High 444"), NV_ENC_H264_PROFILE_HIGH_444_GUID));
    } else if (is_same(profiles_guids_[i], NV_ENC_H264_PROFILE_STEREO_GUID)) {
      res.push_back(std::make_pair(std::string("Stereo"), NV_ENC_H264_PROFILE_STEREO_GUID));
    } else if (is_same(profiles_guids_[i], NV_ENC_H264_PROFILE_SVC_TEMPORAL_SCALABILTY)) {
      res.push_back(std::make_pair(std::string("SVC temporal scalability"),
                                   NV_ENC_H264_PROFILE_SVC_TEMPORAL_SCALABILTY));
    } else if (is_same(profiles_guids_[i], NV_ENC_H264_PROFILE_CONSTRAINED_HIGH_GUID)) {
      res.push_back(std::make_pair(std::string("Constrained high"),
                                   NV_ENC_H264_PROFILE_CONSTRAINED_HIGH_GUID));
    } else if (is_same(profiles_guids_[i], NV_ENC_HEVC_PROFILE_MAIN_GUID)) {
      res.push_back(std::make_pair(std::string("Main (HEVC)"), NV_ENC_HEVC_PROFILE_MAIN_GUID));
    } else
      debug("unknown profile GUID from nvenc");
  }
  return res;
}

std::vector<std::pair<std::string, NV_ENC_BUFFER_FORMAT>> NVencES::get_input_formats(
    GUID encodeGUID) {
  std::vector<std::pair<std::string, NV_ENC_BUFFER_FORMAT>> res;
  input_formats_GUID_ = encodeGUID;
  // supported codecs
  NVencAPI::api.nvEncGetInputFormats(
      encoder_, input_formats_GUID_, buf_formats_, kArraySize, &input_formats_num_);
  for (uint32_t i = 0; i < input_formats_num_; ++i) {
    switch (buf_formats_[i]) {
      case NV_ENC_BUFFER_FORMAT_UNDEFINED:
        warning("nvEncGetInputFormats gives NV_ENC_BUFFER_FORMAT_UNDEFINED (?)");
        break;
      case NV_ENC_BUFFER_FORMAT_NV12:
        res.push_back(std::make_pair(std::string("NV12"), NV_ENC_BUFFER_FORMAT_NV12));
        break;
      case NV_ENC_BUFFER_FORMAT_YV12:
        res.push_back(std::make_pair(std::string("YV12"), NV_ENC_BUFFER_FORMAT_YV12));
        break;
      case NV_ENC_BUFFER_FORMAT_IYUV:
        res.push_back(std::make_pair(std::string("IYUV"), NV_ENC_BUFFER_FORMAT_IYUV));
        break;
      case NV_ENC_BUFFER_FORMAT_YUV444:
        res.push_back(std::make_pair(std::string("YUV444"), NV_ENC_BUFFER_FORMAT_YUV444));
        break;
      case NV_ENC_BUFFER_FORMAT_ARGB:
        res.push_back(std::make_pair(std::string("ARGB"), NV_ENC_BUFFER_FORMAT_ARGB));
        break;
      case NV_ENC_BUFFER_FORMAT_AYUV:
        res.push_back(std::make_pair(std::string("AYUV"), NV_ENC_BUFFER_FORMAT_AYUV));
        break;
      default:
        debug("unknown input format from nvenc");
    }
  }
  return res;
}

std::pair<int, int> NVencES::get_max_width_height(GUID encodeGUID) {
  max_width_height_GUID_ = encodeGUID;
  NVencAPI::api.nvEncGetEncodeCaps(encoder_, max_width_height_GUID_, &caps_params_, &max_width_);
  caps_params_.capsToQuery = NV_ENC_CAPS_HEIGHT_MAX;
  NVencAPI::api.nvEncGetEncodeCaps(encoder_, max_width_height_GUID_, &caps_params_, &max_height_);
  return std::make_pair(max_width_, max_height_);
}

bool NVencES::is_same(const GUID& g1, const GUID& g2) {
  return (g1.Data1 == g2.Data1 && g1.Data2 == g2.Data2 && g1.Data3 == g2.Data3 &&
          g1.Data4[0] == g2.Data4[0] && g1.Data4[1] == g2.Data4[1] && g1.Data4[2] == g2.Data4[2] &&
          g1.Data4[3] == g2.Data4[3] && g1.Data4[4] == g2.Data4[4] && g1.Data4[5] == g2.Data4[5] &&
          g1.Data4[6] == g2.Data4[6] && g1.Data4[7] == g2.Data4[7]);
}

bool NVencES::initialize_encoder(GUID encodeGuid,
                                 GUID presetGuid,
                                 GUID profileGuid,
                                 uint32_t bitrate,
                                 uint32_t width,
                                 uint32_t height,
                                 uint32_t frameRateNum,
                                 uint32_t framerateDen,
                                 NV_ENC_BUFFER_FORMAT format) {
  memset(&init_params_, 0, sizeof(init_params_));
  init_params_.version = NV_ENC_INITIALIZE_PARAMS_VER;
  init_params_.encodeGUID = encodeGuid;
  init_params_.presetGUID = presetGuid;
  init_params_.encodeWidth = width;
  init_params_.encodeHeight = height;
  init_params_.frameRateNum = frameRateNum;
  init_params_.frameRateDen = framerateDen;
  init_params_.enablePTD = 1;  // enable the Picture Type Decision is be
                               // taken by the NvEncodeAPI interface.

  memset(&preset_config_, 0, sizeof(preset_config_));
  preset_config_.version = NV_ENC_PRESET_CONFIG_VER;
  preset_config_.presetCfg.version = NV_ENC_CONFIG_VER;
  encode_GUID_ = encodeGuid;
  preset_GUID_ = presetGuid;
  if (NV_ENC_SUCCESS !=
      NVencAPI::api.nvEncGetEncodePresetConfig(
          encoder_, encode_GUID_, preset_GUID_, &preset_config_)) {
    warning("nvenc cannot get encode preset config");
  } else {
    preset_config_.presetCfg.version = NV_ENC_CONFIG_VER;
    preset_config_.presetCfg.profileGUID = profileGuid;
    preset_config_.presetCfg.gopLength = 10;  // 3 I frames per second @ 30FPS
    if (bitrate) preset_config_.presetCfg.rcParams.averageBitRate = bitrate;

    if (is_same(encodeGuid, NV_ENC_CODEC_H264_GUID)) {
      preset_config_.presetCfg.encodeCodecConfig.h264Config.level = NV_ENC_LEVEL_AUTOSELECT;
      if (format == NV_ENC_BUFFER_FORMAT_YUV444)
        preset_config_.presetCfg.encodeCodecConfig.h264Config.chromaFormatIDC = 3;
      else
        preset_config_.presetCfg.encodeCodecConfig.h264Config.chromaFormatIDC = 1;
      preset_config_.presetCfg.encodeCodecConfig.h264Config.repeatSPSPPS = 1;
    } else if (is_same(encodeGuid, NV_ENC_CODEC_HEVC_GUID)) {
      preset_config_.presetCfg.encodeCodecConfig.hevcConfig.level = NV_ENC_LEVEL_AUTOSELECT;
      if (format == NV_ENC_BUFFER_FORMAT_YUV444)
        preset_config_.presetCfg.encodeCodecConfig.hevcConfig.chromaFormatIDC = 3;
      else
        preset_config_.presetCfg.encodeCodecConfig.hevcConfig.chromaFormatIDC = 1;
      preset_config_.presetCfg.encodeCodecConfig.hevcConfig.repeatSPSPPS = 1;
    } else {
      warning("Unknown codec for hardware encoding.");
      return false;
    }

    init_params_.encodeConfig = &preset_config_.presetCfg;
  }

  NVENCSTATUS status = NVencAPI::api.nvEncInitializeEncoder(encoder_, &init_params_);
  if (NV_ENC_SUCCESS != status) {
    warning("encode session initialization failed");
    return false;
  }
  buffers_ = std::make_unique<NVencBuffers>(encoder_, width, height, format);
  return true;
}

bool NVencES::copy_to_next_input_buffer(void* data, size_t size) {
  if (!buffers_) return false;
  return buffers_->copy_to_next_input_buffer(data, size);
}
bool NVencES::encode_current_input() {
  if (!buffers_) return false;
  return buffers_->encode_current_input();
}

bool NVencES::process_encoded_frame(std::function<void(void*, uint32_t)> fun) {
  if (!buffers_) return false;
  return buffers_->process_encoded_frame(fun);
}

}  // namespace switcher
