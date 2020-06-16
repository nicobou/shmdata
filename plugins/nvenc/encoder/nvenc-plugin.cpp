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

#include "./nvenc-plugin.hpp"
#include <gst/gst.h>
#include "cuda/cuda-context.hpp"
#include "switcher/utils/scope-exit.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(NVencPlugin,
                                     "nvenc",
                                     "Hardware video encoder (NVenc)",
                                     "video",
                                     "writer/reader",
                                     "CUDA-based video encoder",
                                     "LGPL",
                                     "Nicolas Bouillot");

NVencPlugin::NVencPlugin(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)),
      default_preset_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "bitrate_from_preset",
          [this](bool value) {
            bitrate_from_preset_ = value;
            if (bitrate_from_preset_) {
              pmanage<MPtr(&property::PBag::disable)>(
                  bitrate_id_, "Cannot set bitrate if using default preset configuration.");
            } else {
              pmanage<MPtr(&property::PBag::enable)>(bitrate_id_);
            }
            return true;
          },
          [this]() { return bitrate_from_preset_; },
          "Default preset configuration",
          "Use default preset configuration.",
          bitrate_from_preset_)),
      bitrate_id_(
          pmanage<MPtr(&property::PBag::make_unsigned_int)>("bitrate",
                                                            [this](const uint32_t& value) {
                                                              bitrate_ = value;
                                                              return true;
                                                            },
                                                            [this]() { return bitrate_; },
                                                            "Desired bitrate",
                                                            "Value of the desired average bitrate.",
                                                            bitrate_,
                                                            1000000,
                                                            20000000)),
      es_(std::make_unique<ThreadedWrapper<NVencES>>(get_log_ptr())),
      shmcntr_(static_cast<Quiddity*>(this)) {
  register_writer_suffix("video-encoded");
  auto devices = CudaContext::get_devices();
  std::vector<std::string> names;
  for (auto& it : devices) {
    devices_nv_ids_.push_back(it.first);
    names.push_back(std::string("GPU #") + std::to_string(it.first) + " " + it.second);
  }
  if (names.empty()) {
    message("ERROR:Could not find any NVENC-enabled GPU.");
    is_valid_ = false;
    return;
  }
  devices_ = property::Selection<>(std::move(names), 0);
  update_device();
  devices_id_ = pmanage<MPtr(&property::PBag::make_selection<>)>(
      "gpu",
      [this](const quiddity::property::IndexOrName& val) {
        if (devices_.get_current_index() == val.index_) return true;
        devices_.select(val);
        update_device();
        return true;
      },
      [this]() { return devices_.get(); },
      "encoder GPU",
      "Selection of the GPU used for encoding",
      devices_);
  pmanage<MPtr(&property::PBag::set_to_current)>(default_preset_id_);

  if (!es_) {
    is_valid_ = false;
    return;
  }
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);
  is_valid_ = es_.get()->invoke<bool>([](NVencES* ctx) { return ctx->safe_bool_idiom(); });
}

void NVencPlugin::update_device() {
  es_.reset();
  es_ = std::make_unique<ThreadedWrapper<NVencES>>(devices_nv_ids_[devices_.get_current_index()],
                                                   get_log_ptr());
  if (!es_.get()->invoke<bool>([](NVencES* ctx) { return ctx->safe_bool_idiom(); })) {
    message(
        "ERROR: nvenc failed to create encoding session "
        "(the total number of simultaneous sessions "
        "may be reached)");
    warning(
        "nvenc failed to create encoding session "
        "(the total number of simultaneous sessions "
        "may be reached)");
    es_.reset();  // this makes init method failing
    return;
  }
  update_codec();
}

void NVencPlugin::update_codec() {
  codecs_guids_ = es_->invoke<decltype(codecs_guids_)>(
      [](NVencES* ctx) { return ctx->get_supported_codecs(); });
  std::vector<std::string> names;
  for (auto& it : codecs_guids_) names.push_back(it.first);
  codecs_ = property::Selection<>(std::move(names), 0);
  auto set = [this](const quiddity::property::IndexOrName& val) {
    if (codecs_.get_current_index() != val.index_) {
      codecs_.select(val);
      update_preset();
      update_profile();
      update_max_width_height();
      update_input_formats();
    }
    return true;
  };
  auto get = [this]() { return codecs_.get_current_index(); };
  if (0 == codecs_id_)
    codecs_id_ = pmanage<MPtr(&property::PBag::make_selection<>)>(
        "codec", set, get, "Codec", "Codec property::Selection<>", codecs_);
  else
    pmanage<MPtr(&property::PBag::replace)>(
        codecs_id_,
        std::make_unique<property::Property<property::Selection<>, property::Selection<>::index_t>>(
            set, get, "Codec", "Codec property::Selection<>", codecs_, codecs_.size() - 1));
  update_preset();
  update_profile();
  update_max_width_height();
  update_input_formats();
}

void NVencPlugin::update_preset() {
  auto cur_codec = codecs_.get_current();
  auto guid_iter = std::find_if(
      codecs_guids_.begin(), codecs_guids_.end(), [&](const std::pair<std::string, GUID>& codec) {
        return codec.first == cur_codec;
      });
  presets_guids_ = es_->invoke<decltype(presets_guids_)>(
      [&](NVencES* ctx) { return ctx->get_presets(guid_iter->second); });
  std::vector<std::string> names;
  size_t index_low_lantency_default = 0;
  size_t current_index = 0;
  for (auto& it : presets_guids_) {
    names.push_back(it.first);
    // FIXME (the following does not work with SIP and sometimes with decoder)
    // if (it.first == "Low Latency default") index_low_lantency_default = current_index;
    ++current_index;
  }
  presets_ = property::Selection<>(std::move(names), index_low_lantency_default);
  auto set = [this](const quiddity::property::IndexOrName& val) {
    if (presets_.get_current_index() != val.index_) presets_.select(val);
    return true;
  };
  auto get = [this]() { return presets_.get(); };
  if (0 == presets_id_)
    presets_id_ = pmanage<MPtr(&property::PBag::make_selection<>)>(
        "preset", set, get, "Preset", "Preset property::Selection<>", presets_);
  else
    pmanage<MPtr(&property::PBag::replace)>(
        presets_id_,
        std::make_unique<
            property::Property<property::Selection<>, quiddity::property::IndexOrName>>(
            set, get, "Preset", "Preset property::Selection<>", presets_, presets_.size() - 1));
}

void NVencPlugin::update_profile() {
  auto cur_codec = codecs_.get_current();
  auto guid_iter = std::find_if(
      codecs_guids_.begin(), codecs_guids_.end(), [&](const std::pair<std::string, GUID>& codec) {
        return codec.first == cur_codec;
      });
  profiles_guids_ = es_->invoke<decltype(profiles_guids_)>(
      [&](NVencES* ctx) { return ctx->get_profiles(guid_iter->second); });
  std::vector<std::string> names;
  for (auto& it : profiles_guids_) names.push_back(it.first);
  profiles_ = property::Selection<>(std::move(names), 0);
  auto set = [this](const quiddity::property::IndexOrName& val) {
    if (profiles_.get_current_index() != val.index_) profiles_.select(val);
    return true;
  };
  auto get = [this]() { return profiles_.get(); };
  if (0 == profiles_id_)
    profiles_id_ = pmanage<MPtr(&property::PBag::make_selection<>)>(
        "profile", set, get, "Profile", "Profile property::Selection<>", profiles_);
  else
    pmanage<MPtr(&property::PBag::replace)>(
        profiles_id_,
        std::make_unique<
            property::Property<property::Selection<>, quiddity::property::IndexOrName>>(
            set, get, "Profile", "Profile property::Selection<>", profiles_, profiles_.size() - 1));
}

void NVencPlugin::update_max_width_height() {
  auto cur_codec = codecs_.get_current();
  auto guid_iter = std::find_if(
      codecs_guids_.begin(), codecs_guids_.end(), [&](const std::pair<std::string, GUID>& codec) {
        return codec.first == cur_codec;
      });
  auto mwh = es_->invoke<std::pair<int, int>>(
      [&](NVencES* ctx) { return ctx->get_max_width_height(guid_iter->second); });
  max_width_ = mwh.first;
  max_height_ = mwh.second;
  auto getwidth = [this]() { return this->max_width_; };
  if (0 == max_width_id_)
    max_width_id_ = pmanage<MPtr(&property::PBag::make_int)>("maxwidth",
                                                             nullptr,
                                                             getwidth,
                                                             "Max width",
                                                             "Max video source width",
                                                             max_width_,
                                                             max_width_,
                                                             max_width_);
  else
    pmanage<MPtr(&property::PBag::notify)>(max_width_id_);

  auto getheight = [this]() { return max_height_; };
  if (0 == max_height_id_)
    max_height_id_ = pmanage<MPtr(&property::PBag::make_int)>("maxheight",
                                                              nullptr,
                                                              getheight,
                                                              "Max height",
                                                              "Max video source height",
                                                              max_height_,
                                                              max_height_,
                                                              max_height_);
  else
    pmanage<MPtr(&property::PBag::notify)>(max_height_id_);
}

void NVencPlugin::update_input_formats() {
  auto cur_codec = codecs_.get_current();
  auto guid_iter = std::find_if(
      codecs_guids_.begin(), codecs_guids_.end(), [&](const std::pair<std::string, GUID>& codec) {
        return codec.first == cur_codec;
      });
  video_formats_.clear();
  video_formats_ = es_->invoke<decltype(video_formats_)>(
      [&](NVencES* ctx) { return ctx->get_input_formats(guid_iter->second); });
  for (auto& it : video_formats_) {
    std::string format;
    if ("NV12" == it.first)
      format = "NV12";
    else if ("YV12" == it.first)
      format = "YV12";
    else if ("IYUV" == it.first)
      format = "I420";
    else if ("YUV444" == it.first)
      format = "Y444";
    else if ("ARGB" == it.first)
      format = "ARGB";
    else if ("AYUV" == it.first)
      format = "AYUV";
    else
      warning("format not supported by NVencPlugin :%", it.first);

    if (!format.empty()) it.first = std::string("video/x-raw, format=(string)") + format;
  }
}

bool NVencPlugin::on_shmdata_disconnect() {
  shm_.reset(nullptr);
  es_.reset();
  es_ = std::make_unique<ThreadedWrapper<NVencES>>(devices_nv_ids_[devices_.get_current_index()],
                                                   get_log_ptr());
  shmw_.reset(nullptr);

  pmanage<MPtr(&property::PBag::enable)>(devices_id_);
  pmanage<MPtr(&property::PBag::enable)>(presets_id_);
  pmanage<MPtr(&property::PBag::enable)>(profiles_id_);
  pmanage<MPtr(&property::PBag::enable)>(codecs_id_);
  pmanage<MPtr(&property::PBag::enable)>(max_width_id_);
  pmanage<MPtr(&property::PBag::enable)>(max_height_id_);
  pmanage<MPtr(&property::PBag::enable)>(default_preset_id_);
  pmanage<MPtr(&property::PBag::set_to_current)>(default_preset_id_);

  return true;
}

bool NVencPlugin::on_shmdata_connect(const std::string& shmpath) {
  // Needed to avoid concurrency with old shmdata follower.
  shm_.reset(nullptr);
  shm_.reset(new shmdata::Follower(
      this,
      shmpath,
      [this](void* data, size_t size) { this->on_shmreader_data(data, size); },
      [this](const std::string& data_descr) { this->on_shmreader_server_connected(data_descr); }));

  pmanage<MPtr(&property::PBag::disable)>(devices_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(presets_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(profiles_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(codecs_id_, shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(max_width_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(max_height_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(default_preset_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);
  pmanage<MPtr(&property::PBag::disable)>(bitrate_id_,
                                          shmdata::Connector::disabledWhenConnectedMsg);

  return true;
}

bool NVencPlugin::can_sink_caps(const std::string& strcaps) {
  GstCaps* caps = gst_caps_from_string(strcaps.c_str());
  On_scope_exit {
    if (nullptr != caps) gst_caps_unref(caps);
  };
  return video_formats_.end() !=
         std::find_if(video_formats_.begin(),
                      video_formats_.end(),
                      [&](const std::pair<std::string, NV_ENC_BUFFER_FORMAT>& caps_iter) {
                        GstCaps* curcaps = gst_caps_from_string(caps_iter.first.c_str());
                        On_scope_exit {
                          if (nullptr != curcaps) gst_caps_unref(curcaps);
                        };
                        return gst_caps_can_intersect(curcaps, caps);
                      });
}

void NVencPlugin::on_shmreader_data(void* data, size_t size) {
  if (!es_.get()->invoke<bool>(
          [&](NVencES* ctx) { return ctx->copy_to_next_input_buffer(data, size); })) {
    warning("error copying data to nvenc");
    return;
  }
  es_.get()->invoke_async([](NVencES* ctx) { ctx->encode_current_input(); });
  es_.get()->invoke_async([&](NVencES* ctx) {
    ctx->process_encoded_frame([&](void* data, uint32_t enc_size) {
      shmw_->writer<MPtr(&::shmdata::Writer::copy_to_shm)>(data, enc_size);
      shmw_->bytes_written(enc_size);
    });
  });
}

void NVencPlugin::on_shmreader_server_connected(const std::string& data_descr) {
  GstCaps* caps = gst_caps_from_string(data_descr.c_str());
  On_scope_exit {
    if (nullptr != caps) gst_caps_unref(caps);
  };
  GstStructure* s = gst_caps_get_structure(caps, 0);
  if (nullptr == s) {
    warning("cannot get structure from caps (nvenc)");
    return;
  }
  gint width = 0, height = 0;
  if (!gst_structure_get_int(s, "width", &width) || !gst_structure_get_int(s, "height", &height)) {
    warning("cannot get width/height from shmdata description (nvenc)");
    return;
  }
  gint frameNum = 0, frameDen = 0;
  if (!gst_structure_get_fraction(s, "framerate", &frameNum, &frameDen)) {
    warning("cannot get framerate from shmdata description (nvenc)");
    return;
  }
  const char* format = gst_structure_get_string(s, "format");
  if (nullptr == format) {
    warning("cannot get video format from shmdata description (nvenc)");
    return;
  }
  auto format_str = std::string(format);
  auto buf_format = NV_ENC_BUFFER_FORMAT_UNDEFINED;
  if (format_str == "NV12")
    buf_format = NV_ENC_BUFFER_FORMAT_NV12;
  else if (format_str == "YV12")
    buf_format = NV_ENC_BUFFER_FORMAT_YV12;
  else if (format_str == "I420")
    buf_format = NV_ENC_BUFFER_FORMAT_IYUV;
  else if (format_str == "Y444")
    buf_format = NV_ENC_BUFFER_FORMAT_YUV444;
  else if (format_str == "ARGB")
    buf_format = NV_ENC_BUFFER_FORMAT_ARGB;
  else if (format_str == "AYUV")
    buf_format = NV_ENC_BUFFER_FORMAT_AYUV;
  else {
    warning("video format % not supported by switcher nvenc plugin", format_str);
    return;
  }

  auto cur_codec = codecs_.get_current();
  auto guid_iter = std::find_if(
      codecs_guids_.begin(), codecs_guids_.end(), [&](const std::pair<std::string, GUID>& codec) {
        return codec.first == cur_codec;
      });
  auto cur_preset = presets_.get_current();
  auto preset_iter = std::find_if(
      presets_guids_.begin(),
      presets_guids_.end(),
      [&](const std::pair<std::string, GUID>& preset) { return preset.first == cur_preset; });

  auto cur_profile = profiles_.get_current();
  auto profiles_iter = std::find_if(
      profiles_guids_.begin(),
      profiles_guids_.end(),
      [&](const std::pair<std::string, GUID>& profile) { return profile.first == cur_profile; });

  es_->invoke_async([&](NVencES* ctx) {
    ctx->initialize_encoder(guid_iter->second,
                            preset_iter->second,
                            profiles_iter->second,
                            bitrate_from_preset_ ? 0 : bitrate_,
                            width,
                            height,
                            frameNum,
                            frameDen,
                            buf_format);
  });
  shmw_.reset();

  std::string codec;
  if (cur_codec == "HEVC")
    codec = "x-h265";
  else
    codec = "x-h264";

  shmw_ = std::make_unique<shmdata::Writer>(
      this,
      make_shmpath("video-encoded"),
      1,
      std::string("video/" + codec +
                  ", stream-format=(string)byte-stream, "
                  "alignment=(string)au, profile=(string)baseline" +
                  ", width=(int)" + std::to_string(width) + ", height=(int)" +
                  std::to_string(height) +
                  ", pixel-aspect-ratio=(fraction)1/1, framerate=(fraction)" +
                  std::to_string(frameNum) + "/" + std::to_string(frameDen)));
}

}  // namespace quiddities
}  // namespace switcher
