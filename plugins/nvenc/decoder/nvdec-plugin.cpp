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

#include "./nvdec-plugin.hpp"
#include <nvcuvid.h>
#include <cstring>
#include "cuda/cuda-context.hpp"
#include "switcher/scope-exit.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(NVdecPlugin,
                                     "nvdec",
                                     "Hardware video decoder (NVdec)",
                                     "video",
                                     "writer/reader",
                                     "CUDA-based video decoder",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::array<const char*, 5> NVdecPlugin::kSupportedCodecs{
    {"video/x-h264", "video/x-h265", "video/mpeg", "video/x-jpeg", "image/jpeg"}};

NVdecPlugin::NVdecPlugin(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)), shmcntr_(static_cast<Quiddity*>(this)) {
  auto devices = CudaContext::get_devices();
  std::vector<std::string> names;
  for (auto& it : devices) {
    devices_nv_ids_.push_back(it.first);
    names.push_back(std::string("GPU #") + std::to_string(it.first) + " " + it.second);
  }
  if (names.empty()) {
    message("ERROR:Could not find any CUDA-enabled GPU.");
    is_valid_ = false;
    return;
  }
  devices_ = Selection<>(std::move(names), 0);
  devices_id_ =
      pmanage<MPtr(&PContainer::make_selection<>)>("gpu",
                                                   [this](const IndexOrName& val) {
                                                     if (devices_.get_current_index() == val.index_)
                                                       return true;
                                                     devices_.select(val);
                                                     return true;
                                                   },
                                                   [this]() { return devices_.get(); },
                                                   "decoder GPU",
                                                   "Selection of the GPU used for decoding",
                                                   devices_);
  if (devices_.empty()) {
    is_valid_ = false;
    return;
  }
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);
}

bool NVdecPlugin::can_sink_caps(const std::string& strcaps) {
  GstCaps* caps = gst_caps_from_string(strcaps.c_str());
  On_scope_exit {
    if (nullptr != caps) gst_caps_unref(caps);
  };

  for (auto& codec : kSupportedCodecs) {
    GstCaps* codec_caps = gst_caps_from_string(codec);
    On_scope_exit {
      if (nullptr != codec_caps) gst_caps_unref(codec_caps);
    };
    if (gst_caps_can_intersect(codec_caps, caps)) return true;
  }

  return false;
}

bool NVdecPlugin::on_shmdata_connect(const std::string& shmpath) {
  shmfollower_.reset(nullptr);
  shmfollower_ = std::make_unique<ShmdataFollower>(
      this,
      shmpath,
      [this](void* data, size_t size) { this->on_shmreader_data(data, size); },
      [this](const std::string& data_descr) { this->on_shmreader_server_connected(data_descr); },
      [this]() { this->on_shmreader_server_disconnected(); });

  pmanage<MPtr(&PContainer::disable)>(devices_id_, ShmdataConnector::disabledWhenConnectedMsg);
  return static_cast<bool>(shmfollower_.get());
}

bool NVdecPlugin::on_shmdata_disconnect() {
  shmfollower_.reset(nullptr);
  pmanage<MPtr(&PContainer::enable)>(devices_id_);
  return true;
}

void NVdecPlugin::on_shmreader_data(void* data, size_t size) {
  if (!ds_)
    ds_ = std::make_unique<ThreadedWrapper<NVencDS>>(
        devices_nv_ids_[devices_.get_current_index()], video_codec_, get_log_ptr());

  if (!ds_->invoke<MPtr(&NVencDS::safe_bool_idiom)>()) {
    warning("nvcuvid failed to create a decoding session (nvdec).");
    ds_.reset(nullptr);
    return;
  }

  ds_.get()->invoke<MPtr(&NVencDS::parse_data)>([&](CUvideoparser parser) {
    CUVIDSOURCEDATAPACKET packet;
    packet.payload_size = size;
    packet.payload = (const unsigned char*)data;
    cuvidParseVideoData(parser, &packet);
  });

  ds_.get()->invoke<MPtr(&NVencDS::process_decoded)>([&](const unsigned char* data_decoded,
                                                         unsigned int data_width,
                                                         unsigned int data_height,
                                                         unsigned int pitch,
                                                         bool& scaled) {
    if (!pitch) return;
    guint8* src = (guint8*)data_decoded;
    guint height = data_height * 3 / 2;  // Only NV12 support for now, 12bpp.

    {
      if (scaled) {
        size_t start_width = caps_.find("width=") + 6;
        size_t end_width = caps_.find_first_of(",", start_width);
        size_t start_height = caps_.find("height=") + 7;
        size_t end_height = caps_.find_first_of(",", start_height);
        caps_.replace(start_width, end_width - start_width, std::to_string(data_width));
        caps_.replace(start_height, end_height - start_height, std::to_string(data_height));
        writer_size_ = data_width * data_height * 3 / 2;

        shmwriter_.reset(nullptr);
        shmwriter_ =
            std::make_unique<ShmdataWriter>(this, make_file_name("video"), writer_size_, caps_);

        scaled = false;
      }

      std::unique_ptr<shmdata::OneWriteAccess> shm_ptr;

      if (writer_size_ > shmwriter_->writer<MPtr(&shmdata::Writer::alloc_size)>())
        shm_ptr =
            shmwriter_->writer<MPtr(&shmdata::Writer::get_one_write_access_resize)>(writer_size_);
      else
        shm_ptr = shmwriter_->writer<MPtr(&shmdata::Writer::get_one_write_access)>();

      shm_ptr->notify_clients(writer_size_);

      guint8* dest = (guint8*)shm_ptr->get_mem();
      for (guint y = 0; y < height; ++y) {
        memcpy(dest, src, data_width);
        dest += data_width;
        src += pitch;
      }
    }

    shmwriter_->bytes_written(writer_size_);
  });
}

void NVdecPlugin::on_shmreader_server_connected(const std::string& data_descr) {
  GstCaps* caps = gst_caps_from_string(data_descr.c_str());
  On_scope_exit {
    if (nullptr != caps) gst_caps_unref(caps);
  };

  GstStructure* s = gst_caps_get_structure(caps, 0);
  if (nullptr == s) {
    warning("Cannot get structure from caps (nvdec)");
    return;
  }

  gint width = 0, height = 0;
  if (!gst_structure_get_int(s, "width", &width) || !gst_structure_get_int(s, "height", &height)) {
    warning("Cannot get width/height from shmdata description (nvdec)");
    return;
  }

  gint frame_num = 0, frame_den = 0;
  if (!gst_structure_get_fraction(s, "framerate", &frame_num, &frame_den)) {
    warning("Cannot get framerate from shmdata description (nvdec)");
    return;
  }

  const std::string type = gst_structure_get_name(s);
  if (type.empty()) {
    warning("Cannot get codec from shmdata description (nvdec)");
    return;
  }

  std::string content_type = type.substr(0, 5);
  if (content_type != "video" && content_type != "image") {
    warning("Shmdata description is neither a video nor an image (nvdec).");
    return;
  } else {
    std::string codec = type.substr(6);
    gint mpeg_version = 0;
    gst_structure_get_int(s, "mpegversion", &mpeg_version);

    video_codec_ = convert_video_type_to_cuda(content_type, codec, mpeg_version);

    if (video_codec_ == cudaVideoCodec_NumCodecs) {
      warning("Could not find valid codec in shmdata description (nvdec)");
      return;
    }
  }

  caps_ = "video/x-raw, format=NV12, width=" + std::to_string(width) + ", height=" +
          std::to_string(height) + ", framerate=" + std::to_string(frame_num) + "/" +
          std::to_string(frame_den) + ", pixel-aspect-ratio=1/1";
  writer_size_ = width * height * 3 / 2;  // Size is known because only NV12 is supported for now.

  ds_.reset(nullptr);
  shmwriter_.reset(nullptr);
  shmwriter_ = std::make_unique<ShmdataWriter>(this, make_file_name("video"), writer_size_, caps_);
}

void NVdecPlugin::on_shmreader_server_disconnected() {
  ds_.reset(nullptr);
  shmwriter_.reset(nullptr);
}

cudaVideoCodec NVdecPlugin::convert_video_type_to_cuda(const std::string& content_type,
                                                       const std::string& codec,
                                                       guint mpeg_version) {
  cudaVideoCodec video_codec = cudaVideoCodec_NumCodecs;
  if (content_type == "video") {
    if (codec == "x-h264")
      video_codec = cudaVideoCodec_H264;
    else if (codec == "x-h265")
      video_codec = cudaVideoCodec_HEVC;
    else if (codec == "x-jpeg")
      video_codec = cudaVideoCodec_JPEG;
    else if (codec == "mpeg") {
      switch (mpeg_version) {
        case 1:
          video_codec = cudaVideoCodec_MPEG1;
          break;
        case 2:
          video_codec = cudaVideoCodec_MPEG2;
          break;
          break;
        case 4:
          video_codec = cudaVideoCodec_MPEG4;
          break;
        default:
#ifdef DEBUG
          std::cerr << "(nvdec) Unsupported mpeg version: " << std::to_string(mpeg_version) << '\n';
#endif
          break;
      }
    } else {
#ifdef DEBUG
      std::cerr << "(nvdec) Unsupported video version " << std::to_string(mpeg_version) << '\n';
#endif
    }
  } else if (content_type == "image" && codec == "jpeg") {
    video_codec = cudaVideoCodec_JPEG;
  }

  return video_codec;
}

}  // namespace switcher
