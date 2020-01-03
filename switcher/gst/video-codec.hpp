/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_GST_VIDEO_CODEC_H__
#define __SWITCHER_GST_VIDEO_CODEC_H__

#include <unordered_set>
#include <vector>
#include "../shmdata/gst-shm-tree-updater.hpp"
#include "./pipeliner.hpp"
#include "./unique-gst-element.hpp"

namespace switcher {
namespace gst {

class VideoCodec {
 public:
  VideoCodec(quiddity::Quiddity* quid,
             const std::string& shmpath_to_encode,
             const std::string& shmpath_encoded = {});
  VideoCodec() = delete;
  ~VideoCodec() = default;
  VideoCodec(const VideoCodec&) = delete;
  VideoCodec& operator=(const VideoCodec&) = delete;

  void set_shm(const std::string& shmpath);
  void set_none();
  bool start();
  bool stop();

 private:
  quiddity::Quiddity* quid_;
  quiddity::method::meth_id_t reset_id_;
  // shmdata path
  std::string shmpath_to_encode_;
  std::string shm_encoded_path_;
  bool custom_shmsink_path_;
  // gst pipeline
  std::unique_ptr<Pipeliner> gst_pipeline_;
  // video encoding
  UGstElem shmsrc_{"shmdatasrc"};
  UGstElem queue_codec_element_{"queue"};
  UGstElem color_space_codec_element_{"videoconvert"};
  UGstElem codec_element_{"x264enc"};
  UGstElem shm_encoded_{"shmdatasink"};
  std::unique_ptr<GstShmTreeUpdater> shmsrc_sub_{nullptr};
  std::unique_ptr<GstShmTreeUpdater> shmsink_sub_{nullptr};
  std::vector<std::string> codec_properties_{};
  quiddity::property::Selection<> codecs_;
  quiddity::property::prop_id_t codec_id_;
  // codec params black list
  std::unordered_set<std::string> param_black_list_{"analyse",  // x264enc
                                                    "arnr-type",
                                                    "timebase",
                                                    "error-resilient",
                                                    "frame-packing",  // x264enc
                                                    "multipass-cache-file",
                                                    "multipass-mode",
                                                    "name",
                                                    "parent",
                                                    "snapshot",
                                                    "temporal-scalability-layer-id",
                                                    "temporal-scalability-periodicity",
                                                    "temporal-scalability-rate-decimator",
                                                    "temporal-scalability-target-bitrate",
                                                    "tune",  // x264enc
                                                    "twopass-vbr-bias",
                                                    "twopass-vbr-maxsection",
                                                    "twopass-vbr-minsection"};
  // shmdatasrc copy-buffers property:
  bool copy_buffers_{false};
  // parameter grouping
  quiddity::property::prop_id_t param_group_id_;

  bool remake_codec_elements();
  void make_codec_properties();
  void uninstall_codec_properties();
  void make_bin();
  void show();
  void hide();
  quiddity::property::prop_id_t install_codec();
  bool reset_codec_configuration();
  static void set_codec(const gint value, void* user_data);
  static gint get_codec(void* user_data);
  // static gboolean get_codec_long_list(void *user_data);
  // static void set_codec_long_list(gboolean mute, void *user_data);
  static gboolean sink_factory_filter(GstPluginFeature* feature, gpointer data);
  static gint sink_compare_ranks(GstPluginFeature* f1, GstPluginFeature* f2);
};

}  // namespace gst
}  // namespace switcher
#endif
