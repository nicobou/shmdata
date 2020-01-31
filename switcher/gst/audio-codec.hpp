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

#ifndef __SWITCHER_GST_AUDIO_CODEC_H__
#define __SWITCHER_GST_AUDIO_CODEC_H__

#include <unordered_set>
#include <vector>
#include "../shmdata/gst-tree-updater.hpp"
#include "./pipeliner.hpp"
#include "./unique-gst-element.hpp"

namespace switcher {
namespace gst {
class AudioCodec {
 public:
  AudioCodec(quiddity::Quiddity* quid);
  AudioCodec() = delete;
  ~AudioCodec() = default;
  AudioCodec(const AudioCodec&) = delete;
  AudioCodec& operator=(const AudioCodec&) = delete;

  bool start(const std::string& shmpath, const std::string& shmpath_encoded = {});
  bool stop();

 private:
  quiddity::Quiddity* quid_;
  quiddity::method::meth_id_t reset_id_;
  // shmdata path
  std::string shmpath_to_encode_{};
  std::string shm_encoded_path_{};
  // gst pipeline
  std::unique_ptr<Pipeliner> gst_pipeline_;
  // audio encoding
  UGstElem shmsrc_{"shmdatasrc"};
  UGstElem queue_codec_element_{"queue"};
  UGstElem audio_convert_{"audioconvert"};
  UGstElem audio_resample_{"audioresample"};
  UGstElem codec_element_{"opusenc"};
  UGstElem shm_encoded_{"shmdatasink"};
  std::unique_ptr<shmdata::GstTreeUpdater> shmsrc_sub_{nullptr};
  std::unique_ptr<shmdata::GstTreeUpdater> shmsink_sub_{nullptr};
  // codec props
  quiddity::property::Selection<> codecs_;
  quiddity::property::prop_id_t codec_id_;
  std::vector<std::string> codec_properties_{};
  // codec params black list
  std::unordered_set<std::string> param_black_list_{
      "name", "parent", "hard-resync", "mark-granule", "perfect-timestamp", "tolerance"};
  quiddity::property::prop_id_t group_codec_id_{0};
  // shmdatasrc copy-buffers property:
  bool copy_buffers_{true};

  bool remake_codec_elements();
  void make_codec_properties();
  void uninstall_codec_properties();
  void toggle_codec_properties(bool enable);
  void make_bin();
  void show();
  void hide();
  bool has_enough_channels(const std::string& str_caps);
  quiddity::property::prop_id_t install_codec();
  bool reset_codec_configuration();
  static gboolean sink_factory_filter(GstPluginFeature* feature, gpointer data);
  static gint sink_compare_ranks(GstPluginFeature* f1, GstPluginFeature* f2);
};

}  // namespace gst
}  // namespace switcher
#endif
