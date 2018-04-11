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
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/shmdata-utils.hpp"
#include "switcher/unique-gst-element.hpp"

namespace switcher {
class quiddity;

class GstAudioCodec {
 public:
  GstAudioCodec(Quiddity* quid);
  GstAudioCodec() = delete;
  ~GstAudioCodec() = default;
  GstAudioCodec(const GstAudioCodec&) = delete;
  GstAudioCodec& operator=(const GstAudioCodec&) = delete;

  bool start(const std::string& shmpath, const std::string& shmpath_encoded = {});
  bool stop();

 private:
  Quiddity* quid_;
  MContainer::meth_id_t reset_id_;
  // shmdata path
  std::string shmpath_to_encode_{};
  std::string shm_encoded_path_{};
  // gst pipeline
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  // audio encoding
  UGstElem shmsrc_{"shmdatasrc"};
  UGstElem queue_codec_element_{"queue"};
  UGstElem audio_convert_{"audioconvert"};
  UGstElem audio_resample_{"audioresample"};
  UGstElem codec_element_{"opusenc"};
  UGstElem shm_encoded_{"shmdatasink"};
  std::unique_ptr<GstShmdataSubscriber> shmsrc_sub_{nullptr};
  std::unique_ptr<GstShmdataSubscriber> shmsink_sub_{nullptr};
  // codec props
  Selection<> codecs_;
  PContainer::prop_id_t codec_id_;
  std::vector<std::string> codec_properties_{};
  // codec params black list
  std::unordered_set<std::string> param_black_list_{
      "name", "parent", "hard-resync", "mark-granule", "perfect-timestamp", "tolerance"};
  PContainer::prop_id_t group_codec_id_{0};
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
  PContainer::prop_id_t install_codec();
  bool reset_codec_configuration();
  static gboolean sink_factory_filter(GstPluginFeature* feature, gpointer data);
  static gint sink_compare_ranks(GstPluginFeature* f1, GstPluginFeature* f2);
};

}  // namespace switcher
#endif
