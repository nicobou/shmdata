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

#include <vector>
#include <unordered_set>
#include "switcher/unique-gst-element.hpp"
#include "switcher/custom-property-helper.hpp"
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {
class quiddity;

class GstVideoCodec {
 public:
  GstVideoCodec(Quiddity *quid,
                CustomPropertyHelper *prop_helper,
                const std::string &shmpath_to_encode,
                const std::string &shmpath_encoded = {});
  GstVideoCodec() = delete;
  ~GstVideoCodec();
  GstVideoCodec(const GstVideoCodec &) = delete;
  GstVideoCodec &operator=(const GstVideoCodec &) = delete;

  void set_shm(const std::string &shmpath);
  bool start();
  bool stop();
  
 private:
  Quiddity *quid_;
  // shmdata path
  std::string shmpath_to_encode_;
  std::string shm_encoded_path_;
  bool custom_shmsink_path_;
  // gst pipeline
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  // video encoding
  UGstElem shmsrc_{"shmdatasrc"};
  UGstElem queue_codec_element_{"queue"};
  UGstElem color_space_codec_element_{"videoconvert"};
  UGstElem codec_element_{"vp8enc"};
  UGstElem shm_encoded_{"shmdatasink"};
  std::unique_ptr<GstShmdataSubscriber> shmsrc_sub_{nullptr};
  std::unique_ptr<GstShmdataSubscriber> shmsink_sub_{nullptr};
  // codec props
  GParamSpec *primary_codec_spec_{nullptr};
  GEnumValue primary_codec_[128]{};
  GParamSpec *secondary_codec_spec_{nullptr};
  GEnumValue secondary_codec_[128]{};
  gint codec_{0};
  // short or long codec list
  GParamSpec *codec_long_list_spec_{nullptr};
  bool codec_long_list_{false};
  std::vector<std::string> codec_properties_{};
  CustomPropertyHelper *prop_helper_;
  // codec params black list
  std::unordered_set<std::string> param_black_list_{"name", "parent",
        "twopass-vbr-bias", "twopass-vbr-minsection", "twopass-vbr-maxsection",
        "multipass-mode", "multipass-cache-file",
        "snapshot",
        "temporal-scalability-target-bitrate", "temporal-scalability-rate-decimator",
        "temporal-scalability-periodicity", "temporal-scalability-layer-id",
        "error-resilient"};
  // shmdatasrc copy-buffers property:
  bool copy_buffers_{false};
  
  bool remake_codec_elements();
  void make_codec_properties();
  void uninstall_codec_properties();
  void make_bin();
  void show();
  void hide();
  static void set_codec(const gint value, void *user_data);
  static gint get_codec(void *user_data);
  // static gboolean get_codec_long_list(void *user_data);
  // static void set_codec_long_list(gboolean mute, void *user_data);
  static gboolean sink_factory_filter(GstPluginFeature *feature,
                                      gpointer data);
  static gint sink_compare_ranks(GstPluginFeature *f1,
                                 GstPluginFeature *f2);
  static gboolean reset_codec_configuration(gpointer /*unused */ , gpointer user_data);
};

}  // namespace switcher
#endif
