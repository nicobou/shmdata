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

#include "./gst-audio-codec.hpp"
#include "switcher/gprop-to-prop.hpp"
#include "switcher/gst-utils.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/startable-quiddity.hpp"

namespace switcher {
GstAudioCodec::GstAudioCodec(Quiddity* quid)
    : quid_(quid),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)),
      codecs_(GstUtils::element_factory_list_to_pair_of_vectors(
                  GST_ELEMENT_FACTORY_TYPE_AUDIO_ENCODER, GST_RANK_SECONDARY, true, {"vorbisenc"}),
              0),
      codec_id_(install_codec()) {
  quid_->install_method("Reset codec configuration",
                        "reset",
                        "Reset codec configuration to default",
                        "success or fail",
                        Method::make_arg_description("none", nullptr),
                        (Method::method_ptr)&reset_codec_configuration,
                        G_TYPE_BOOLEAN,
                        Method::make_arg_type_description(G_TYPE_NONE, nullptr),
                        this);
  reset_codec_configuration(nullptr, this);
  stop();
}

void GstAudioCodec::hide() {
  quid_->disable_method("reset");
  quid_->pmanage<MPtr(&PContainer::disable)>(codec_id_, StartableQuiddity::disabledWhenStartedMsg);
  quid_->pmanage<MPtr(&PContainer::disable)>(group_codec_id_,
                                             StartableQuiddity::disabledWhenStartedMsg);
}

void GstAudioCodec::show() {
  quid_->enable_method("reset");
  quid_->pmanage<MPtr(&PContainer::enable)>(codec_id_);
  quid_->pmanage<MPtr(&PContainer::enable)>(group_codec_id_);
}

void GstAudioCodec::make_bin() {
  if (0 != codecs_.get_current_index()) {
    gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                     shmsrc_.get_raw(),
                     queue_codec_element_.get_raw(),
                     audio_convert_.get_raw(),
                     audio_resample_.get_raw(),
                     codec_element_.get_raw(),
                     shm_encoded_.get_raw(),
                     nullptr);
    gst_element_link_many(shmsrc_.get_raw(),
                          queue_codec_element_.get_raw(),
                          audio_convert_.get_raw(),
                          audio_resample_.get_raw(),
                          codec_element_.get_raw(),
                          shm_encoded_.get_raw(),
                          nullptr);
  }
}

bool GstAudioCodec::remake_codec_elements() {
  if (0 != codecs_.get_current_index()) {
    if (!UGstElem::renew(shmsrc_, {"socket-path"}) ||
        !UGstElem::renew(shm_encoded_, {"socket-path", "sync", "async"}) ||
        !UGstElem::renew(audio_convert_) || !UGstElem::renew(audio_resample_) ||
        !UGstElem::renew(queue_codec_element_) ||
        !UGstElem::renew(codec_element_, codec_properties_)) {
      return false;
    }
  }  // end no codec
  return true;
}

void GstAudioCodec::uninstall_codec_properties() {
  for (auto& it : codec_properties_)
    quid_->pmanage<MPtr(&PContainer::remove)>(quid_->pmanage<MPtr(&PContainer::get_id)>(it));
  codec_properties_.clear();
}

void GstAudioCodec::toggle_codec_properties(bool enable) {
  for (auto& it : codec_properties_) {
    if (enable)
      quid_->pmanage<MPtr(&PContainer::enable)>(quid_->pmanage<MPtr(&PContainer::get_id)>(it));
    else
      quid_->pmanage<MPtr(&PContainer::disable)>(quid_->pmanage<MPtr(&PContainer::get_id)>(it),
                                                 StartableQuiddity::disabledWhenStartedMsg);
  }
}

void GstAudioCodec::make_codec_properties() {
  guint num_properties = 0;
  GParamSpec** props =
      g_object_class_list_properties(G_OBJECT_GET_CLASS(codec_element_.get_raw()), &num_properties);
  On_scope_exit { g_free(props); };
  group_codec_id_ = quid_->pmanage<MPtr(&PContainer::make_group)>(
      "codec_config", "Audio codec properties", "Configure the codec");
  for (guint i = 0; i < num_properties; i++) {
    auto param_name = g_param_spec_get_name(props[i]);
    if (param_black_list_.end() == param_black_list_.find(param_name)) {
      quid_->pmanage<MPtr(&PContainer::push_parented)>(
          param_name,
          "codec_config",
          GPropToProp::to_prop(G_OBJECT(codec_element_.get_raw()), param_name));
      codec_properties_.push_back(param_name);
    }
  }
}

gboolean GstAudioCodec::reset_codec_configuration(gpointer /*unused */, gpointer user_data) {
  GstAudioCodec* context = static_cast<GstAudioCodec*>(user_data);
  {
    auto lock = context->quid_->pmanage<MPtr(&PContainer::get_lock)>(context->codec_id_);
    context->codecs_.select(IndexOrName("Opus audio encoder"));
    context->quid_->pmanage<MPtr(&PContainer::notify)>(context->codec_id_);
  }
  return TRUE;
}

bool GstAudioCodec::start(const std::string& shmpath, const std::string& shmpath_encoded) {
  hide();
  toggle_codec_properties(false);
  if (0 == codecs_.get_current_index()) return false;

  shmpath_to_encode_ = shmpath;
  if (shmpath_encoded.empty())
    shm_encoded_path_ = shmpath_to_encode_ + "-encoded";
  else
    shm_encoded_path_ = shmpath_encoded;

  g_object_set(G_OBJECT(shmsrc_.get_raw()), "socket-path", shmpath_to_encode_.c_str(), nullptr);
  g_object_set(G_OBJECT(shm_encoded_.get_raw()),
               "socket-path",
               shm_encoded_path_.c_str(),
               "sync",
               FALSE,
               "async",
               FALSE,
               nullptr);

  shmsink_sub_ = std::make_unique<GstShmdataSubscriber>(
      shm_encoded_.get_raw(),
      [this](const std::string& caps) {
        this->quid_->graft_tree(
            ".shmdata.writer." + shm_encoded_path_,
            ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(quid_, ".shmdata.writer." + shm_encoded_path_));
  shmsrc_sub_ = std::make_unique<GstShmdataSubscriber>(
      shmsrc_.get_raw(),
      [this](const std::string& caps) {
        if (!this->has_enough_channels(caps)) {
          // FIXME: To do in can_sink_caps of audioenc when destination caps are implemented.
          quid_->message(
              "ERROR:audio codec does not support the number of channels connected to it.");
          quid_->warning("audio codec does not support the number of channels connected to it.");
        }
        this->quid_->graft_tree(
            ".shmdata.reader." + shmpath_to_encode_,
            ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(quid_, ".shmdata.reader." + shmpath_to_encode_));
  make_bin();
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  if (copy_buffers_) g_object_set(G_OBJECT(shmsrc_.get_raw()), "copy-buffers", TRUE, nullptr);
  gst_pipeline_->play(true);
  return true;
}

bool GstAudioCodec::stop() {
  show();
  toggle_codec_properties(true);
  if (0 != codecs_.get_current_index()) {
    shmsink_sub_.reset();
    shmsrc_sub_.reset();
    quid_->prune_tree(".shmdata.writer." + shm_encoded_path_);
    quid_->prune_tree(".shmdata.reader." + shmpath_to_encode_);
    remake_codec_elements();
    make_codec_properties();
    gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);
  }
  return true;
}

PContainer::prop_id_t GstAudioCodec::install_codec() {
  codecs_.select(IndexOrName("Opus audio encoder"));
  return quid_->pmanage<MPtr(&PContainer::make_selection<>)>(
      "codec",
      [this](const IndexOrName& val) {
        uninstall_codec_properties();
        codecs_.select(val);
        if (0 == val.index_) return true;
        codec_element_.mute(codecs_.get_attached().c_str());
        remake_codec_elements();
        make_codec_properties();
        return true;
      },
      [this]() { return codecs_.get(); },
      "Audio Codecs",
      "Selected audio codec for encoding",
      codecs_);
}

bool GstAudioCodec::has_enough_channels(const std::string& str_caps) {
  GstCaps* caps = gst_caps_from_string(str_caps.c_str());
  On_scope_exit {
    if (nullptr != caps) gst_caps_unref(caps);
  };

  GstStructure* s = gst_caps_get_structure(caps, 0);
  if (nullptr == s) {
    quid_->warning("Cannot get structure from caps (audioenc)");
    return false;
  }

  gint channels = 0;
  if (!gst_structure_get_int(s, "channels", &channels)) {
    quid_->warning("Cannot get channels number from shmdata description (audioenc)");
    return false;
  }

  std::pair<int, int> channels_range = GstUtils::get_gst_element_capability_as_range(
      gst_plugin_feature_get_name(gst_element_get_factory(codec_element_.get_raw())),
      "channels",
      GST_PAD_SINK);

  return channels <= channels_range.second;
}

}  // namespace switcher
