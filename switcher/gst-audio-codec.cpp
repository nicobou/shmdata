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

#include "switcher/quiddity.hpp"
#include "switcher/gst-utils.hpp"
#include "switcher/std2.hpp"
#include "switcher/scope-exit.hpp"
#include "./gst-audio-codec.hpp"


namespace switcher {
GstAudioCodec::GstAudioCodec(Quiddity *quid,
                             CustomPropertyHelper *prop_helper,
                             const std::string &shmpath,
                             const std::string &shmpath_encoded):
    quid_(quid),
    shmpath_to_encode_(shmpath),
    shm_encoded_path_(shmpath_encoded),
    custom_shmsink_path_(!shmpath_encoded.empty()),
    gst_pipeline_(std2::make_unique<GstPipeliner>(nullptr, nullptr)),
    prop_helper_(prop_helper){
  GstUtils::element_factory_list_to_g_enum(primary_codec_,
                                           GST_ELEMENT_FACTORY_TYPE_AUDIO_ENCODER,
                                           GST_RANK_PRIMARY,
                                           true,
                                           {"vorbisenc"});  // black list
  GstUtils::element_factory_list_to_g_enum(secondary_codec_,
                                           GST_ELEMENT_FACTORY_TYPE_AUDIO_ENCODER,
                                           GST_RANK_SECONDARY,
                                           true,
                                           {"vorbisenc"});
  primary_codec_spec_ =
      prop_helper_->make_enum_property("codec",
                                        "Codec Short List",
                                        codec_,
                                        primary_codec_,
                                        (GParamFlags) G_PARAM_READWRITE,
                                        GstAudioCodec::set_codec,
                                        GstAudioCodec::get_codec,
                                        this);
    quid_->install_property_by_pspec(prop_helper_->get_gobject(),
                                     primary_codec_spec_,
                                     "codec",
                                     "Audio Codecs (Short List)");
  secondary_codec_spec_ =
      prop_helper_->make_enum_property("codec",
                                       "Codec Long List",
                                       codec_,
                                       secondary_codec_,
                                       (GParamFlags) G_PARAM_READWRITE,
                                       GstAudioCodec::set_codec,
                                       GstAudioCodec::get_codec,
                                       this);
  quid_->install_method("Reset codec configuration",
                        "reset",
                        "Reset codec configuration to default",
                        "success or fail",
                        Method::make_arg_description("none", nullptr),
                        (Method::method_ptr) &reset_codec_configuration,
                        G_TYPE_BOOLEAN,
                        Method::make_arg_type_description(G_TYPE_NONE,
                                                          nullptr),
                        this);
  codec_long_list_spec_ =
      prop_helper_->make_boolean_property("more_codecs",
                                           "Get More codecs",
                                           (gboolean) FALSE,
                                           (GParamFlags) G_PARAM_READWRITE,
                                           GstAudioCodec::set_codec_long_list,
                                           GstAudioCodec::get_codec_long_list,
                                           this);
  quid->install_property_by_pspec(prop_helper_->get_gobject(),
                                  codec_long_list_spec_,
                                  "more_codecs", "More Codecs");
  set_shm(shmpath);
  reset_codec_configuration(nullptr, this);
}

GstAudioCodec::~GstAudioCodec() {
  GstUtils::free_g_enum_values(primary_codec_);
  GstUtils::free_g_enum_values(secondary_codec_);
}

void GstAudioCodec::hide(){
  quid_->disable_method("reset");
  quid_->disable_property("codec");
  quid_->disable_property("more_codecs");
}

void GstAudioCodec::show(){
  quid_->enable_method("reset");
  quid_->enable_property("codec");
  quid_->enable_property("more_codecs");
}

void GstAudioCodec::make_bin(){
  if (codec_ != 0) {
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
  if (codec_ != 0) {  //no codec
    if (!UGstElem::renew(shmsrc_, {"socket-path"})
        || !UGstElem::renew(shm_encoded_, {"socket-path", "sync", "async"})
        || !UGstElem::renew(audio_convert_)
        || !UGstElem::renew(audio_resample_)
        || !UGstElem::renew(queue_codec_element_)
        || !UGstElem::renew(codec_element_, codec_properties_)){
      g_warning("error renewing a codec related gst element");
      return false;
    }
  }  // end no codec
  return true;
}

void GstAudioCodec::set_codec(const gint value, void *user_data) {
  GstAudioCodec *context = static_cast<GstAudioCodec *>(user_data);
  context->uninstall_codec_properties();
  context->codec_ = value;
  if (0 == value)
    return;
  std::string codec_name = context->secondary_codec_[context->codec_].value_nick;
  context->codec_element_.mute(codec_name.c_str());
  context->remake_codec_elements();
  context->make_codec_properties();
}

gint GstAudioCodec::get_codec(void *user_data) {
  GstAudioCodec *context = static_cast<GstAudioCodec *>(user_data);
  return context->codec_;
}

gboolean GstAudioCodec::get_codec_long_list(void *user_data) {
  GstAudioCodec *context = static_cast<GstAudioCodec *>(user_data);
  return context->codec_long_list_;
}

void
GstAudioCodec::set_codec_long_list(gboolean codec_long_list,
                                 void *user_data) {
  GstAudioCodec *context = static_cast<GstAudioCodec *>(user_data);
  context->codec_long_list_ = codec_long_list;

  if (codec_long_list) {
    context->quid_->uninstall_property("codec");
    context->quid_->install_property_by_pspec(context->prop_helper_->get_gobject(),
                                              context->secondary_codec_spec_,
                                              "codec", "Audio Codecs (Long List)");
  } else {
    context->quid_->uninstall_property("codec");
    context->quid_->install_property_by_pspec(context->prop_helper_->get_gobject(),
                                              context->primary_codec_spec_,
                                              "codec",
                                              "Audio Codecs (Short List)");
  }
  // reset codec value
  reset_codec_configuration(nullptr, context);
}

void GstAudioCodec::uninstall_codec_properties(){
  for (auto &it : codec_properties_)
    quid_->uninstall_property(it);
  codec_properties_.clear();
}

void GstAudioCodec::make_codec_properties() {
  guint num_properties = 0;
  GParamSpec **props = g_object_class_list_properties(
      G_OBJECT_GET_CLASS(codec_element_.get_raw()), &num_properties);
  On_scope_exit{g_free(props);};
  for (guint i = 0; i < num_properties; i++) {
    auto param_name = g_param_spec_get_name(props[i]);
    if (param_black_list_.end() == param_black_list_.find(param_name)){
      quid_->install_property_by_pspec(G_OBJECT(codec_element_.get_raw()),
                                       props[i],
                                       param_name,
                                       g_param_spec_get_nick(props[i]));
      codec_properties_.push_back(param_name);
    }
  }
}

gboolean GstAudioCodec::reset_codec_configuration(gpointer /*unused */ , gpointer user_data) {
  GstAudioCodec *context = static_cast<GstAudioCodec *>(user_data);
  context->quid_->set_property("codec","opusenc");
  // context->make_codec_properties();
  // context->quid_->set_property("deadline","30000");  //30ms
  // context->quid_->set_property("target-bitrate", "2000000"); // 2Mbps
  // context->quid_->set_property("end-usage", "1"); // CBR
  // context->quid_->set_property("keyframe-max-dist", "5");
  return TRUE;
}

bool GstAudioCodec::start(){
  hide();
  uninstall_codec_properties();
  if (0 == codec_)
    return true;
  shmsink_sub_ = std2::make_unique<GstShmdataSubscriber>(
      shm_encoded_.get_raw(),
      [this]( const std::string &caps){
        this->quid_->graft_tree(".shmdata.writer." + shm_encoded_path_,
                                ShmdataUtils::make_tree(caps,
                                                        ShmdataUtils::get_category(caps),
                                                        0));
      },
      [this](GstShmdataSubscriber::num_bytes_t byte_rate){
        this->quid_->graft_tree(".shmdata.writer." + shm_encoded_path_ + ".byte_rate",
                                data::Tree::make(std::to_string(byte_rate)));
      });
  shmsrc_sub_ = std2::make_unique<GstShmdataSubscriber>(
      shmsrc_.get_raw(),
      [this]( const std::string &caps){
        this->quid_->graft_tree(".shmdata.reader." + shmpath_to_encode_,
                                ShmdataUtils::make_tree(caps,
                                                        ShmdataUtils::get_category(caps),
                                                        0));
      },
      [this](GstShmdataSubscriber::num_bytes_t byte_rate){
        this->quid_->graft_tree(".shmdata.reader." + shmpath_to_encode_ + ".byte_rate",
                                data::Tree::make(std::to_string(byte_rate)));
      });
  make_bin();
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()),
               "async-handling", TRUE,
               nullptr);
  if (copy_buffers_)
    g_object_set(G_OBJECT(shmsrc_.get_raw()),
                 "copy-buffers", TRUE,
                 nullptr);
  gst_pipeline_->play(true);
  return true;
}

bool GstAudioCodec::stop(){
  show();
  if (0 != codec_) {
    shmsink_sub_.reset();
    shmsrc_sub_.reset();
    quid_->prune_tree(".shmdata.writer." + shm_encoded_path_);
    quid_->prune_tree(".shmdata.reader." + shmpath_to_encode_);
    remake_codec_elements();
    make_codec_properties();
    gst_pipeline_ = std2::make_unique<GstPipeliner>(nullptr, nullptr);
  }
  return true;
}

void GstAudioCodec::set_shm(const std::string &shmpath){
  shmpath_to_encode_ = shmpath;
  if (!custom_shmsink_path_)
    shm_encoded_path_ = shmpath_to_encode_ + "-encoded";
  g_object_set(G_OBJECT(shmsrc_.get_raw()),
               "socket-path", shmpath_to_encode_.c_str(),
               nullptr);
  g_object_set(G_OBJECT(shm_encoded_.get_raw()),
               "socket-path", shm_encoded_path_.c_str(),
               "sync", FALSE,
               "async", FALSE,
               nullptr);
}

}  // namespace switcher
