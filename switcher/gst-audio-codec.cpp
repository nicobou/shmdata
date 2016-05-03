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
#include "switcher/quiddity.hpp"
#include "switcher/gst-utils.hpp"
#include "switcher/std2.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/gprop-to-prop.hpp"

namespace switcher {
GstAudioCodec::GstAudioCodec(Quiddity *quid,
                             const std::string &shmpath,
                             const std::string &shmpath_encoded):
    quid_(quid),
    shmpath_to_encode_(shmpath),
    shm_encoded_path_(shmpath_encoded),
    custom_shmsink_path_(!shmpath_encoded.empty()),
    gst_pipeline_(std2::make_unique<GstPipeliner>(nullptr, nullptr)),
    primary_codec_(GstUtils::element_factory_list_to_pair_of_vectors(
        GST_ELEMENT_FACTORY_TYPE_AUDIO_ENCODER,
        GST_RANK_PRIMARY,
        true,
        {"vorbisenc"}), 0),
  secondary_codec_(GstUtils::element_factory_list_to_pair_of_vectors(
      GST_ELEMENT_FACTORY_TYPE_AUDIO_ENCODER,
      GST_RANK_SECONDARY,
      true,
      {"vorbisenc"}), 0),
  codec_id_(install_codec(true/*primary*/)),
  codec_long_list_id_(quid_->pmanage<MPtr(&PContainer::make_bool)>(
      "more_codecs",
      [this](const bool &val){
        codec_long_list_ = val;
        quid_->pmanage<MPtr(&PContainer::remove)>(codec_id_);
        if (val) 
          install_codec(true);   // primary
        else
          install_codec(false);  // secondary
        reset_codec_configuration(nullptr, this);
        return true;
      },
      [this](){return codec_long_list_;},
      "More Codecs",
      "Enable more codecs in selection",
      codec_long_list_)){
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
  set_shm(shmpath);
  reset_codec_configuration(nullptr, this);  
}

void GstAudioCodec::hide(){
  quid_->disable_method("reset");
  quid_->pmanage<MPtr(&PContainer::enable)>(codec_id_, false);
  quid_->pmanage<MPtr(&PContainer::enable)>(codec_long_list_id_, false);
}

void GstAudioCodec::show(){
  quid_->enable_method("reset");
  quid_->pmanage<MPtr(&PContainer::enable)>(codec_id_, true);
  quid_->pmanage<MPtr(&PContainer::enable)>(codec_long_list_id_, true);
}

void GstAudioCodec::make_bin(){
  if (0 != secondary_codec_.get()) {
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
  if (0 != secondary_codec_.get()) {
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


void GstAudioCodec::uninstall_codec_properties(){
  for (auto &it : codec_properties_)
    quid_->pmanage<MPtr(&PContainer::remove)>(
        quid_->pmanage<MPtr(&PContainer::get_id)>(it));
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
      quid_->pmanage<MPtr(&PContainer::push)>(
          param_name,
          GPropToProp::to_prop(G_OBJECT(codec_element_.get_raw()), param_name));
      codec_properties_.push_back(param_name);
    }
  }
}

gboolean GstAudioCodec::reset_codec_configuration(gpointer /*unused */ , gpointer user_data) {
  GstAudioCodec *context = static_cast<GstAudioCodec *>(user_data);
  auto codec_prop_id = context->quid_->pmanage<MPtr(&PContainer::get_id)>("codec");
  context->quid_->pmanage<MPtr(&PContainer::set_str)>(codec_prop_id, "opusenc");
  return TRUE;
}

bool GstAudioCodec::start(){
  hide();
  uninstall_codec_properties();
  if (0 != secondary_codec_.get()) 
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
                                InfoTree::make(byte_rate));
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
                                InfoTree::make(byte_rate));
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
  if (0 != secondary_codec_.get()) {
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

PContainer::prop_id_t GstAudioCodec::install_codec(bool primary){
  return quid_->pmanage<MPtr(&PContainer::make_selection)>(
      "codec",
      [this](const size_t &val){
        uninstall_codec_properties();
        // primary_codec_ is used for documentation, secondary is the one
        // used for actual work
        secondary_codec_.select(val);
        if (0 == val)
          return true;
        codec_element_.mute(secondary_codec_.get_current_nick().c_str());
        remake_codec_elements();
        make_codec_properties();
        return true;
      },
      [this](){return secondary_codec_.get();},
      "Audio Codecs",
      "Selected audio codec for encoding",
      primary ? primary_codec_ : secondary_codec_);
}

}  // namespace switcher
