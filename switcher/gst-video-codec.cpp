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

#include "./gst-video-codec.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/gst-utils.hpp"
#include "switcher/std2.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/gprop-to-prop.hpp"

namespace switcher {
GstVideoCodec::GstVideoCodec(Quiddity *quid,
                             const std::string &shmpath,
                             const std::string &shmpath_encoded):
    quid_(quid),
    shmpath_to_encode_(shmpath),
    shm_encoded_path_(shmpath_encoded),
    custom_shmsink_path_(!shmpath_encoded.empty()),
    gst_pipeline_(std2::make_unique<GstPipeliner>(nullptr, nullptr)),
    primary_codec_(GstUtils::element_factory_list_to_pair_of_vectors(
        GST_ELEMENT_FACTORY_TYPE_VIDEO_ENCODER,
        GST_RANK_PRIMARY,
        true,
        {"schroenc", "theoraenc"}), 0),
  secondary_codec_(GstUtils::element_factory_list_to_pair_of_vectors(
      GST_ELEMENT_FACTORY_TYPE_VIDEO_ENCODER,
      GST_RANK_SECONDARY,
      true,
      {"schroenc", "theoraenc"}), 0),
  codec_id_(install_codec()),
  codec_long_list_id_(quid_->pmanage<MPtr(&PContainer::make_bool)>(
      "more_codecs",
      [this](const bool &val){
        codec_long_list_ = val;
        quid_->pmanage<MPtr(&PContainer::remove)>(codec_id_);
        if (val) {
          use_primary_codec_ = false;
          codec_id_ = install_codec();
          use_primary_codec_ = true;
        } else {
          use_primary_codec_ = false;
          codec_id_ = install_codec();
        }
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

void GstVideoCodec::hide(){
  quid_->disable_method("reset");
  quid_->pmanage<MPtr(&PContainer::enable)>(codec_id_, false);
  quid_->pmanage<MPtr(&PContainer::enable)>(codec_long_list_id_, false);
}

void GstVideoCodec::show(){
  quid_->enable_method("reset");
  quid_->pmanage<MPtr(&PContainer::enable)>(codec_id_, true);
  quid_->pmanage<MPtr(&PContainer::enable)>(codec_long_list_id_, true);
}

void GstVideoCodec::make_bin(){
  if (0 != use_primary_codec_ ? primary_codec_.get() : secondary_codec_.get()) {
    gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                     shmsrc_.get_raw(),
                     queue_codec_element_.get_raw(),
                     color_space_codec_element_.get_raw(),
                     codec_element_.get_raw(),
                     shm_encoded_.get_raw(),
                     nullptr);
    gst_element_link_many(shmsrc_.get_raw(),
                          queue_codec_element_.get_raw(),
                          color_space_codec_element_.get_raw(),
                          codec_element_.get_raw(),
                          shm_encoded_.get_raw(),
                          nullptr);
  }
}

bool GstVideoCodec::remake_codec_elements() {
  if (0 != quid_->pmanage<MPtr(&PContainer::get<Selection::index_t>)>(codec_id_)) {
    if (!UGstElem::renew(shmsrc_, {"socket-path"})
        || !UGstElem::renew(shm_encoded_, {"socket-path", "sync", "async"})
        || !UGstElem::renew(color_space_codec_element_)
        || !UGstElem::renew(queue_codec_element_)
        || !UGstElem::renew(codec_element_, codec_properties_)){
      g_warning("error renewing a codec related gst element");
      return false;
    }
  }  // end no codec
  return true;
}


void GstVideoCodec::uninstall_codec_properties(){
  for (auto &it : codec_properties_)
    quid_->pmanage<MPtr(&PContainer::remove)>(
        quid_->pmanage<MPtr(&PContainer::get_id)>(it));
  codec_properties_.clear();
}

void GstVideoCodec::make_codec_properties() {
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

gboolean GstVideoCodec::reset_codec_configuration(gpointer /*unused */ , gpointer user_data) {
  GstVideoCodec *context = static_cast<GstVideoCodec *>(user_data);
  auto &quid = context->quid_;
  g_print("%s %d \n", __FUNCTION__, __LINE__);
  // quid->pmanage<MPtr(&PContainer::set<Selection::index_t>)>(
  //     context->codec_id_, 5);
  auto *codec_sel = context->use_primary_codec_ ?
      &context->primary_codec_ : &context->secondary_codec_;
  codec_sel->select(context->secondary_codec_.get_index("On2 VP8 Encoder"));
  quid->pmanage<MPtr(&PContainer::notify)>(context->codec_id_);
  context->make_codec_properties();
  quid->pmanage<MPtr(&PContainer::set_str)>(
      quid->pmanage<MPtr(&PContainer::get_id)>("deadline"), "30000");
  quid->pmanage<MPtr(&PContainer::set_str)>(
      quid->pmanage<MPtr(&PContainer::get_id)>("target-bitrate"), "2000000");  // 2Mbps
  quid->pmanage<MPtr(&PContainer::set_str)>(
      quid->pmanage<MPtr(&PContainer::get_id)>("end-usage"), "1");  // CBR
  quid->pmanage<MPtr(&PContainer::set_str)>(
      quid->pmanage<MPtr(&PContainer::get_id)>("keyframe-max-dist"), "5");
  return TRUE;
}

bool GstVideoCodec::start(){
  hide();
  uninstall_codec_properties();
  if (0 == quid_->pmanage<MPtr(&PContainer::get<Selection::index_t>)>(codec_id_)) 
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
                                InfoTree::make(std::to_string(byte_rate)));
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
                                InfoTree::make(std::to_string(byte_rate)));
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

bool GstVideoCodec::stop(){
  show();
  if (0 != quid_->pmanage<MPtr(&PContainer::get<Selection::index_t>)>(codec_id_)) {
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

void GstVideoCodec::set_shm(const std::string &shmpath){
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

PContainer::prop_id_t GstVideoCodec::install_codec(){
  return quid_->pmanage<MPtr(&PContainer::make_selection)>(
      "codec",
      [this](const Selection::index_t &val){
        uninstall_codec_properties();
        use_primary_codec_ ? primary_codec_.select(val) : secondary_codec_.select(val);
        if (0 == val)
          return true;
        std::string codec_name = use_primary_codec_ ?
            primary_codec_.get_current_nick() : secondary_codec_.get_current_nick();
        codec_element_.mute(codec_name.c_str());
        if (codec_name == "x264enc") copy_buffers_ = true;
        else copy_buffers_ = false;
        remake_codec_elements();
        make_codec_properties();
        if (codec_name == "x264enc")
          g_object_set(G_OBJECT(codec_element_.get_raw()), "byte-stream", TRUE, nullptr);
        return true;
      },
      [this](){return use_primary_codec_ ? primary_codec_.get() : secondary_codec_.get();},
      "Video Codecs",
      "Selected video codec for encoding",
      use_primary_codec_ ? primary_codec_ : secondary_codec_);
}

}  // namespace switcher
