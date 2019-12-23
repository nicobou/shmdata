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
#include "../infotree/information-tree-json.hpp"
#include "../quiddity/property/gprop-to-prop.hpp"
#include "../quiddity/quiddity.hpp"
#include "../quiddity/startable-quiddity.hpp"
#include "../utils/scope-exit.hpp"
#include "./gst-utils.hpp"

namespace switcher {
GstVideoCodec::GstVideoCodec(Quiddity* quid,
                             const std::string& shmpath,
                             const std::string& shmpath_encoded)
    : quid_(quid),
      reset_id_(quid_->mmanage<MPtr(&MContainer::make_method<std::function<bool()>>)>(
          "reset",
          JSONSerializer::deserialize(
              R"(
                  {
                   "name" : "Reset codec configuration",
                   "description" : "Reset codec configuration to default",
                   "arguments" : []
                  }
              )"),
          [this]() { return reset_codec_configuration(); })),
      shmpath_to_encode_(shmpath),
      shm_encoded_path_(shmpath_encoded),
      custom_shmsink_path_(!shmpath_encoded.empty()),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)),
      codecs_(
          GstUtils::element_factory_list_to_pair_of_vectors(GST_ELEMENT_FACTORY_TYPE_VIDEO_ENCODER,
                                                            GST_RANK_SECONDARY,
                                                            true,
                                                            {"schroenc", "theoraenc"}),
          0),
      codec_id_(install_codec()),
      param_group_id_(quid_->pmanage<MPtr(&PContainer::make_group)>(
          "codec_params", "Codec configuration", "Codec specific parameters")) {
  set_shm(shmpath);
  reset_codec_configuration();
  quid_->pmanage<MPtr(&PContainer::set_to_current)>(codec_id_);
}

void GstVideoCodec::hide() {
  quid_->mmanage<MPtr(&MContainer::disable)>(reset_id_, StartableQuiddity::disabledWhenStartedMsg);
  quid_->pmanage<MPtr(&PContainer::disable)>(codec_id_, StartableQuiddity::disabledWhenStartedMsg);
  quid_->pmanage<MPtr(&PContainer::disable)>(param_group_id_,
                                             StartableQuiddity::disabledWhenStartedMsg);
}

void GstVideoCodec::show() {
  quid_->mmanage<MPtr(&MContainer::enable)>(reset_id_);
  quid_->pmanage<MPtr(&PContainer::enable)>(codec_id_);
  quid_->pmanage<MPtr(&PContainer::enable)>(param_group_id_);
}

void GstVideoCodec::make_bin() {
  if (0 != codecs_.get_current_index()) {
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
  if (0 != codecs_.get_current_index()) {
    if (!UGstElem::renew(shmsrc_, {"socket-path"}) ||
        !UGstElem::renew(shm_encoded_, {"socket-path", "sync", "async"}) ||
        !UGstElem::renew(color_space_codec_element_) || !UGstElem::renew(queue_codec_element_) ||
        !UGstElem::renew(codec_element_, codec_properties_)) {
      return false;
    }
  }  // end no codec
  return true;
}

void GstVideoCodec::uninstall_codec_properties() {
  for (auto& it : codec_properties_)
    quid_->pmanage<MPtr(&PContainer::remove)>(quid_->pmanage<MPtr(&PContainer::get_id)>(it));
  codec_properties_.clear();
}

void GstVideoCodec::make_codec_properties() {
  uninstall_codec_properties();
  guint num_properties = 0;
  GParamSpec** props =
      g_object_class_list_properties(G_OBJECT_GET_CLASS(codec_element_.get_raw()), &num_properties);
  On_scope_exit { g_free(props); };
  for (guint i = 0; i < num_properties; i++) {
    auto param_name = g_param_spec_get_name(props[i]);
    if (param_black_list_.end() == param_black_list_.find(param_name)) {
      quid_->pmanage<MPtr(&PContainer::push_parented)>(
          param_name,
          "codec_params",
          GPropToProp::to_prop(G_OBJECT(codec_element_.get_raw()), param_name));
      codec_properties_.push_back(param_name);
    }
  }
}

bool GstVideoCodec::reset_codec_configuration() {
  auto& quid = quid_;
  auto* codec_sel = &codecs_;
  codec_sel->select(codecs_.get_index("x264enc"));
  quid->pmanage<MPtr(&PContainer::notify)>(codec_id_);
  make_codec_properties();
  quid->pmanage<MPtr(&PContainer::set_str)>(quid->pmanage<MPtr(&PContainer::get_id)>("bitrate"),
                                            "4096");
  quid->pmanage<MPtr(&PContainer::set_str)>(quid->pmanage<MPtr(&PContainer::get_id)>("pass"),
                                            "cbr");
  return true;
}

bool GstVideoCodec::start() {
  hide();
  if (0 == quid_->pmanage<MPtr(&PContainer::get<IndexOrName>)>(codec_id_).index_) return true;
  shmsink_sub_ = std::make_unique<GstShmTreeUpdater>(
      quid_, shm_encoded_.get_raw(), shm_encoded_path_, GstShmTreeUpdater::Direction::writer);
  shmsrc_sub_ = std::make_unique<GstShmTreeUpdater>(
      quid_, shmsrc_.get_raw(), shmpath_to_encode_, GstShmTreeUpdater::Direction::reader);
  make_bin();

  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  if (copy_buffers_) g_object_set(G_OBJECT(shmsrc_.get_raw()), "copy-buffers", TRUE, nullptr);
  gst_pipeline_->play(true);
  return true;
}

bool GstVideoCodec::stop() {
  show();
  if (0 != quid_->pmanage<MPtr(&PContainer::get<IndexOrName>)>(codec_id_).index_) {
    shmsink_sub_.reset();
    shmsrc_sub_.reset();
    remake_codec_elements();
    make_codec_properties();
    gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);
  }
  return true;
}

void GstVideoCodec::set_shm(const std::string& shmpath) {
  shmpath_to_encode_ = shmpath;
  if (!custom_shmsink_path_) shm_encoded_path_ = shmpath_to_encode_ + "-encoded";
  g_object_set(G_OBJECT(shmsrc_.get_raw()),
               "do-timestamp",
               TRUE,
               "socket-path",
               shmpath_to_encode_.c_str(),
               nullptr);
  g_object_set(G_OBJECT(shm_encoded_.get_raw()),
               "socket-path",
               shm_encoded_path_.c_str(),
               "sync",
               FALSE,
               "async",
               FALSE,
               nullptr);
}

PContainer::prop_id_t GstVideoCodec::install_codec() {
  return quid_->pmanage<MPtr(&PContainer::make_selection<>)>(
      "codec",
      [this](const IndexOrName& val) {
        uninstall_codec_properties();
        codecs_.select(val);
        if (0 == val.index_) return true;
        std::string codec_name = codecs_.get_attached();
        codec_element_.mute(codec_name.c_str());
        if (codec_name == "x264enc")
          copy_buffers_ = true;
        else
          copy_buffers_ = false;
        remake_codec_elements();
        make_codec_properties();
        if (codec_name == "x264enc")
          g_object_set(G_OBJECT(codec_element_.get_raw()), "byte-stream", TRUE, nullptr);
        return true;
      },
      [this]() { return codecs_.get(); },
      "Video Codecs",
      "Selected video codec for encoding",
      codecs_);
}

void GstVideoCodec::set_none() {
  quid_->pmanage<MPtr(&PContainer::set<IndexOrName>)>(
      quid_->pmanage<MPtr(&PContainer::get_id)>("codec"), 0);
}

}  // namespace switcher
