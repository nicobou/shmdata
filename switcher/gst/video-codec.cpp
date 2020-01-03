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

#include "./video-codec.hpp"
#include "../infotree/information-tree-json.hpp"
#include "../quiddity/property/gprop-to-prop.hpp"
#include "../quiddity/quiddity.hpp"
#include "../quiddity/startable.hpp"
#include "../utils/scope-exit.hpp"
#include "./utils.hpp"

namespace switcher {
namespace gst {
VideoCodec::VideoCodec(quiddity::Quiddity* quid,
                       const std::string& shmpath,
                       const std::string& shmpath_encoded)
    : quid_(quid),
      reset_id_(quid_->mmanage<MPtr(&quiddity::method::MBag::make_method<std::function<bool()>>)>(
          "reset",
          infotree::json::deserialize(
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
      gst_pipeline_(std::make_unique<Pipeliner>(nullptr, nullptr)),
      codecs_(gst::utils::element_factory_list_to_pair_of_vectors(
                  GST_ELEMENT_FACTORY_TYPE_VIDEO_ENCODER,
                  GST_RANK_SECONDARY,
                  true,
                  {"schroenc", "theoraenc"}),
              0),
      codec_id_(install_codec()),
      param_group_id_(quid_->pmanage<MPtr(&quiddity::property::PBag::make_group)>(
          "codec_params", "Codec configuration", "Codec specific parameters")) {
  set_shm(shmpath);
  reset_codec_configuration();
  quid_->pmanage<MPtr(&quiddity::property::PBag::set_to_current)>(codec_id_);
}

void VideoCodec::hide() {
  quid_->mmanage<MPtr(&quiddity::method::MBag::disable)>(
      reset_id_, quiddity::Startable::disabledWhenStartedMsg);
  quid_->pmanage<MPtr(&quiddity::property::PBag::disable)>(
      codec_id_, quiddity::Startable::disabledWhenStartedMsg);
  quid_->pmanage<MPtr(&quiddity::property::PBag::disable)>(
      param_group_id_, quiddity::Startable::disabledWhenStartedMsg);
}

void VideoCodec::show() {
  quid_->mmanage<MPtr(&quiddity::method::MBag::enable)>(reset_id_);
  quid_->pmanage<MPtr(&quiddity::property::PBag::enable)>(codec_id_);
  quid_->pmanage<MPtr(&quiddity::property::PBag::enable)>(param_group_id_);
}

void VideoCodec::make_bin() {
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

bool VideoCodec::remake_codec_elements() {
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

void VideoCodec::uninstall_codec_properties() {
  for (auto& it : codec_properties_)
    quid_->pmanage<MPtr(&quiddity::property::PBag::remove)>(
        quid_->pmanage<MPtr(&quiddity::property::PBag::get_id)>(it));
  codec_properties_.clear();
}

void VideoCodec::make_codec_properties() {
  uninstall_codec_properties();
  guint num_properties = 0;
  GParamSpec** props =
      g_object_class_list_properties(G_OBJECT_GET_CLASS(codec_element_.get_raw()), &num_properties);
  On_scope_exit { g_free(props); };
  for (guint i = 0; i < num_properties; i++) {
    auto param_name = g_param_spec_get_name(props[i]);
    if (param_black_list_.end() == param_black_list_.find(param_name)) {
      quid_->pmanage<MPtr(&quiddity::property::PBag::push_parented)>(
          param_name,
          "codec_params",
          quiddity::property::to_prop(G_OBJECT(codec_element_.get_raw()), param_name));
      codec_properties_.push_back(param_name);
    }
  }
}

bool VideoCodec::reset_codec_configuration() {
  auto& quid = quid_;
  auto* codec_sel = &codecs_;
  codec_sel->select(codecs_.get_index("x264enc"));
  quid->pmanage<MPtr(&quiddity::property::PBag::notify)>(codec_id_);
  make_codec_properties();
  quid->pmanage<MPtr(&quiddity::property::PBag::set_str)>(
      quid->pmanage<MPtr(&quiddity::property::PBag::get_id)>("bitrate"), "4096");
  quid->pmanage<MPtr(&quiddity::property::PBag::set_str)>(
      quid->pmanage<MPtr(&quiddity::property::PBag::get_id)>("pass"), "cbr");
  return true;
}

bool VideoCodec::start() {
  hide();
  if (0 == quid_
               ->pmanage<MPtr(&quiddity::property::PBag::get<quiddity::property::IndexOrName>)>(
                   codec_id_)
               .index_)
    return true;
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

bool VideoCodec::stop() {
  show();
  if (0 != quid_
               ->pmanage<MPtr(&quiddity::property::PBag::get<quiddity::property::IndexOrName>)>(
                   codec_id_)
               .index_) {
    shmsink_sub_.reset();
    shmsrc_sub_.reset();
    remake_codec_elements();
    make_codec_properties();
    gst_pipeline_ = std::make_unique<Pipeliner>(nullptr, nullptr);
  }
  return true;
}

void VideoCodec::set_shm(const std::string& shmpath) {
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

quiddity::property::prop_id_t VideoCodec::install_codec() {
  return quid_->pmanage<MPtr(&quiddity::property::PBag::make_selection<>)>(
      "codec",
      [this](const quiddity::property::IndexOrName& val) {
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

void VideoCodec::set_none() {
  quid_->pmanage<MPtr(&quiddity::property::PBag::set<quiddity::property::IndexOrName>)>(
      quid_->pmanage<MPtr(&quiddity::property::PBag::get_id)>("codec"), 0);
}

}  // namespace gst
}  // namespace switcher
