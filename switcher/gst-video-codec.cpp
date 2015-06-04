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
#include "./gst-video-codec.hpp"


namespace switcher {
GstVideoCodec::GstVideoCodec(Quiddity *quid):
    quid_(quid),
    custom_props_(std::make_shared<CustomPropertyHelper>()),
    video_output_format_(std2::make_unique<DefaultVideoFormat>(quid_)) {
  GstUtils::element_factory_list_to_g_enum(primary_codec_,
                                           GST_ELEMENT_FACTORY_TYPE_VIDEO_ENCODER,
                                           GST_RANK_PRIMARY,
                                           true);

  GstUtils::element_factory_list_to_g_enum(secondary_codec_,
                                           GST_ELEMENT_FACTORY_TYPE_VIDEO_ENCODER,
                                           GST_RANK_SECONDARY,
                                           true);
  primary_codec_spec_ =
      custom_props_->make_enum_property("codec",
                                        "Codec Short List",
                                        codec_,
                                        primary_codec_,
                                        (GParamFlags) G_PARAM_READWRITE,
                                        GstVideoCodec::set_codec,
                                        GstVideoCodec::get_codec,
                                        this);
  quid_->install_property_by_pspec(custom_props_->get_gobject(),
                                   primary_codec_spec_,
                                   "codec",
                                   "Video Codecs (Short List)");
  secondary_codec_spec_ =
      custom_props_->make_enum_property("codec",
                                        "Codec Long List",
                                        codec_,
                                        secondary_codec_,
                                        (GParamFlags) G_PARAM_READWRITE,
                                        GstVideoCodec::set_codec,
                                        GstVideoCodec::get_codec,
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
  // FIXME decide what to do with this
  // codec_long_list_spec_ =
  //     custom_props_->make_boolean_property("more_codecs",
  //                                          "Get More codecs",
  //                                          (gboolean) FALSE,
  //                                          (GParamFlags) G_PARAM_READWRITE,
  //                                          GstVideoCodec::set_codec_long_list,
  //                                          GstVideoCodec::get_codec_long_list,
  //                                          this);
  // install_property_by_pspec(custom_props_->get_gobject(),
  //                           codec_long_list_spec_,
  //                           "more_codecs", "More Codecs");
  shm_raw_path_ = quid_->make_file_name("video");
  shm_encoded_path_ = quid_->make_file_name("encoded");
  g_object_set(G_OBJECT(shm_raw_.get_raw()),
               "socket-path", shm_raw_path_.c_str(), nullptr);
  g_object_set(G_OBJECT(shm_encoded_.get_raw()),
               "socket-path", shm_encoded_path_.c_str(), nullptr);
  make_bin();
  video_output_format_->make_format_property("format", "Convert Pixel Format To");
  make_codec_properties();
  reset_codec_configuration(nullptr, this);
}

GstVideoCodec::~GstVideoCodec() {
  GstUtils::free_g_enum_values(primary_codec_);
  GstUtils::free_g_enum_values(secondary_codec_);
}

void GstVideoCodec::set_visible(bool visible){
  if (visible)
    show();
  hide();
}

void GstVideoCodec::hide(){
  quid_->disable_method("reset");
  quid_->disable_property("codec");
  quid_->disable_property("more_codecs");
  video_output_format_->disable_property();
}

void GstVideoCodec::show(){
  quid_->enable_method("reset");
  quid_->enable_property("codec");
  quid_->enable_property("more_codecs");
  video_output_format_->enable_property();
}

void GstVideoCodec::make_bin(){
  gst_bin_add(GST_BIN(bin_.get_raw()), video_tee_.get_raw());
  gst_bin_add(GST_BIN(bin_.get_raw()), shm_raw_.get_raw());
  std::string caps_str = video_output_format_->get_caps_str();
  if (caps_str != "none") {
    GstCaps *usercaps = gst_caps_from_string(caps_str.c_str());
    g_object_set(G_OBJECT(caps_filter_raw_.get_raw()),
                 "caps", usercaps,
                 nullptr);
    gst_caps_unref(usercaps);
    gst_bin_add(GST_BIN(bin_.get_raw()), color_space_raw_.get_raw());
    gst_bin_add(GST_BIN(bin_.get_raw()), caps_filter_raw_.get_raw());
    
    gst_element_link_many(video_tee_.get_raw(),
                          color_space_raw_.get_raw(),
                          caps_filter_raw_.get_raw(),
                          shm_raw_.get_raw(),
                          nullptr);
  } else {
    gst_element_link(video_tee_.get_raw(), shm_raw_.get_raw() );
  }
  if (codec_ != 0) {
  gst_bin_add_many(GST_BIN(bin_.get_raw()),
                   shm_encoded_.get_raw(),
                   color_space_codec_element_.get_raw(),
                   queue_codec_element_.get_raw(),
                   codec_element_.get_raw(),
                   nullptr);
  gst_element_link_many(video_tee_.get_raw(),
                        queue_codec_element_.get_raw(),
                        color_space_codec_element_.get_raw(),
                        codec_element_.get_raw(),
                        shm_encoded_.get_raw(),
                        nullptr);
  }
}

bool GstVideoCodec::remake_codec_elements() {
  for (auto &it : codec_properties_)
    quid_->uninstall_property(it);
  // elements are owner by the user
  if (!UGstElem::renew(shm_raw_, {"socket-path"})
     || !UGstElem::renew(video_tee_)
     || !UGstElem::renew(bin_)){
    g_warning("error renewing a shmdatasink related gst element");
    return false;
  }
  std::string caps_str = video_output_format_->get_caps_str();
  if (caps_str != "none") {
    if (!UGstElem::renew(color_space_raw_)
        || !UGstElem::renew(caps_filter_raw_)){
      g_warning("error renewing a video format related gst element");
      return false;
    }
  }
  if (codec_ != 0) {  //no codec
    if (!UGstElem::renew(shm_encoded_, {"socket-path"})
        || !UGstElem::renew(color_space_codec_element_)
        || !UGstElem::renew(queue_codec_element_)
        || !UGstElem::renew(codec_element_, codec_properties_)){
      g_warning("error renewing a codec related gst element");
      return false;
    }
    for (auto &it : codec_properties_) {
      quid_->install_property(G_OBJECT(codec_element_.get_raw()), it, it, it);
    }
  }  // end no codec
  make_bin();
  return true;
}

void GstVideoCodec::set_codec(const gint value, void *user_data) {
  GstVideoCodec *context = static_cast<GstVideoCodec *>(user_data);
  context->codec_ = value;
  context->codec_element_.mute(context->secondary_codec_[context->codec_].value_nick);
  context->remake_codec_elements();
}

gint GstVideoCodec::get_codec(void *user_data) {
  GstVideoCodec *context = static_cast<GstVideoCodec *>(user_data);
  return context->codec_;
}

// gboolean GstVideoCodec::get_codec_long_list(void *user_data) {
//   GstVideoCodec *context = static_cast<GstVideoCodec *>(user_data);
//   return context->codec_long_list_;
// }

// void
// GstVideoCodec::set_codec_long_list(gboolean codec_long_list,
//                                  void *user_data) {
//   GstVideoCodec *context = static_cast<GstVideoCodec *>(user_data);
//   context->codec_long_list_ = codec_long_list;

//   if (codec_long_list) {
//     context->uninstall_property("codec");
//     context->install_property_by_pspec(context->
//                                        custom_props_->get_gobject(),
//                                        context->secondary_codec_spec_,
//                                        "codec", "Video Codecs (Long List)");
//   } else {
//     context->uninstall_property("codec");
//     context->install_property_by_pspec(context->
//                                        custom_props_->get_gobject(),
//                                        context->primary_codec_spec_,
//                                        "codec",
//                                        "Video Codecs (Short List)");
//   }
//   // reset codec value
//   set_codec(0, context);
// }

void GstVideoCodec::make_codec_properties() {
  codec_properties_.push_back("quality");     // jpegenc
  codec_properties_.push_back("idct-method");  // jpegenc
  codec_properties_.push_back("speed-preset");        // x264
  codec_properties_.push_back("bitrate");     // x264
  codec_properties_.push_back("threads");     // x264
  codec_properties_.push_back("ref");  // x264
  codec_properties_.push_back("trellis");     // x264
  codec_properties_.push_back("key-int-max");  // x264
  codec_properties_.push_back("speed");       // vp8
  codec_properties_.push_back("mode");        // vp8
  codec_properties_.push_back("error-resilient");     // vp8
  codec_properties_.push_back("max-latency");  // vp8
  codec_properties_.push_back("max-keyframe-distance");       // vp8
  codec_properties_.push_back("qmin");        // smokeenc
  codec_properties_.push_back("qmax");        // smokeenc
  codec_properties_.push_back("keyframe");    // smokeenc
  codec_properties_.push_back("rate-control");        // schroenc (dirac)
  codec_properties_.push_back("max-bitrate");  // schroenc (dirac)
  codec_properties_.push_back("min-bitrate");  // schroenc (dirac)
  codec_properties_.push_back("snapshot");    // png
  codec_properties_.push_back("compression-level");   // png
}

gboolean GstVideoCodec::reset_codec_configuration(gpointer /*unused */ , gpointer user_data) {
  GstVideoCodec *context = static_cast<GstVideoCodec *>(user_data);
  context->quid_->set_property("format", "0");  // do not apply
  context->quid_->set_property("codec","vp8enc");
  context->quid_->set_property("quality","5");
  context->quid_->set_property("bitrate","0");
  context->quid_->set_property("threads","4");
  context->quid_->set_property("speed","7");
  context->quid_->set_property("mode","0");  // vbr
  context->quid_->set_property("error-resilient","true");
  context->quid_->set_property("max-latency","10");
  context->quid_->set_property("max-keyframe-distance","5");
  return TRUE;
}

}  // namespace switcher
