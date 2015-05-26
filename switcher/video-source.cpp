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

#include "./video-source.hpp"
#include "./gst-utils.hpp"
#include "./std2.hpp"
#include "./scope-exit.hpp"

namespace switcher {
VideoSource::VideoSource():
    custom_props_(std::make_shared<CustomPropertyHelper>()),
    video_output_format_(std2::make_unique<DefaultVideoFormat>(this)) {
  init_startable(this);
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
                                        VideoSource::set_codec,
                                        VideoSource::get_codec,
                                        this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            primary_codec_spec_,
                            "codec",
                            "Video Codecs (Short List)");
  secondary_codec_spec_ =
      custom_props_->make_enum_property("codec",
                                        "Codec Long List",
                                        codec_,
                                        secondary_codec_,
                                        (GParamFlags) G_PARAM_READWRITE,
                                        VideoSource::set_codec,
                                        VideoSource::get_codec,
                                        this);
  install_method("Reset codec configuration",
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
  //                                          VideoSource::set_codec_long_list,
  //                                          VideoSource::get_codec_long_list,
  //                                          this);
  // install_property_by_pspec(custom_props_->get_gobject(),
  //                           codec_long_list_spec_,
  //                           "more_codecs", "More Codecs");
  video_output_format_->make_format_property("format", "Convert Pixel Format To");
  make_codec_properties();
  reset_codec_configuration(nullptr, this);
}

VideoSource::~VideoSource() {
  GstUtils::free_g_enum_values(primary_codec_);
  GstUtils::free_g_enum_values(secondary_codec_);
}

bool VideoSource::make_new_shmdatas() {
  clear_shmdatas();
  reset_bin();
  if (!GstUtils::make_element("tee", &video_tee_)) {
    g_warning("missing element for starting video source\n");
    return false;
  }

  gst_bin_add_many(GST_BIN(get_bin()),
                   rawvideo_,
                   video_tee_,
                   nullptr);
  gst_element_link_many(rawvideo_,
                        video_tee_,
                        nullptr);
  
  std::string caps_str = video_output_format_->get_caps_str();
  GstElement *colorspace = nullptr;
  GstElement *capsfilter = nullptr;
  if (caps_str != "none") {
    GstUtils::make_element("ffmpegcolorspace", &colorspace);
    GstUtils::make_element("capsfilter", &capsfilter);
    GstCaps *usercaps = gst_caps_from_string(caps_str.c_str());
    g_object_set(G_OBJECT(capsfilter),
                 "caps", usercaps,
                 nullptr);
    gst_caps_unref(usercaps);
    gst_bin_add_many(GST_BIN(get_bin()),
                     colorspace,
                     capsfilter,
                     nullptr);
    gst_element_link_many(video_tee_,
                          colorspace,
                          capsfilter,
                          nullptr);
  }

  ShmdataWriter::ptr shmdata_writer = std::make_shared<ShmdataWriter>();
  shmdata_path_ = make_file_name("video");
  shmdata_writer->set_path(shmdata_path_.c_str());
  if (caps_str != "none") {
    GstCaps *caps = gst_caps_new_simple(caps_str.c_str(),
                                        nullptr);
    On_scope_exit{gst_caps_unref(caps);};
    shmdata_writer->plug(get_bin(), capsfilter, caps);
  } else {
    shmdata_writer->plug(get_bin(), video_tee_, nullptr);
  }
  register_shmdata(shmdata_writer);

  GstUtils::sync_state_with_parent(rawvideo_);
  GstUtils::sync_state_with_parent(video_tee_);
  if (caps_str != "none") {
    GstUtils::sync_state_with_parent(colorspace);
    GstUtils::sync_state_with_parent(capsfilter);
  }

  if (codec_ != 0) {
    remake_codec_elements();
    gst_bin_add_many(GST_BIN(get_bin()),
                     queue_codec_element_,
                     codec_element_,
                     color_space_codec_element_,
                     nullptr);

    gst_element_link_many(video_tee_,
                          queue_codec_element_,
                          color_space_codec_element_,
                          codec_element_, nullptr);

    ShmdataWriter::ptr shmdata_codec;
    shmdata_codec.reset(new ShmdataWriter());
    std::string shmdata_path = make_file_name("encoded-video");
    shmdata_codec->set_path(shmdata_path.c_str());

    GstPad *srcpad = gst_element_get_static_pad(codec_element_, "src");
    shmdata_codec->plug(get_bin(), srcpad);
    gst_object_unref(srcpad);
    register_shmdata(shmdata_codec);
    GstUtils::sync_state_with_parent(queue_codec_element_);
    GstUtils::sync_state_with_parent(color_space_codec_element_);
    GstUtils::sync_state_with_parent(codec_element_);
  }
  return true;
}

bool VideoSource::start() {
  rawvideo_ = nullptr;
  if (!make_video_source(&rawvideo_)) {
    g_debug("cannot make video source");
    return false;
  }
  if (!make_new_shmdatas())
    return false;
  if (!on_start())
    return false;
  disable_method("reset");
  disable_property("codec");
  disable_property("more_codecs");
  video_output_format_->disable_property();
  return true;
}

bool VideoSource::stop() {
  bool res = on_stop();
  unregister_shmdata(make_file_name("encoded-video"));
  unregister_shmdata(make_file_name("video"));
  reset_bin();
  enable_method("reset");
  enable_property("codec");
  enable_property("more_codecs");
  video_output_format_->enable_property();
  return res;
}

bool VideoSource::remake_codec_elements() {
  if (codec_ == 0)
    return false;
  for (auto &it : codec_properties_)
    uninstall_property(it);
  GstElement *tmp_codec_element = codec_element_;
  GstElement *tmp_color_space_codec_element = color_space_codec_element_;
  GstElement *tmp_queue_codec_element = queue_codec_element_;
  if (!GstUtils::make_element(secondary_codec_[codec_].value_nick, &codec_element_)
      || !GstUtils::make_element("ffmpegcolorspace", &color_space_codec_element_)
      || !GstUtils::make_element("queue", &queue_codec_element_))
    return false;
  // copy property value and register codec properties
  for (auto &it : codec_properties_) {
    if (nullptr != tmp_codec_element)
      GstUtils::apply_property_value(G_OBJECT(tmp_codec_element),
                                     G_OBJECT(codec_element_),
                                     it.c_str());
    install_property(G_OBJECT(codec_element_), it, it, it);
  }
  GstUtils::clean_element(tmp_codec_element);
  GstUtils::clean_element(tmp_color_space_codec_element);
  GstUtils::clean_element(tmp_queue_codec_element);
  return true;
}

void VideoSource::set_codec(const gint value, void *user_data) {
  VideoSource *context = static_cast<VideoSource *>(user_data);
  context->codec_ = value;
  context->remake_codec_elements();
}

gint VideoSource::get_codec(void *user_data) {
  VideoSource *context = static_cast<VideoSource *>(user_data);
  return context->codec_;
}

// gboolean VideoSource::get_codec_long_list(void *user_data) {
//   VideoSource *context = static_cast<VideoSource *>(user_data);
//   return context->codec_long_list_;
// }

// void
// VideoSource::set_codec_long_list(gboolean codec_long_list,
//                                  void *user_data) {
//   VideoSource *context = static_cast<VideoSource *>(user_data);
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

void VideoSource::make_codec_properties() {
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

gboolean VideoSource::reset_codec_configuration(gpointer /*unused */ , gpointer user_data) {
  VideoSource *context = static_cast<VideoSource *>(user_data);
  context->set_property("format", "0");  // do not apply
  context->set_property("codec","vp8enc");
  context->set_property("quality","5");
  context->set_property("bitrate","0");
  context->set_property("threads","4");
  context->set_property("speed","7");
  context->set_property("mode","0");  // vbr
  context->set_property("error-resilient","true");
  context->set_property("max-latency","10");
  context->set_property("max-keyframe-distance","5");
  return TRUE;
}

}
