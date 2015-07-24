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

#include "switcher/shmdata-utils.hpp"
#include "switcher/std2.hpp"
#include "switcher/scope-exit.hpp"
#include "./gst-pixel-format-converter.hpp"

namespace switcher {
GstPixelFormatConverter::GstPixelFormatConverter(
    Quiddity *quid,
    CustomPropertyHelper *prop_helper,
    const char *property_name,
    const char *display_text):
    quid_(quid),
    gst_pipeline_(std2::make_unique<GstPipeliner>(nullptr, nullptr)),
    custom_props_(prop_helper),
    prop_name_(property_name) {
  make_format_property(property_name, display_text);
}

void GstPixelFormatConverter::make_format_property(const char *name,
                                                   const char *display_text) {
  GstElementFactory *factory = gst_element_factory_find("videoconvert"); 
  On_scope_exit{gst_object_unref(factory);};
  const GList *list = gst_element_factory_get_static_pad_templates(factory);  
  guint i = 0;  
  while (nullptr != list) {  
    GstStaticPadTemplate *templ = reinterpret_cast<GstStaticPadTemplate *>(list->data);  
    if (templ->direction == GST_PAD_SRC) {
      GstCaps *caps = gst_static_pad_template_get_caps(templ);
      // copying for removing fields in struture  
      GstStructure *structure = gst_caps_get_structure(caps, 0);
      const GValue *format_list = gst_structure_get_value (structure, "format");
      if (format_list)
        for (guint j = 0; j < gst_value_list_get_size(format_list); ++j){
          formats_.emplace_back(
              g_value_get_string(gst_value_list_get_value(format_list, j)));
          video_format_[i].value = i;
          video_format_[i].value_name = formats_.back().c_str();
          video_format_[i].value_nick = formats_.back().c_str();
          ++i;
        }
      gst_caps_unref(caps);
    }  // templ->direction == GST_PAD_SRC
    list = g_list_next(list);  
  }  
  // writing end of video_format_
  video_format_[i].value = 0;
  video_format_[i].value_name = nullptr;
  video_format_[i].value_nick = nullptr;
  video_format_spec_ =
      custom_props_->make_enum_property(name,
                                        display_text,
                                        format_,
                                        video_format_,
                                        (GParamFlags) G_PARAM_READWRITE,
                                        GstPixelFormatConverter::set_format,
                                        GstPixelFormatConverter::get_format,
                                        this);
  quid_->install_property_by_pspec(custom_props_->get_gobject(),
                                   video_format_spec_,
                                   name,
                                   display_text);
}

void GstPixelFormatConverter::set_format(const gint value, void *user_data) {
  GstPixelFormatConverter *context = static_cast<GstPixelFormatConverter *>(user_data);
  context->format_ = value;
}

gint GstPixelFormatConverter::get_format(void *user_data) {
  GstPixelFormatConverter *context = static_cast<GstPixelFormatConverter *>(user_data);
  return context->format_;
}

std::string GstPixelFormatConverter::get_caps_str() const{
  return std::string("video/x-raw, format=(string)") + video_format_[format_].value_nick;  
}

bool GstPixelFormatConverter::disable_property() {
  return quid_->disable_property(prop_name_);
}

bool GstPixelFormatConverter::enable_property() {
  return quid_->enable_property(prop_name_);
}

bool GstPixelFormatConverter::start(const std::string &shmpath_to_convert,
                                    const std::string &shmpath_converted){
  if(shmpath_converted.empty() || shmpath_to_convert.empty()){
    g_warning("GstPixelFormatConverter::start needs non empty paths");
    return false;
  }
  if(shmpath_to_convert == shmpath_converted_){
    g_warning("GstPixelFormatConverter::start cannot convert itself");
    return false;
  }
  shmpath_converted_ = shmpath_converted;
  shmpath_to_convert_ = shmpath_to_convert;
  disable_property();
  GstCaps *caps = gst_caps_from_string (get_caps_str().c_str());
  g_object_set(G_OBJECT(capsfilter_.get_raw()),
               "caps", caps,
               nullptr);
  gst_caps_unref(caps);
  g_object_set(G_OBJECT(shmsrc_.get_raw()),
               "socket-path", shmpath_to_convert.c_str(), nullptr);
  g_object_set(G_OBJECT(shm_converted_.get_raw()),
               "socket-path", shmpath_converted.c_str(),
               "sync", false,
               nullptr);
  shmsink_sub_ = std2::make_unique<GstShmdataSubscriber>(
      shm_converted_.get_raw(),
      [this]( const std::string &caps){
        this->quid_->graft_tree(".shmdata.writer." + shmpath_converted_,
                                ShmdataUtils::make_tree(caps,
                                                        ShmdataUtils::get_category(caps),
                                                        0));
      },
      [this](GstShmdataSubscriber::num_bytes_t byte_rate){
        this->quid_->graft_tree(".shmdata.writer." + shmpath_converted_ + ".byte_rate",
                                data::Tree::make(std::to_string(byte_rate)));
      });
  shmsrc_sub_ = std2::make_unique<GstShmdataSubscriber>(
      shmsrc_.get_raw(),
      [this]( const std::string &caps){
        this->quid_->graft_tree(".shmdata.reader." + shmpath_to_convert_,
                                ShmdataUtils::make_tree(caps,
                                                        ShmdataUtils::get_category(caps),
                                                        0));
      },
      [this](GstShmdataSubscriber::num_bytes_t byte_rate){
        this->quid_->graft_tree(".shmdata.reader." + shmpath_to_convert_ + ".byte_rate",
                                data::Tree::make(std::to_string(byte_rate)));
      });
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   shmsrc_.get_raw(),
                   queue_codec_element_.get_raw(),
                   color_space_codec_element_.get_raw(),
                   capsfilter_.get_raw(),
                   shm_converted_.get_raw(),
                   nullptr);
  gst_element_link_many(shmsrc_.get_raw(),
                        queue_codec_element_.get_raw(),
                        color_space_codec_element_.get_raw(),
                        capsfilter_.get_raw(),
                        shm_converted_.get_raw(),
                        nullptr);
  gst_pipeline_->play(true);
  return true;
}

bool GstPixelFormatConverter::stop(){
  enable_property();
  shmsink_sub_.reset();
  shmsrc_sub_.reset();
  quid_->prune_tree(".shmdata.writer." + shmpath_converted_);
  quid_->prune_tree(".shmdata.reader." + shmpath_to_convert_);
  if (!UGstElem::renew(shmsrc_, {"socket-path"})
      || !UGstElem::renew(shm_converted_, {"socket-path", "sync"})
      || !UGstElem::renew(color_space_codec_element_)
      || !UGstElem::renew(capsfilter_)
      || !UGstElem::renew(queue_codec_element_)){
    g_warning("error renewing a pixel format converter related gst element");
    return false;
  }
  gst_pipeline_ = std2::make_unique<GstPipeliner>(nullptr, nullptr);
  return true;
}

}  // namespace switcher
