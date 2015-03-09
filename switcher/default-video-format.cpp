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

#include "./default-video-format.hpp"

namespace switcher {
DefaultVideoFormat::DefaultVideoFormat(Quiddity *quid):
    quid_(quid),
    custom_props_(std::make_shared<CustomPropertyHelper>()) {
}

void DefaultVideoFormat::make_format_property(const char *name,
                                              const char *display_text) {
  prop_name_ = name;
  GstElementFactory *factory = gst_element_factory_find("ffmpegcolorspace"); 
  const GList *list = gst_element_factory_get_static_pad_templates(factory);  
  // first option is do not format
  caps_.emplace_back("none");
  formats_.emplace_back("Do not apply");
  video_format_[0].value = 0;
  video_format_[0].value_name = formats_.back().c_str();
  video_format_[0].value_nick = caps_.back().c_str();
  guint i = 0;  
  while (NULL != list) {  
    GstStaticPadTemplate *templ = (GstStaticPadTemplate *)list->data;  
    if (templ->direction == GST_PAD_SRC) {  
      GstCaps *caps = gst_static_caps_get(&templ->static_caps);  
      // copying for removing fields in struture  
      GstCaps *copy = gst_caps_copy(caps);  
      gst_caps_unref(caps);  
      guint size = gst_caps_get_size(copy);  
      i = 0;
      for (; i < size; i++) {  
        GstStructure *structure = gst_caps_get_structure(copy, i );  
        gst_structure_remove_fields(structure,  
                                    "format", "width", "height", "framerate",  
                                    NULL);  
        // removing endianness for rgb
        if (std::string(gst_structure_get_name(structure)) == "video/x-raw-rgb") 
          gst_structure_remove_field(structure, "endianness");  

        // copying the caps
        GstCaps *copy_nth = gst_caps_copy_nth(copy, i);   
        gchar *caps_str = gst_caps_to_string(copy_nth);
        caps_.emplace_back(caps_str);
        // reducing the string for menu display
        std::string format(std::string(caps_str), 12);  // removing video/x-raw-
        std::string int_str("(int)");
        size_t found = format.find(int_str);
        while(std::string::npos != found) {
          format.replace(found, 5, "");
          found = format.find(int_str, found);
        }
        formats_.emplace_back(std::move(format));
        //
        video_format_[i+1].value = i+1;
        video_format_[i+1].value_name = formats_.back().c_str();
        video_format_[i+1].value_nick = caps_.back().c_str();
        g_free(caps_str);   
        gst_caps_unref(copy_nth);   
      }  
      gst_caps_unref(copy);
    }  // templ->direction == GST_PAD_SRC
    list = g_list_next(list);  
  }  
  gst_object_unref(factory);
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
                                        DefaultVideoFormat::set_format,
                                        DefaultVideoFormat::get_format,
                                        this);
  quid_->install_property_by_pspec(custom_props_->get_gobject(),
                                   video_format_spec_,
                                   name,
                                   display_text);
}

void DefaultVideoFormat::set_format(const gint value, void *user_data) {
  DefaultVideoFormat *context = static_cast<DefaultVideoFormat *>(user_data);
  context->format_ = value;
}

gint DefaultVideoFormat::get_format(void *user_data) {
  DefaultVideoFormat *context = static_cast<DefaultVideoFormat *>(user_data);
  return context->format_;
}

std::string DefaultVideoFormat::get_caps_str() const{
  return caps_[format_];  
}

bool DefaultVideoFormat::disable_property() {
  return quid_->disable_property(prop_name_);
}

bool DefaultVideoFormat::enable_property() {
  return quid_->enable_property(prop_name_);
}
 
}  // namespace switcher
