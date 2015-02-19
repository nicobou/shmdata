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

#include "./decodebin2.hpp"
#include "./gst-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Decodebin2,
                                     "Shmdata Decoder",
                                     "decoder",
                                     "connect to a shmdata, decode it and write decoded frames to shmdata(s)",
                                     "LGPL",
                                     "decodebin", "Nicolas Bouillot");

Decodebin2::Decodebin2(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper>()),
    decodebin_(new DecodebinToShmdata(static_cast<GstPipeliner *>(this))),
    media_counters_() {
}

bool Decodebin2::init_gpipe() {
    media_label_spec_ =
      custom_props_->make_string_property("media-label",
                                          "the media label",
                                          "",
                                          (GParamFlags)G_PARAM_READWRITE,
                                          Decodebin2::set_media_label,
                                          Decodebin2::get_media_label,
                                          this);
    install_property_by_pspec(custom_props_->get_gobject(),
                              media_label_spec_,
                              "media-label",
                              "Media Label");
    
    decodebin_->invoke(std::bind(&SinglePadGstSink::set_sink_element,
                                 this,
                                 std::placeholders::_1));
    SinglePadGstSink::set_on_first_data_hook(Decodebin2::make_decodebin_active, this);
    return true;
}

void
Decodebin2::make_decodebin_active(ShmdataReader *caller,
                                  void *user_data) {
  Decodebin2 *context = static_cast<Decodebin2 *>(user_data);
  if (!context->media_label_.empty())
    context->decodebin_->set_media_label(std::string(context->media_label_));
  context->decodebin_->
      invoke_with_return<gboolean>(std::bind(gst_bin_add,
                                             GST_BIN(context->get_bin()),
                                             std::placeholders::_1));
  context->decodebin_->
      invoke(std::bind(GstUtils::sync_state_with_parent,
                       std::placeholders::_1));
}

void Decodebin2::set_media_label(const gchar *value, void *user_data) {
  Decodebin2 *context = static_cast<Decodebin2 *>(user_data);
  context->media_label_ = std::string(value);
  context->custom_props_->
      notify_property_changed(context->media_label_spec_);
}

const gchar *Decodebin2::get_media_label(void *user_data) {
  Decodebin2 *context = static_cast<Decodebin2 *>(user_data);
  return context->media_label_.c_str();
}

}  // namespace switcher
