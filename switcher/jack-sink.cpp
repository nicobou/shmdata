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

#include "./jack-sink.hpp"
#include "./gst-utils.hpp"
#include "./quiddity-command.hpp"
#ifdef HAVE_CONFIG_H
#include "../config.h"
#endif

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(JackSink,
                                     "Audio Display (Jack)",
                                     "audio",
                                     "Audio display with minimal features",
                                     "LGPL",
                                     "jacksink",
                                     "Nicolas Bouillot");

bool JackSink::init_gpipe() {
  client_name_ = get_name();
  if (false == make_elements())
    return false;
  init_startable(this);
  client_name_spec_ =
      custom_props_->make_string_property("jack-client-name",
                                          "the jack client name",
                                          "switcher",
                                          (GParamFlags) G_PARAM_READWRITE,
                                          JackSink::set_client_name,
                                          JackSink::get_client_name, this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            client_name_spec_,
                            "client-name",
                            "Client Name");
  install_property(G_OBJECT(volume_), "volume", "volume", "Volume");
  install_property(G_OBJECT(volume_), "mute", "mute", "Mute");
  return true;
}

JackSink::JackSink(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper>()) {
}

JackSink::~JackSink() {
  GstUtils::clean_element(jacksink_);
}

bool JackSink::make_elements() {
  GError *error = nullptr;
  std::string description (std::string("audioconvert ! audioresample ! volume ! ")
                           + "audioconvert ! "
                           + "jackaudiosink provide-clock=false slave-method=resample "
                           + "client-name=" + client_name_ + " sync=false");
  GstElement *jacksink = gst_parse_bin_from_description(description.c_str(), TRUE, &error);
  if (error != nullptr) {
    g_warning("%s", error->message);
    g_error_free(error);
    return false;
  }
  g_object_set(G_OBJECT(jacksink), "async-handling", TRUE, nullptr);
  GstElement *volume = GstUtils::get_first_element_from_factory_name(GST_BIN(jacksink),
                                                                     "volume");
  if (nullptr != volume_) {
    GstUtils::apply_property_value(G_OBJECT(volume_),
                                   G_OBJECT(volume),
                                   "volume");
    GstUtils::apply_property_value(G_OBJECT(volume_),
                                   G_OBJECT(volume),
                                   "mute");
  }

  if (nullptr != jacksink_) 
    GstUtils::clean_element(jacksink_);
  jacksink_ = jacksink;
  volume_ = volume;
  return true;
}

bool JackSink::start() {
  if (false == make_elements())
    return false;
  set_sink_element(jacksink_);
  reinstall_property(G_OBJECT(volume_),
                     "volume", "volume", "Volume");
  reinstall_property(G_OBJECT(volume_),
                     "mute", "mute", "Mute");
  return true;
}

bool JackSink::stop() {
  if (!make_elements())
    return false;
  reset_bin();
  reinstall_property(G_OBJECT(volume_),
                     "volume", "volume", "Volume");
  reinstall_property(G_OBJECT(volume_),
                     "mute", "mute", "Mute");
  return true;
}

void JackSink::set_client_name(const gchar *value, void *user_data) {
  JackSink *context = static_cast<JackSink *>(user_data);
  context->client_name_ = std::string(value);
  context->custom_props_->
      notify_property_changed(context->client_name_spec_);
}

const gchar *JackSink::get_client_name(void *user_data) {
  JackSink *context = static_cast<JackSink *>(user_data);
  return context->client_name_.c_str();
}

void JackSink::on_shmdata_disconnect() {
  stop();
}

void JackSink::on_shmdata_connect(std::string /* shmdata_sochet_path */ ) {
  if (is_started()) {
    stop();
    set_sink_element_no_connect(jacksink_);
  }
}

bool JackSink::can_sink_caps(std::string caps) {
  return GstUtils::can_sink_caps("audioconvert", caps);
}
}
