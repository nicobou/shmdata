/*
 * This file is part of switcher-jack.
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

#include <gst/gst.h>
#include "./jack-audio-source.hpp"
#include "./gst-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(JackToShmdata,
                                     "Jack Audio Device2",
                                     "audio",
                                     "get audio from jack",
                                     "LGPL",
                                     "jacksrc2",
                                     "Nicolas Bouillot");

JackToShmdata::JackToShmdata(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper>()) {
}

bool JackToShmdata::init_gpipe() {
  if (!jack_client_) {
    g_warning("JackClient cannot be instancied");
    return false;
  }
  init_startable(this);
  num_channels_spec_ =
      custom_props_->make_int_property("channels",
                                       "number of channels",
                                       1,
                                       256,
                                       num_channels_,
                                       (GParamFlags) G_PARAM_READWRITE,
                                       JackToShmdata::set_num_channels,
                                       JackToShmdata::get_num_channels,
                                       this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            num_channels_spec_, "channels", "Channels");
  connect_physical_port_prop_ =
      custom_props_->make_boolean_property("connect",
                                           "automatically connect to physical ports",
                                           (gboolean) TRUE,
                                           (GParamFlags)G_PARAM_READWRITE,
                                           JackToShmdata::set_connect_phys,
                                           JackToShmdata::get_connect_phys,
                                           this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            connect_physical_port_prop_,
                            "connect",
                            "automatically connect to physical ports");
  client_name_ = g_strdup(get_name().c_str());  // FIXME use std::string
  client_name_spec_ =
      custom_props_->make_string_property("jack-client-name",
                                          "the jack client name",
                                          "switcher",
                                          (GParamFlags) G_PARAM_READWRITE,
                                          JackToShmdata::set_client_name,
                                          JackToShmdata::get_client_name,
                                          this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            client_name_spec_,
                            "client-name", "Client Name");
  return true;
}

JackToShmdata::~JackToShmdata() {
  if (nullptr != client_name_)
    g_free(client_name_);
}

bool JackToShmdata::start() {
  disable_property("channels");
  disable_property("client-name");
  disable_property("connect");
  return true;
}

bool JackToShmdata::stop() {
  enable_property("channels");
  enable_property("client-name");
  enable_property("connect");
  return true;
}

void JackToShmdata::set_num_channels(const gint value, void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  context->num_channels_ = value;
  GObjectWrapper::notify_property_changed(context->gobject_->get_gobject(),
                                          context->num_channels_spec_);
}

gint JackToShmdata::get_num_channels(void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  return context->num_channels_;
}

void JackToShmdata::set_client_name(const gchar *value, void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  if (nullptr != context->client_name_)
    g_free(context->client_name_);
  context->client_name_ = g_strdup(value);
  context->custom_props_->
      notify_property_changed(context->client_name_spec_);
}

const gchar *JackToShmdata::get_client_name(void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  return context->client_name_;
}

gboolean JackToShmdata::get_connect_phys(void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  if (!context->connect_phys_)
    return FALSE;
  return TRUE;
}

void JackToShmdata::set_connect_phys(gboolean connect, void *user_data) {
  JackToShmdata *context = static_cast<JackToShmdata *>(user_data);
  if (connect)
    context->connect_phys_ = true;
  else
    context->connect_phys_ = false;
  context->custom_props_->notify_property_changed(context->connect_physical_port_prop_);
}

}  // namespace swittcher
