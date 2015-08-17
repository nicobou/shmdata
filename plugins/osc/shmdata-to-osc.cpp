/*
 * This file is part of switcher-osc.
 *
 * switcher-osc is free software; you can redistribute it and/or
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

#include "switcher/json-builder.hpp"
#include "./shmdata-to-osc.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    ShmdataToOsc,
    "shmOSCsink",
    "OSC sender",
    "network",
    "reader",
    "shmOSCsink reveives OSC messages and updates associated property",
    "LGPL",
    "Nicolas Bouillot");

ShmdataToOsc::ShmdataToOsc(const std::string &):
    shmcntr_(static_cast<Quiddity *>(this)),
    custom_props_(new CustomPropertyHelper()) {
}

bool ShmdataToOsc::init() {
  init_startable(this);
  shmcntr_.install_connect_method(
      [this](const std::string &shmpath){return this->connect(shmpath);},
      [this](const std::string &){return this->disconnect();},
      [this](){return this->disconnect();},
      [this](const std::string &caps){return this->can_sink_caps(caps);},
      1);
  port_spec_ =
      custom_props_->make_int_property("Port",
                                       "OSC destination port",
                                       1,
                                       65536,
                                       port_,
                                       (GParamFlags) G_PARAM_READWRITE,
                                       set_port, get_port, this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            port_spec_, "port", "Port");
  host_spec_ =
      custom_props_->make_string_property("host",
                                          "destination host",
                                          host_.c_str(),
                                          (GParamFlags) G_PARAM_READWRITE,
                                          ShmdataToOsc::set_host,
                                          ShmdataToOsc::get_host, this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            host_spec_, "host", "Destination Host");
  return true;
}

ShmdataToOsc::~ShmdataToOsc() {
  stop();
}

void ShmdataToOsc::set_port(const gint value, void *user_data) {
  ShmdataToOsc *context = static_cast<ShmdataToOsc *>(user_data);
  if (value == context->port_)
    return;
  context->stop();
  context->port_ = value;
  context->start();
  context->custom_props_->notify_property_changed(context->port_spec_);
}

gint ShmdataToOsc::get_port(void *user_data) {
  ShmdataToOsc *context = static_cast<ShmdataToOsc *>(user_data);
  return context->port_;
}

bool ShmdataToOsc::start() {
  stop();
  {
    std::unique_lock<std::mutex> lock(address_mutex_);
    address_ = lo_address_new(host_.c_str(), std::to_string(port_).c_str());
  }
  if (nullptr == address_)
    return false;
  return true;
}

bool ShmdataToOsc::stop() {
  if (nullptr != address_) {
    std::unique_lock<std::mutex> lock(address_mutex_);
    lo_address_free(address_);
    address_ = nullptr;
  }
  return true;
}

void ShmdataToOsc::set_host(const gchar *value, void *user_data) {
  ShmdataToOsc *context = static_cast<ShmdataToOsc *>(user_data);
  if (0 == context->host_.compare(value))
    return;
  context->stop();
  context->host_ = value;
  context->start();
  context->custom_props_->notify_property_changed(context->host_spec_);
}

const gchar *ShmdataToOsc::get_host(void *user_data) {
  ShmdataToOsc *context = static_cast<ShmdataToOsc *>(user_data);
  return context->host_.c_str();
}

bool ShmdataToOsc::connect(const std::string &path) {
  shm_.reset(new ShmdataFollower(this,
                                 path,
                                 [this](void *data, size_t size){
                                   this->on_shmreader_data(data, size);
                                 }));
  return true;
}

bool ShmdataToOsc::disconnect() {
  shm_.reset(nullptr);
  return true;
}

void
ShmdataToOsc::on_shmreader_data(void *data,
                                size_t data_size) {
  const char *path = lo_get_path(data, data_size);
  lo_message msg = lo_message_deserialise(data,
                                          data_size,
                                          nullptr);   // error code
  if (nullptr != msg) {
    std::unique_lock<std::mutex> lock(address_mutex_);
    // lo_message_pp (msg);
    if (nullptr != address_)
      lo_send_message(address_, path, msg);
    lo_message_free(msg);
  }
}

bool ShmdataToOsc::can_sink_caps(const std::string &caps) {
  return (0 == caps.find("application/x-libloserialized-osc"));
}

}  // namespace switcher
