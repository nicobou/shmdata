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

#include "./shmdata-to-osc.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    ShmdataToOsc,
    "OSCsink",
    "OSC sender",
    "network",
    "reader",
    "shmOSCsink reveives OSC messages and updates associated property",
    "LGPL",
    "Nicolas Bouillot");

ShmdataToOsc::ShmdataToOsc(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)), shmcntr_(static_cast<Quiddity*>(this)) {
  init_startable(this);
  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->connect(shmpath); },
      [this](const std::string&) { return this->disconnect(); },
      [this]() { return this->disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);
  pmanage<MPtr(&PContainer::make_int)>("port",
                                       [this](const int& val) {
                                         if (val == port_) return true;
                                         stop();
                                         port_ = val;
                                         return start();
                                       },
                                       [this]() { return port_; },
                                       "Port",
                                       "OSC destination",
                                       port_,
                                       1,
                                       65536);
  pmanage<MPtr(&PContainer::make_string)>("host",
                                          [this](const std::string& val) {
                                            if (val == host_) return true;
                                            stop();
                                            host_ = val;
                                            return start();
                                          },
                                          [this]() { return host_; },
                                          "Destination host",
                                          "Destination host",
                                          host_);
}

ShmdataToOsc::~ShmdataToOsc() { stop(); }

bool ShmdataToOsc::start() {
  stop();
  {
    std::unique_lock<std::mutex> lock(address_mutex_);
    address_ = lo_address_new(host_.c_str(), std::to_string(port_).c_str());
  }
  if (nullptr == address_) return false;
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

bool ShmdataToOsc::connect(const std::string& path) {
  shm_ = std::make_unique<ShmdataFollower>(
      this,
      path,
      [this](void* data, size_t size) { this->on_shmreader_data(data, size); },
      nullptr,
      nullptr,
      ShmdataStat::kDefaultUpdateInterval);
  return true;
}

bool ShmdataToOsc::disconnect() {
  shm_.reset(nullptr);
  return true;
}

void ShmdataToOsc::on_shmreader_data(void* data, size_t data_size) {
  const char* path = lo_get_path(data, data_size);
  lo_message msg = lo_message_deserialise(data, data_size, nullptr);  // error code
  if (nullptr != msg) {
    std::unique_lock<std::mutex> lock(address_mutex_);
    // lo_message_pp (msg);
    if (nullptr != address_) lo_send_message(address_, path, msg);
    lo_message_free(msg);
  }
}

bool ShmdataToOsc::can_sink_caps(const std::string& caps) {
  return (0 == caps.find("application/x-libloserialized-osc"));
}

}  // namespace switcher
