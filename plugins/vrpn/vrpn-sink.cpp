/*
 * This file is part of switcher-vrpn.
 *
 * switcher-vrpn is free software; you can redistribute it and/or
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

#include "vrpn-sink.hpp"
#include <iomanip>
#include <iostream>

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(VRPNSink,
                                     "vrpnsink",
                                     "VRPN Sink Server",
                                     "vrpn",
                                     "reader/device",
                                     "Plugin to create a local VRPN server from shmdata sources.",
                                     "LGPL",
                                     "François Ubald Brien");

VRPNSink::VRPNSink(const std::string&)
    : port_id_(pmanage<MPtr(&PContainer::make_int)>("port",
                                                    [this](int val) {
                                                      port_ = val;
                                                      return true;
                                                    },
                                                    [this]() { return port_; },
                                                    "Port",
                                                    "Port that the VRPN server will listen to.",
                                                    port_,
                                                    1,
                                                    65536)),
      advanced_id_(pmanage<MPtr(&PContainer::make_group)>(
          "advanced", "Advanced configuration", "Advanced configuration")),
      debug_id_(pmanage<MPtr(&PContainer::make_parented_bool)>("debug",
                                                               "advanced",
                                                               [this](bool val) {
                                                                 debug_ = val;
                                                                 return true;
                                                               },
                                                               [this]() { return debug_; },
                                                               "Debug",
                                                               "Debug values to console",
                                                               debug_)),
      shmcntr_(static_cast<Quiddity*>(this)) {}

bool VRPNSink::init() {
  init_startable(this);

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->connect(shmpath); },
      [this](const std::string&) { return this->disconnect(); },
      [this]() { return this->disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);

  return true;
}

void VRPNSink::on_shmreader_data(void* data, size_t size) {
  if (!is_started()) {
    return;
  }

  // BUFFER
  char* data_ptr = static_cast<char*>(data);

  // SENDER
  uint32_t senderNameLength = ntohl(*((uint32_t*)data_ptr));
  data_ptr += sizeof(uint32_t);
  std::string senderName(data_ptr, senderNameLength);
  data_ptr += senderNameLength;

  // TYPE
  uint32_t typeNameLength = ntohl(*((uint32_t*)data_ptr));
  data_ptr += sizeof(uint32_t);
  std::string typeName(data_ptr, typeNameLength);
  data_ptr += typeNameLength;

  // TIME
  timeval time;
  time.tv_sec = ntohl(*((decltype(time.tv_sec)*)data_ptr));
  data_ptr += sizeof(time.tv_sec);
  time.tv_usec = ntohl(*((decltype(time.tv_usec)*)data_ptr));
  data_ptr += sizeof(time.tv_usec);

  // PACK MESSAGE
  uint32_t payload_len = ntohl(*((uint32_t*)data_ptr));
  data_ptr += sizeof(uint32_t);
  connection_->raw()->pack_message(payload_len,
                                   time,
                                   connection_->raw()->register_message_type(typeName.c_str()),
                                   connection_->raw()->register_sender(senderName.c_str()),
                                   static_cast<char*>(data_ptr),
                                   vrpn_CONNECTION_RELIABLE);

  if (debug_) {
    g_debug("VRPNSink   <<< Sender: %s Type: %s Length: %lu Payload: %u",
            senderName.c_str(),
            typeName.c_str(),
            size,
            payload_len);

    unsigned char* d = static_cast<unsigned char*>(data);
    std::stringstream ss;
    ss << "VRPNSink   <<< ";
    for (int i = 0; i < (int)size; ++i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << (int)d[i] << " ";
    }
    g_debug("%s", ss.str().c_str());
  }
}

bool VRPNSink::start() {
  connection_ = std::make_unique<VRPNServerConnection>(port_);
  if (!connection_->raw()->doing_okay()) {
    g_message("ERROR: Connection is not doing okay.");
    return false;
  }

  pmanage<MPtr(&PContainer::disable)>(port_id_, disabledWhenStartedMsg);

  loopTask_ = std::make_unique<PeriodicTask>([this]() { this->loop(); },
                                             std::chrono::milliseconds(vrpnLoopInterval));

  g_debug("Started VRPN server");

  return true;
}

bool VRPNSink::stop() {
  g_debug("Stopping VRPN server");

  loopTask_.reset(nullptr);
  connection_.reset(nullptr);
  pmanage<MPtr(&PContainer::enable)>(port_id_);

  return true;
}

bool VRPNSink::connect(std::string path) {
  shm_.reset(new ShmdataFollower(
      this, path, [this](void* data, size_t size) { this->on_shmreader_data(data, size); }));
  return true;
}

bool VRPNSink::disconnect() {
  shm_.reset(nullptr);
  return true;
}

bool VRPNSink::can_sink_caps(std::string caps) { return (0 == caps.find("application/vrpn")); }

void VRPNSink::loop() {
  if (!connection_->raw()->doing_okay()) {
    g_warning("Connection is not doing okay.");
  }
  connection_->raw()->mainloop();
}

}  // namespace switcher
