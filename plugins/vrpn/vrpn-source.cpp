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

#include "vrpn-source.hpp"
#include <iomanip>
#include <iostream>

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    VRPNSource,
    "vrpnsrc",
    "VRPN Source Client",
    "vrpn",
    "writer/hid/device",
    "Plugin to connect to a VRPN server and share its controls through shmdata and/or properties.",
    "LGPL",
    "François Ubald Brien");

VRPNSource::VRPNSource(const std::string&)
    : host_id_(pmanage<MPtr(&PContainer::make_string)>("host",
                                                       [this](const std::string& val) {
                                                         host_ = val;
                                                         return true;
                                                       },
                                                       [this]() { return host_; },
                                                       "Host",
                                                       "VRPN server hostname or IP address.",
                                                       host_)),
      port_id_(pmanage<MPtr(&PContainer::make_int)>("port",
                                                    [this](int val) {
                                                      port_ = val;
                                                      return true;
                                                    },
                                                    [this]() { return port_; },
                                                    "Port",
                                                    "Port that the VRPN client will connect to.",
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
                                                               debug_)) {}

bool VRPNSource::init() {
  init_startable(this);
  shm_ = std::make_unique<ShmdataWriter>(this, make_file_name("vrpn"), 128, "application/vrpn");
  if (!shm_.get()) {
    g_message("ERROR: VRPN failed to initialize its shmdata");
    shm_.reset(nullptr);
    return false;
  }
  return true;
}

bool VRPNSource::start() {
  if (host_.empty()) {
    g_message("ERROR: Host is required.");
    return false;
  }

  std::string connectionString;
  connectionString += host_;
  connectionString += ":";
  connectionString += std::to_string(port_);
  connection_ = std::make_unique<VRPNClientConnection>(connectionString);
  if (!connection_->raw()->doing_okay()) {
    g_message("ERROR: Connection is not doing okay.");
    return false;
  }

  pmanage<MPtr(&PContainer::disable)>(host_id_, StartableQuiddity::disabledWhenStartedMsg);

  connection_->raw()->register_handler(vrpn_ANY_TYPE, handle_message, this, vrpn_ANY_SENDER);
  loopTask_ =
      std::make_unique<PeriodicTask>([this]() { this->loop(); }, VRPNSource::vrpnLoopInterval);

  g_debug("Started VRPN connection");

  return true;
}

bool VRPNSource::stop() {
  g_debug("Stopping VRPN connection");

  loopTask_.reset(nullptr);
  connection_->raw()->unregister_handler(vrpn_ANY_TYPE, handle_message, this, vrpn_ANY_SENDER);
  connection_.reset(nullptr);
  pmanage<MPtr(&PContainer::enable)>(host_id_);

  return true;
}

void VRPNSource::loop() {
  if (!connection_->raw()->doing_okay()) {
    g_warning("Connection is not doing okay.");
  }

  if (!connection_->raw()->connected()) {
    g_warning("Disconnected.");
  }

  connection_->raw()->mainloop();
}

int VRPNSource::handle_message(void* userData, vrpn_HANDLERPARAM p) {
  auto* context = static_cast<VRPNSource*>(userData);

  std::string senderName = context->connection_->raw()->sender_name(p.sender);
  std::string typeName = context->connection_->raw()->message_type_name(p.type);

  std::vector<unsigned char> buffer((size_t)(
      sizeof(uint32_t) + senderName.length() + sizeof(uint32_t) + typeName.length() +
      sizeof(p.msg_time.tv_sec) + sizeof(p.msg_time.tv_usec) + sizeof(uint32_t) + p.payload_len));

  // BUFFER
  auto buffer_ptr = buffer.data();

  // SENDER
  *((uint32_t*)buffer_ptr) = htonl(senderName.length());
  buffer_ptr += sizeof(uint32_t);
  memcpy(buffer_ptr, senderName.data(), senderName.length());
  buffer_ptr += senderName.length();

  // TYPE
  *((uint32_t*)buffer_ptr) = htonl(typeName.length());
  buffer_ptr += sizeof(uint32_t);
  memcpy(buffer_ptr, typeName.data(), typeName.length());
  buffer_ptr += typeName.length();

  // TIME
  *((decltype(p.msg_time.tv_sec)*)buffer_ptr) = htonl(p.msg_time.tv_sec);
  buffer_ptr += sizeof(p.msg_time.tv_sec);
  *((decltype(p.msg_time.tv_usec)*)buffer_ptr) = htonl(p.msg_time.tv_usec);
  buffer_ptr += sizeof(p.msg_time.tv_usec);

  // PAYLOAD
  *((decltype(p.payload_len)*)buffer_ptr) = htonl(p.payload_len);
  buffer_ptr += sizeof(p.payload_len);
  memcpy(buffer_ptr, p.buffer, p.payload_len);

  // DEBUG
  if (context->debug_) {
    g_debug("VRPNSource >>> Sender: %s Type: %s Length: %lu Payload: %d",
            senderName.c_str(),
            typeName.c_str(),
            buffer.size(),
            p.payload_len);

    std::stringstream ss;
    ss << "VRPNSource >>> ";
    for (auto const& value : buffer) {
      ss << std::hex << std::setfill('0') << std::setw(2) << (int)value << " ";
    }
    g_debug("%s", ss.str().c_str());
  }

  // WRITE TO SHMDATA
  context->shm_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(buffer.data(), buffer.size());
  context->shm_->bytes_written(buffer.size());

  return 0;
}

}  // namespace switcher
