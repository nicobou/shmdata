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
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataToOsc,
                                     "OSCsink",
                                     "OSC sender",
                                     "Receives OSC messages and updates associated property",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string ShmdataToOsc::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "osc",
      "description": "OSC stream",
      "can_do": ["application/x-libloserialized-osc"]
    }
  ]
}
)");

ShmdataToOsc::ShmdataToOsc(quiddity::Config&& conf)
    : Quiddity(
          std::forward<quiddity::Config>(conf),
          {kConnectionSpec,
           [this](const std::string& shmpath, claw::sfid_t) { return on_shmdata_connect(shmpath); },
           [this](claw::sfid_t) { return on_shmdata_disconnect(); }}),
      Startable(this),
      port_id_(pmanage<MPtr(&property::PBag::make_int)>(
          "port",
          [this](const int& val) {
            if (port_ == val) return true;
            port_ = val;
            if (!address_) return true;
            stop();
            start();
            return true;
          },
          [this]() { return port_; },
          "Port",
          "OSC destination port",
          port_,
          1,
          65536)),
      host_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "host",
          [this](const std::string& val) {
            if (host_ == val) return true;
            host_ = val;
            if (!address_) return true;
            stop();
            start();
            return true;
          },
          [this]() { return host_; },
          "Destination Host",
          "OSC destination host",
          host_)),
      autostart_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "autostart",
          [this](bool val) {
            autostart_ = val;
            return true;
          },
          [this]() { return autostart_; },
          "Autostart",
          "Start processing on shmdata connect or not",
          autostart_)) {}

ShmdataToOsc::~ShmdataToOsc() { stop(); }

bool ShmdataToOsc::start() {
  if (address_) {
    LOGGER_WARN(this->logger, "OSCsink already started");
    return true;
  }
  if (host_.empty()) {
    LOGGER_ERROR(this->logger, "host must not be empty");
    return false;
  }

  {
    std::unique_lock<std::mutex> lock(address_mutex_);
    address_ = lo_address_new(host_.c_str(), std::to_string(port_).c_str());
  }

  if (!address_) {
    LOGGER_ERROR(this->logger, "could not start OSCsink");
    return false;
  }

  return true;
}

bool ShmdataToOsc::stop() {
  if (address_) {
    std::unique_lock<std::mutex> lock(address_mutex_);
    lo_address_free(address_);
    address_ = nullptr;
  }
  return true;
}

bool ShmdataToOsc::on_shmdata_connect(const std::string& path) {
  shm_ = std::make_unique<shmdata::Follower>(
      this,
      path,
      [this](void* data, size_t size) { this->on_shmreader_data(data, size); },
      nullptr,
      nullptr,
      shmdata::Stat::kDefaultUpdateInterval,
      shmdata::Follower::Direction::reader,
      true);
  if (autostart_ && !address_) {
    return pmanage<MPtr(&property::PBag::set_str_str)>("started", "true");
  }
  return true;
}

bool ShmdataToOsc::on_shmdata_disconnect() {
  shm_.reset();
  if (autostart_) {
    return pmanage<MPtr(&property::PBag::set_str_str)>("started", "false");
  }
  return true;
}

void ShmdataToOsc::on_shmreader_data(void* data, size_t data_size) {
  if (address_) {
    const char* path = lo_get_path(data, data_size);
    lo_message msg = lo_message_deserialise(data, data_size, nullptr);  // error code
    if (msg) {
      std::unique_lock<std::mutex> lock(address_mutex_);
      // lo_message_pp (msg);
      lo_send_message(address_, path, msg);
      lo_message_free(msg);
    }
  }
}

}  // namespace quiddities
}  // namespace switcher
