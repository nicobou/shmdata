/*
 * This file is part of switcher-protocol-mapper.
 *
 * switcher-curl is free software; you can redistribute it and/or
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

#include "protocol-osc.hpp"
#include <lo/lo.h>

namespace switcher {

ProtocolOsc::ProtocolOsc(Quiddity* quid, const InfoTree* tree) : ProtocolReader(quid, tree) {
  auto url = tree->branch_read_data<std::string>("url");

  url_id_ = quid->pmanage<MPtr(&PContainer::make_string)>(
      "remote_url",
      [this](const std::string& val) {
        url_ = val;
        if (url_.empty()) return false;

        osc_address_.reset();
        osc_address_ = std::make_unique<OscAddress>(url_);
        return osc_address_ != nullptr;
      },
      [this]() { return url_; },
      "Remote host URL",
      "URL of the remote host receiving OSC (e.g: osc.udp://localhost:4444)",
      url_);

  if (!url.empty()) quid->pmanage<MPtr(&PContainer::set<std::string>)>(url_id_, url);
}

bool ProtocolOsc::make_properties(Quiddity* quid, const InfoTree* tree) {
  auto props = tree->get_child_keys(".");
  for (auto& it : props) {
    auto continuous = false;
    if (continuous_) {
      continuous = tree->branch_read_data<std::string>(it + ".continuous") == "true";
    }

    auto type = tree->branch_read_data<std::string>(it + ".type");
    auto path = tree->branch_read_data<std::string>(it + ".path");
    auto value = tree->branch_get_value(it + ".value");
    if (!type.empty() && value.is_null()) continue;

    quid->pmanage<MPtr(&PContainer::make_bool)>(
        it,
        [ this, quidptr = quid, it, type, path, value, continuous ](bool val) {
          if (url_.empty()) {
            quidptr->warning("No remote url set, cannot send OSC messages (protocol-mapper).");
            quidptr->message("No remote url set, cannot send OSC messages.");
            return false;
          }

          // Here to keep a constant off state on bool property awaiting for bang property.
          if (continuous) vals_[it] = val;

          if (val) {
            if (continuous) {
              std::lock_guard<std::mutex> lock(ptask_mutex_);
              tasks_.insert(std::make_pair(
                  it,
                  ProtocolReader::Command(
                      [this, type, path, value]() {
                        if (send_osc_code(type, path, value) == -1) {
#ifdef DEBUG
                          std::cerr << "Failed to send OSC message, probably wrong type or value"
                                    << '\n';
#endif
                        }
                      },
                      continuous)));
            } else {
              if (send_osc_code(type, path, value) == -1) {
#ifdef DEBUG
                std::cerr << "Failed to send OSC message, probably wrong type or value" << '\n';
#endif
              }
            }
          } else {
            if (continuous_) {
              std::lock_guard<std::mutex> lock(ptask_mutex_);
              auto found = tasks_.find(it);
              if (found != tasks_.end()) tasks_.erase(found);
            }
          }
          return true;
        },
        [this, it]() { return vals_[it]; },
        tree->branch_read_data<std::string>(it + ".name"),
        tree->branch_read_data<std::string>(it + ".descr"),
        true);
  }
  return true;
}

int ProtocolOsc::send_osc_code(const std::string& type, const std::string& path, const Any& value) {
  int status = -1;

  if (type == "d") {
    status = lo_send(osc_address_->address, path.c_str(), type.c_str(), value.copy_as<double>());
  } else if (type == "f") {
    status = lo_send(osc_address_->address, path.c_str(), type.c_str(), value.copy_as<float>());
  } else if (type == "h") {
    status = lo_send(osc_address_->address, path.c_str(), type.c_str(), value.copy_as<int64_t>());
  } else if (type == "i") {
    status = lo_send(osc_address_->address, path.c_str(), type.c_str(), value.copy_as<int32_t>());
  } else if (type == "s") {
    status = lo_send(
        osc_address_->address, path.c_str(), type.c_str(), value.copy_as<std::string>().c_str());
  } else if (type.empty()) {
    // Some OSC controlled software like Ardour only understand empty messages for some commands.
    auto msg = lo_message_new();
    if (msg) {
      status = lo_send_message(osc_address_->address, path.c_str(), msg);
      lo_message_free(msg);
    }
  } else {
#ifdef DEBUG
    std::cerr << "Ignoring unknown OSC message type " << type << " (protocol-mapper osc)" << '\n';
#endif
  }

  return status;
}

ProtocolOsc::OscAddress::OscAddress(const std::string& url) {
  if (!url.empty()) address = lo_address_new_from_url(url.c_str());
}

ProtocolOsc::OscAddress::~OscAddress() {
  if (address) lo_address_free(address);
}
}
