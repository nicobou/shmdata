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

#include "./connector.hpp"

#include "../gst/utils.hpp"
#include "../infotree/information-tree.hpp"
#include "../infotree/json-serializer.hpp"
#include "../quiddity/container.hpp"
#include "../quiddity/quid-id-t.hpp"
#include "../quiddity/quiddity.hpp"
#include "../shmdata/stat.hpp"
#include "../utils/scope-exit.hpp"

namespace switcher {
namespace shmdata {

const std::string Connector::disabledWhenConnectedMsg =
    "this property is disabled when a stream is connected";
const std::string Connector::disabledWhenDisconnectedMsg =
    "this property is disabled when no stream is connected";

Connector::Connector(quiddity::Quiddity* quid) : quid_(quid) {}

bool Connector::install_connect_method(OnConnect on_connect_cb,
                                       OnDisconnect on_disconnect_cb,
                                       OnDisconnectAll on_disconnect_all_cb,
                                       CanSinkCaps on_can_sink_caps_cb,
                                       unsigned int max_reader) {
  if (quid_ == nullptr) {
    quid_->warning("Connector is created without quiddity");
    return false;
  }
  quid_->graft_tree(".shmdata.max_reader", InfoTree::make(max_reader));
  on_connect_cb_ = on_connect_cb;
  on_disconnect_cb_ = on_disconnect_cb;
  on_disconnect_all_cb_ = on_disconnect_all_cb;
  on_can_sink_caps_cb_ = on_can_sink_caps_cb;
  quid_->mmanage<MPtr(&quiddity::method::MBag::make_method<std::function<bool(std::string)>>)>(
      "connect",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Connect",
                   "description" : "connect to a shmdata",
                   "arguments" : [
                     {
                        "long name" : "Shmdata Path",
                        "description" : "shmdata path to connect with"
                     }
                   ]
                  }
              )"),
      [this](const std::string& path) {
        if (nullptr == on_connect_cb_) {
          quid_->warning("on connect callback not installed\n");
          return false;
        }
        return on_connect_cb_(path);
      });

  using connect_quid_t = std::function<bool(quiddity::qid_t id, std::string)>;
  quid_->mmanage<MPtr(&quiddity::method::MBag::make_method<connect_quid_t>)>(
      "connect-quid",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Connect a Quiddity",
                   "description" : "connect to a Quiddity shmpath",
                   "arguments" : [
                     {
                        "long name" : "The quiddity id",
                        "description" : "The quiddity id given by the switcher"
                     }, {
                        "long name" : "shmdata suffix",
                        "description" : "Suffix of the shmdata, for instance audio or video2"
                     }
                   ]
                  }
              )"),
      [this](quiddity::qid_t id, const std::string& suffix) {
        if (nullptr == on_connect_cb_) {
          quid_->warning("on connect callback not installed");
          return false;
        }
        auto qrox = quid_->qcontainer_->get_qrox(id);
        if (!qrox) {
          quid_->warning("quiddity % not found: %", std::to_string(id), qrox.msg());
          return false;
        }
        auto path = qrox.get()->make_shmpath(suffix);
        return on_connect_cb_(path);
      });

  quid_->mmanage<MPtr(&quiddity::method::MBag::make_method<std::function<bool(std::string)>>)>(
      "disconnect",
      infotree::json::deserialize(
          R"(
                  {
                   "name" :  "Disconnect",
                   "description" : "disconnect a shmdata",
                   "arguments" : [
                     {
                        "long name" : "Shmdata Path",
                        "description" : "shmdata path to disconnect"
                     }
                   ]
                  }
              )"),
      [this](const std::string& path) {
        if (nullptr == on_disconnect_cb_) {
          quid_->warning("on disconnect callback not installed");
          return false;
        }
        return on_disconnect_cb_(path);
      });

  quid_->mmanage<MPtr(&quiddity::method::MBag::make_method<std::function<bool()>>)>(
      "disconnect-all",
      infotree::json::deserialize(
          R"(
                  {
                   "name" :  "Disconnect All",
                   "description" : "disconnect all shmdata reader",
                   "arguments" : []
                  }
              )"),
      [this]() {
        if (nullptr == on_disconnect_all_cb_) {
          quid_->warning("on disconnect all callback not installed");
          return false;
        }
        return on_disconnect_all_cb_();
      });

  quid_->mmanage<MPtr(&quiddity::method::MBag::make_method<std::function<bool(std::string)>>)>(
      "can-sink-caps",
      infotree::json::deserialize(
          R"(
                  {
                   "name" : "Can sink caps",
                   "description" : "can we connect with this caps",
                   "arguments" : [
                     {
                        "long name" : "String Caps",
                        "description" : "caps as a string"
                     }
                   ]
                  }
              )"),
      [this](const std::string& caps) {
        if (nullptr == on_can_sink_caps_cb_) {
          quid_->warning("on disconnect callback not installed");
          return false;
        }
        return on_can_sink_caps_cb_(caps);
      });
  return true;
}

}  // namespace shmdata
}  // namespace switcher
