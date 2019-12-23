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

#include "./shmdata-connector.hpp"
#include "../gst/gst-utils.hpp"
#include "../infotree/information-tree-json.hpp"
#include "../infotree/information-tree.hpp"
#include "../quiddity/quiddity-container.hpp"
#include "../quiddity/quiddity.hpp"
#include "../shmdata/shmdata-stat.hpp"
#include "../utils/scope-exit.hpp"

namespace switcher {

const std::string ShmdataConnector::disabledWhenConnectedMsg =
    "this property is disabled when a stream is connected";

ShmdataConnector::ShmdataConnector(Quiddity* quid) : quid_(quid) {}

bool ShmdataConnector::install_connect_method(OnConnect on_connect_cb,
                                              OnDisconnect on_disconnect_cb,
                                              OnDisconnectAll on_disconnect_all_cb,
                                              CanSinkCaps on_can_sink_caps_cb,
                                              unsigned int max_reader) {
  if (quid_ == nullptr) {
    quid_->warning("ShmdataConnector is created without quiddity");
    return false;
  }
  quid_->graft_tree(".shmdata.max_reader", InfoTree::make(max_reader));
  on_connect_cb_ = on_connect_cb;
  on_disconnect_cb_ = on_disconnect_cb;
  on_disconnect_all_cb_ = on_disconnect_all_cb;
  on_can_sink_caps_cb_ = on_can_sink_caps_cb;
  quid_->mmanage<MPtr(&MContainer::make_method<std::function<bool(std::string)>>)>(
      "connect",
      JSONSerializer::deserialize(
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
        auto quid_name = quid_->get_quiddity_name_from_file_name(path);
        auto suffix = quid_->get_shmdata_name_from_file_name(path);
        if (!quid_name.empty() && !suffix.empty()) {
          auto tree = InfoTree::make();
          tree->graft("name", InfoTree::make(quid_name));
          tree->graft("suffix", InfoTree::make(suffix));
          quid_->information_tree_->graft(std::string(".shmdata.reader.") + path + ".writer", tree);
        }
        return on_connect_cb_(path);
      });

  using connect_quid_t = std::function<bool(std::string, std::string)>;
  quid_->mmanage<MPtr(&MContainer::make_method<connect_quid_t>)>(
      "connect-quid",
      JSONSerializer::deserialize(
          R"(
                  {
                   "name" : "Connect Quiddity",
                   "description" : "connect to a Quiddity shmpath",
                   "arguments" : [
                     {
                        "long name" : "Quiddity name",
                        "description" : "Name of the Quiddity producing the shmdata"
                     }, {
                        "long name" : "shmdata suffix",
                        "description" : "Suffix of the shmdata, for instance audio or video2"
                     }
                   ]
                  }
              )"),
      [this](const std::string& quid_name, const std::string& suffix) {
        if (nullptr == on_connect_cb_) {
          quid_->warning("on connect callback not installed");
          return false;
        }
        auto qrox = quid_->qcontainer_->get_qrox_from_name(quid_name);
        if (!qrox) {
          quid_->warning("quiddity % not found: %", quid_name, qrox.msg());
          return false;
        }
        auto tree = InfoTree::make();
        tree->graft("name", InfoTree::make(quid_name));
        tree->graft("suffix", InfoTree::make(suffix));
        auto path = qrox.get()->make_shmpath(suffix);
        quid_->information_tree_->graft(std::string(".shmdata.reader.") + path + ".writer", tree);
        return on_connect_cb_(path);
      });

  quid_->mmanage<MPtr(&MContainer::make_method<std::function<bool(std::string)>>)>(
      "disconnect",
      JSONSerializer::deserialize(
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

  quid_->mmanage<MPtr(&MContainer::make_method<std::function<bool()>>)>(
      "disconnect-all",
      JSONSerializer::deserialize(
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

  quid_->mmanage<MPtr(&MContainer::make_method<std::function<bool(std::string)>>)>(
      "can-sink-caps",
      JSONSerializer::deserialize(
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

}  // namespace switcher
