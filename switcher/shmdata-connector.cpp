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
#include "./gst-utils.hpp"
#include "./information-tree-json.hpp"
#include "./information-tree.hpp"
#include "./quiddity.hpp"
#include "./scope-exit.hpp"

namespace switcher {

const std::string ShmdataConnector::disabledWhenConnectedMsg =
    "this property is disabled when a stream is connected";

ShmdataConnector::ShmdataConnector(Quiddity* quid) : quid_(quid) {}

bool ShmdataConnector::install_connect_method(OnConnect on_connect_cb,
                                              OnDisconnect on_disconnect_cb,
                                              OnDisconnectAll on_disconnect_all_cb,
                                              CanSinkCaps on_can_sink_caps_cb,
                                              uint max_reader) {
  if (quid_ == nullptr) {
    quid_->warning("ShmdataConnector is created without quiddity");
    return false;
  }
  InfoTree::ptr tree = InfoTree::make();
  tree->graft(".max_reader", InfoTree::make(max_reader));
  quid_->graft_tree(".shmdata", tree);
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
        On_scope_exit { quid_->prune_tree(std::string(".shmdata.reader.") + path); };
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
        On_scope_exit {
          auto keys = quid_->tree<MPtr(&InfoTree::get_child_keys)>(std::string(".shmdata.reader"));
          if (!keys.empty())
            for (auto& it : keys) {
              quid_->prune_tree(std::string(".shmdata.reader." + it));
            }
        };
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
