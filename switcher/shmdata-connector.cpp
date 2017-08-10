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
  quid_->install_method(
      "Connect",
      "connect",
      "connect to a shmdata",
      "success or fail",
      Method::make_arg_description("Shmdata Path", "path", "shmdata path to connect with", nullptr),
      (Method::method_ptr)&ShmdataConnector::connect_wrapped,
      G_TYPE_BOOLEAN,
      Method::make_arg_type_description(G_TYPE_STRING, nullptr),
      this);
  quid_->install_method(
      "Disconnect",
      "disconnect",
      "disconnect a shmdata",
      "success or fail",
      Method::make_arg_description("Shmdata Path", "path", "shmdata path to connect with", nullptr),
      (Method::method_ptr)&ShmdataConnector::disconnect_wrapped,
      G_TYPE_BOOLEAN,
      Method::make_arg_type_description(G_TYPE_STRING, nullptr),
      this);
  quid_->install_method("Disconnect All",
                        "disconnect-all",
                        "disconnect all shmdata reader",
                        "success or fail",
                        Method::make_arg_description("none", nullptr),
                        (Method::method_ptr)&ShmdataConnector::disconnect_all_wrapped,
                        G_TYPE_BOOLEAN,
                        Method::make_arg_type_description(G_TYPE_NONE, nullptr),
                        this);
  quid_->install_method(
      "Can sink caps",
      "can-sink-caps",
      "can we connect with this caps",
      "true or false",
      Method::make_arg_description("String Caps", "caps", "caps as a string", nullptr),
      (Method::method_ptr)&ShmdataConnector::can_sink_caps_wrapped,
      G_TYPE_BOOLEAN,
      Method::make_arg_type_description(G_TYPE_STRING, nullptr),
      this);
  return true;
}

gboolean ShmdataConnector::connect_wrapped(gpointer path, gpointer user_data) {
  ShmdataConnector* context = static_cast<ShmdataConnector*>(user_data);
  if (nullptr == context->on_connect_cb_) {
    context->quid_->warning("on connect callback not installed\n");
    return FALSE;
  }
  if (context->on_connect_cb_((char*)path))
    return TRUE;
  else
    return FALSE;
}

gboolean ShmdataConnector::disconnect_wrapped(gpointer path, gpointer user_data) {
  ShmdataConnector* context = static_cast<ShmdataConnector*>(user_data);
  On_scope_exit {
    context->quid_->prune_tree(std::string(".shmdata.reader.") + static_cast<char*>(path));
  };
  if (nullptr == context->on_disconnect_cb_) {
    context->quid_->warning("on disconnect callback not installed");
    return FALSE;
  }
  if (context->on_disconnect_cb_((char*)path)) {
    return TRUE;
  } else {
    return FALSE;
  }
}

gboolean ShmdataConnector::disconnect_all_wrapped(gpointer /*unused */, gpointer user_data) {
  ShmdataConnector* context = static_cast<ShmdataConnector*>(user_data);
  On_scope_exit {
    auto keys =
        context->quid_->tree<MPtr(&InfoTree::get_child_keys)>(std::string(".shmdata.reader"));
    if (!keys.empty())
      for (auto& it : keys) {
        context->quid_->prune_tree(std::string(".shmdata.reader." + it));
      }
  };
  if (nullptr == context->on_disconnect_all_cb_) {
    context->quid_->warning("on disconnect all callback not installed");
    return FALSE;
  }
  if (context->on_disconnect_all_cb_())
    return TRUE;
  else
    return FALSE;
}

gboolean ShmdataConnector::can_sink_caps_wrapped(gpointer caps, gpointer user_data) {
  ShmdataConnector* context = static_cast<ShmdataConnector*>(user_data);
  if (nullptr == context->on_can_sink_caps_cb_) {
    context->quid_->warning("on disconnect callback not installed");
    return FALSE;
  }
  if (context->on_can_sink_caps_cb_((char*)caps))
    return TRUE;
  return FALSE;
}

}  // namespace switcher
