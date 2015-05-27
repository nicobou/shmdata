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

#ifndef __SWITCHER_SHMDATA_CONNECTOR_H__
#define __SWITCHER_SHMDATA_CONNECTOR_H__

#include <vector>
#include <unordered_map>
#include "./json-builder.hpp"
#include "./custom-property-helper.hpp"

namespace switcher {
class Quiddity;

class ShmdataConnector
{
 public:
  using OnConnect = std::function<bool(const std::string &/*shmpath*/)>;
  using OnDisconnect = std::function<bool(const std::string &/*shmpath*/)>;
  using OnDisconnectAll = std::function<bool()>;
  using CanSinkCaps = std::function<bool(const std::string &/*caps*/)>;

  explicit ShmdataConnector(Quiddity *quid);
  ShmdataConnector() = delete;
  virtual ~ShmdataConnector(){};
  ShmdataConnector(const ShmdataConnector &) = delete;
  ShmdataConnector &operator=(const ShmdataConnector &) = delete;
  bool install_connect_method(OnConnect on_connect_cb,
                              OnDisconnect on_disconnect_cb,
                              OnDisconnectAll on_disconnect_all_cb,
                              CanSinkCaps on_can_sink_caps_cb,
                              uint max_reader);

 private:
  Quiddity *quid_ {nullptr};
  // methods to install
  OnConnect on_connect_cb_ {nullptr};
  OnDisconnect on_disconnect_cb_ {nullptr};
  OnDisconnectAll on_disconnect_all_cb_ {nullptr};
  CanSinkCaps on_can_sink_caps_cb_ {nullptr};
  static gboolean connect_wrapped(gpointer path, gpointer user_data);
  static gboolean disconnect_wrapped(gpointer path, gpointer user_data);
  static gboolean disconnect_all_wrapped(gpointer /*unused */,
                                         gpointer user_data);
  static gboolean can_sink_caps_wrapped(gpointer caps, gpointer user_data);
};

}  // namespace switcher
#endif
