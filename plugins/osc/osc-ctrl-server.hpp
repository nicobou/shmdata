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

#ifndef __SWITCHER_OSC_CTRL_SERVER_H__
#define __SWITCHER_OSC_CTRL_SERVER_H__

#include <map>
#include <memory>
#include <string>
#include "lo/lo.h"
#include "switcher/switcher-wrapper.hpp"

namespace switcher {
class OscCtrlServer : public SwitcherWrapper {
 public:
  OscCtrlServer(QuiddityConfiguration&&);
  ~OscCtrlServer();
  OscCtrlServer(const OscCtrlServer&) = delete;
  OscCtrlServer& operator=(const OscCtrlServer&) = delete;
  void set_port(const std::string& port);
  // for invocation into osc handlers:
  std::shared_ptr<Switcher> get_quiddity_manager();
  // wrappers
  static gboolean set_port_wrapped(gpointer port, gpointer user_data);

 private:
  std::string port_;
  std::map<std::string, std::pair<std::string, std::string>> osc_subscribers_;  //(host + port)
  lo_server_thread osc_thread_;


  void start();
  void stop();
  static void prop_cb(const std::string& subscriber_name,
                      const std::string& quiddity_name,
                      const std::string& property_name,
                      const std::string& value,
                      void* user_data);
  static int osc_handler(
      const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
  static void osc_error(int num, const char* msg, const char* path);
  static gchar* string_from_osc_arg(char types, lo_arg* data);
  static gchar* string_float_to_string_int(const gchar* string_float);
  gchar* make_internal_subscriber_name(const gchar* name);
  gchar* retrieve_subscriber_name(const gchar* internal_name);
};

SWITCHER_DECLARE_PLUGIN(OscCtrlServer);

}  // namespace switcher
#endif
