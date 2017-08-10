/*
 * This file is part of switcher-gsoap.
 *
 * switcher-gsoap is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SWITCHER_SOAP_CTRL_SERVER_H__
#define __SWITCHER_SOAP_CTRL_SERVER_H__

#include <memory>
#include <mutex>
#include <thread>
#include "switcher/switcher-wrapper.hpp"
#include "webservices/soapcontrolService.h"

namespace switcher {
class SoapCtrlServer : public SwitcherWrapper {
 public:
  SoapCtrlServer(QuiddityConfiguration&&);
  ~SoapCtrlServer();
  SoapCtrlServer(const SoapCtrlServer&) = delete;
  SoapCtrlServer& operator=(const SoapCtrlServer&) = delete;

  bool set_port(int port);
  bool start();
  // for invocation into soap handlers:
  std::shared_ptr<Switcher> get_quiddity_manager();
  // wrappers
  static gboolean set_port_wrapped(gint port, gpointer user_data);

 private:
  struct soap soap_ {};
  int port_{0};
  bool quit_server_thread_{false};
  controlService* service_{nullptr};
  SOAP_SOCKET socket_{-1};
  std::thread thread_{};
  std::mutex mutex_{};
  void server_thread();
  static int http_get(struct soap* soap);
};

SWITCHER_DECLARE_PLUGIN(SoapCtrlServer);

}  // namespace switcher
#endif
