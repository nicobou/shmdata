/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include "switcher/quiddity-manager-wrapper.h"
#include "webservices/soapcontrolService.h"
#include <thread>
#include <memory>

namespace switcher
{

  class SoapCtrlServer : public QuiddityManagerWrapper
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(SoapCtrlServer);
    SoapCtrlServer ();
    ~SoapCtrlServer ();
    SoapCtrlServer (const SoapCtrlServer &) = delete;
    SoapCtrlServer &operator= (const SoapCtrlServer &) = delete;
    bool init ();

    bool set_port (int port);//default port is 8080
    bool start (); 
    bool stop ();
    //for invocation into soap handlers:
    std::shared_ptr<QuiddityManager> get_quiddity_manager ();
    //wrappers
    static gboolean set_port_wrapped (gint port, gpointer user_data);

  private:
    struct soap soap_;
    int port_;
    bool quit_server_thread_;
    controlService *service_; 
    SOAP_SOCKET socket_;
    std::thread thread_;
    void server_thread ();
    static int http_get (struct soap *soap);
  };

  SWITCHER_DECLARE_PLUGIN(SoapCtrlServer);

}  // end of namespace

#endif // ifndef
