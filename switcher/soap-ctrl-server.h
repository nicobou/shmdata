/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
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

#include "quiddity-manager-wrapper.h"
#include <memory>

#include "webservices/soapcontrolService.h"

namespace switcher
{

  class SoapCtrlServer : public QuiddityManagerWrapper
  {
  public:
    typedef std::shared_ptr<SoapCtrlServer> ptr;
    ~SoapCtrlServer ();
    void set_port (int port);//default port is 8080
    void start (); 
    void stop ();
    //for invocation into soap handlers:
    std::shared_ptr<QuiddityManager> get_quiddity_manager ();
    //wrappers
    static gboolean set_port_wrapped (gint port, gpointer user_data);

    bool init ();
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

  private:
    struct soap soap_;
    int port_;
    bool quit_server_thread_;
    controlService *service_; 
    GThread *thread_;
    static gpointer server_thread (gpointer user_data);
    static int http_get (struct soap *soap);
  };

}  // end of namespace

#endif // ifndef
