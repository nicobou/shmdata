/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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


#ifndef __SWITCHER_CTRL_SERVER_H__
#define __SWITCHER_CTRL_SERVER_H__

#include "switcher/quiddity-manager.h"
#include <memory>

#include "switcher/webservices/soapcontrolService.h"

namespace switcher
{

  class CtrlServer : public Quiddity
  {
  public:
    typedef std::shared_ptr<CtrlServer> ptr;
    CtrlServer();
    CtrlServer(QuiddityLifeManager::ptr life_manager);
    ~CtrlServer ();
    void set_quiddity_manager (QuiddityManager::ptr manager);
    void set_port (int port);//default port is 8080
    void start (); 
    void stop ();
    
    static QuiddityDocumentation get_documentation ();

  private:
    void make_ctrlserver ();
    static QuiddityDocumentation doc_;
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
