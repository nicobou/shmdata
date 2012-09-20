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

#include "switcher/base-entity-manager.h"
#include <memory>

#include "switcher/webservices/soapcontrolService.h"

namespace switcher
{

  class CtrlServer : public BaseEntity
  {
  public:
    typedef std::tr1::shared_ptr<CtrlServer> ptr;
    CtrlServer();
    ~CtrlServer ();
    //void set_user_data (void *user_data);
    void set_base_entity_manager (BaseEntityManager *manager);
    void set_port (int port);//default port is 8080
    void start (); 
    void stop ();
    
    static BaseEntityDocumentation get_documentation ();

  private:
    static BaseEntityDocumentation doc_;
    struct soap soap_;
    int port_;
    controlService *service_; 
    GThread *thread_;
    static gpointer server_thread (gpointer user_data);
  };

}  // end of namespace

#endif // ifndef
