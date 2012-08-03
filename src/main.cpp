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

//#include "switcher/runtime.h"
#include "switcher/video-test-source.h"
#include "switcher/ctrl-server.h"
#include "switcher/webservices/control.nsmap"
#include "switcher/base-entity-manager.h"

#include <iostream>

int
main (int argc,
      char *argv[])
{
  using namespace switcher;
  
  
  BaseEntityManager manager;  
  std::vector<std::string> available_object_list = manager.get_list_of_entity_classes ();
  //list available object in factory
  for (uint i=0; i < available_object_list.size (); i++)
    {
      std::cout<< "** available object: " << available_object_list[i] << std::endl; 
    }    
  
  std::cout << "coucou1" << std::endl ;

  // //creating a SOAP webservice controling the manager
   BaseEntity::ptr baseserv = manager.create_entity ("controlserver");
  // CtrlServer::ptr serv = std::tr1::dynamic_pointer_cast<CtrlServer> (baseserv);
  // serv->set_base_entity_manager (&manager);
  // serv->start ();

  std::cout << "coucou2" << std::endl ;

  // Create a runtime
  BaseEntity::ptr runtime = manager.create_entity ("runtime");

  std::cout << "coucou3" << std::endl ;

  // //create a videotest
  BaseEntity::ptr videotest = manager.create_entity("videotestsource");

  std::vector<std::string> available_method = manager.get_list_of_method_names ("videotestsrc0");
  for (uint i=0; i < available_method.size (); i++)
      std::cout<< "** available method: " << available_method[i] << std::endl; 

  //attaching videotestsrc to the runtime
  std::vector<std::string> empty;
  std::vector<std::string> ent_name;
  ent_name.push_back (runtime->get_name());
  manager.entity_invoke_method_with_name_args ("videotestsrc0","set_runtime",empty,ent_name);

  //start the runtime (blocking)
  Runtime::ptr rt = std::tr1::dynamic_pointer_cast<Runtime> (runtime);
  rt->run();
  
  
  return 0;
}



