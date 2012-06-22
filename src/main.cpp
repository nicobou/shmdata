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

#include "switcher/runtime.h"
#include "switcher/base-entity.h"
#include "switcher/creator.h"
#include "switcher/video-test-source.h"
#include "switcher/ctrl-server.h"
#include <iostream>
#include "switcher/webservices/control.nsmap"

int
main (int argc,
      char *argv[])
{
  using namespace switcher;
  
  //Factory + registration of base entity
  Factory<BaseEntity, std::string> factory;
  factory.Register<Runtime> ("runtime");
  factory.Register<VideoTestSource> ("videotestsource");
  factory.Register<CtrlServer> ("controlserver");
  
  //list available object in factory
  std::vector<std::string> available_object_list = factory.getList ();
  for (uint i=0; i < available_object_list.size (); i++)
    {
      std::cout<< "** available object: " << available_object_list[i] << std::endl; 
    }    

  //creating a webservice 
  BaseEntity::ptr baseserv = factory.Create ("controlserver");
  CtrlServer::ptr serv = std::tr1::dynamic_pointer_cast<CtrlServer> (baseserv);
  serv->set_user_data ((void *)&factory);
  serv->start ();

  //Create a runtime
  BaseEntity::ptr runtime = factory.Create ("runtime");
  //printf("Runtime %u\n", runtime->Get());
  Runtime::ptr rt = std::tr1::dynamic_pointer_cast<Runtime> (runtime);
  
  //create a videotest
  BaseEntity::ptr videotest = factory.Create("videotestsource");
  VideoTestSource::ptr seg = std::tr1::dynamic_pointer_cast<VideoTestSource> (videotest); 
  seg->set_runtime (rt); //..and play
  
  //list registered properties of video test
  videotest->list_properties ();
  
  //print, set and print value of a given property 
  std::cout << "----- pattern  " << videotest->get_property ("pattern") << std::endl ;
  videotest->set_property ("pattern","snow");
  std::cout << "----- pattern  " << videotest->get_property ("pattern") << std::endl ;

  // BaseEntity::ptr myvideotest = factory.Create("videotestsource");
  // VideoTestSource::ptr myseg = std::tr1::dynamic_pointer_cast<VideoTestSource> (myvideotest); 
  // rt->add_segment (myseg);


  //start the runtime (blocking)
  rt->run();
  
  return 0;
}



