/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include "switcher/quiddity-manager.h"
//#include <vector>
#include <string>
//#include <iostream>
#include <unistd.h>  //sleep

void 
property_cb(std::string subscriber_name, 
	    std::string quiddity_name, 
	    std::string property_name, 
	    std::string value, 
	    void *user_data)
{
  g_print ("%s %s %s %s\n",
	   subscriber_name.c_str (), 
	   quiddity_name.c_str (), 
	   property_name.c_str (), 
	   value.c_str ());
}



int
main (int argc,
      char *argv[])
{

  {
    bool success = true;
    switcher::QuiddityManager::ptr manager = 
      switcher::QuiddityManager::make_manager("check_fakesink");  
    
    manager->scan_directory_for_plugins ("/usr/local/switcher-0.2/plugins");
    
    manager->make_property_subscriber ("sub", property_cb, NULL);

    manager->create ("runtime", "runtime");

    manager->create ("audiotestsrc", "audio");
    manager->invoke_va ("audio","set_runtime", NULL, "runtime", NULL);
    manager->subscribe_property ("sub","audio","shmdata-writers");
    manager->set_property ("audio", "started", "true");
    manager->create ("fakesink", "vu");
    manager->invoke_va ("vu","set_runtime", NULL, "runtime", NULL);
    manager->subscribe_property ("sub", "vu", "byte-rate");
    manager->invoke_va ("vu", "connect", NULL, "/tmp/switcher_check_fakesink_audio_audio", NULL);
    // g_print ("connected\n"); 
    // usleep (2000000);
    manager->set_property ("audio", "started", "false");
    //g_print ("stoped\n"); 
    manager->remove ("vu");
    //usleep (2000000);
    manager->set_property ("audio", "started", "true");
    //g_print ("started\n"); 
    //g_print ("vu removing\n");
    // manager->remove ("vu");
    // g_print ("vu removed");
    manager->create ("fakesink", "vu");
    manager->invoke_va ("vu","set_runtime", NULL, "runtime", NULL);
    manager->subscribe_property ("sub", "vu", "byte-rate");
    manager->invoke_va ("vu", "connect", NULL, "/tmp/switcher_check_fakesink_audio_audio", NULL);
    // usleep (5000000);
    // g_print ("last remove\n");
    manager->remove ("vu");


  }// releasing manager

  //success
  return 0;
}



