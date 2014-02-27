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

#include "switcher/quiddity-manager.h"
#include <string>
#include <unistd.h>  //sleep

void 
property_cb(std::string subscriber_name, 
	    std::string quiddity_name, 
	    std::string property_name, 
	    std::string value, 
	    void *user_data)
{
  g_debug ("%s %s %s %s\n",
   	   subscriber_name.c_str (), 
   	   quiddity_name.c_str (), 
   	   property_name.c_str (), 
   	   value.c_str ());
}

int
main ()
{  
  {
    switcher::QuiddityManager::ptr manager = 
      switcher::QuiddityManager::make_manager("check_fakesink");  
    
    manager->make_property_subscriber ("sub", property_cb, NULL);
    
    manager->create ("audiotestsrc", "audio");
    manager->subscribe_property ("sub","audio","shmdata-writers");
    manager->set_property ("audio", "started", "true");
    manager->create ("fakesink", "vu");
    manager->subscribe_property ("sub", "vu", "byte-rate");
    manager->invoke_va ("vu", "connect", NULL, "/tmp/switcher_check_fakesink_audio_audio", NULL);
    manager->invoke_va ("vu", "disconnect", NULL, NULL);
    manager->invoke_va ("vu", "connect", NULL, "/tmp/switcher_check_fakesink_audio_audio", NULL);
    manager->set_property ("audio", "started", "false");
    manager->remove ("vu");
    manager->set_property ("audio", "started", "true");

    manager->create ("fakesink", "vu");
    manager->subscribe_property ("sub", "vu", "byte-rate");
    manager->invoke_va ("vu", "disconnect", NULL, NULL);
    manager->invoke_va ("vu", "connect", NULL, "/tmp/switcher_check_fakesink_audio_audio", NULL);
    manager->remove ("vu");
  }// releasing manager

  //success
  return 0;
}



