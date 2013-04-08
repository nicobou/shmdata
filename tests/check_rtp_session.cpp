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

#include "switcher/quiddity-manager.h"
#include <vector>
#include <string>
#include <unistd.h>  //sleep

static bool success;
static char *user_string = "hello world";

void 
mon_property_cb(std::string quiddity_name, 
		std::string property_name, 
		std::string value, 
		void *user_data)
{

  g_print ("%s, %s, %s\n", quiddity_name.c_str (), property_name.c_str (), value.c_str ());
  success = true;
}

static void 
logger_cb (std::string quiddity_name, std::string property_name, std::string value, void *user_data)
{
  g_print ("%s\n", value.c_str());
}


int
main (int argc,
      char *argv[])
{
  success = false;
  
  {
    switcher::QuiddityManager::ptr manager = 
      switcher::QuiddityManager::make_manager("rtptest");  
    
    manager->create ("runtime", "runtime");

    //create logger managing switcher log domain
    manager->create ("logger", "internal_logger");
    //manage logs from shmdata
    manager->invoke_va ("internal_logger", "install_log_handler", "shmdata", NULL);
    //manage logs from GStreamer
    manager->invoke_va ("internal_logger", "install_log_handler", "GStreamer", NULL);
     //manage logs from Glib
     manager->invoke_va ("internal_logger", "install_log_handler", "Glib", NULL);
     //manage logs from Glib-GObject
     manager->invoke_va ("internal_logger", "install_log_handler", "Glib-GObject", NULL);
     manager->set_property ("internal_logger", "mute", "false");
     manager->set_property ("internal_logger", "debug", "true");
     //subscribe to logs:
     manager->make_subscriber ("log_sub", logger_cb, NULL);
     manager->subscribe_property ("log_sub","internal_logger","last-line");
     
    manager->create ("SOAPcontrolServer", "soapserver");
    manager->invoke_va ("soapserver", "set_port", "8084", NULL);

    //testing uncompressed data transmission
    manager->create ("audiotestsrc","a");
    manager->invoke_va ("a", "set_runtime", "runtime", NULL);
    manager->create ("videotestsrc","v");
    manager->invoke_va ("v", "set_runtime", "runtime", NULL);
    manager->create ("rtpsession","rtp");
    manager->invoke_va ("rtp", "set_runtime", "runtime", NULL);
    
    manager->invoke_va ("rtp",
			"add_data_stream",
			"/tmp/switcher_rtptest_a_audio",
			NULL);
    manager->invoke_va ("rtp",
			"add_data_stream",
			"/tmp/switcher_rtptest_v_video",
			NULL);
    manager->invoke_va ("rtp",
			"add_destination",
			"local",
			"localhost",
			NULL);
    manager->invoke_va ("rtp",
     			"add_udp_stream_to_dest",
     			"/tmp/switcher_rtptest_a_audio",
     			"local",
     			"9066",
     			NULL);
    manager->invoke_va ("rtp",
     			"add_udp_stream_to_dest",
     			"/tmp/switcher_rtptest_v_video",
     			"local",
     			"9076",
     			NULL);
    
    //wait 2 sec for the session being created
    usleep (2000000); 

    manager->create ("httpsdp", "uri");
    manager->invoke_va ("uri", "set_runtime", "runtime", NULL);
    manager->invoke_va ("uri",
			"to_shmdata",
     			"http://localhost:8084/sdp?rtpsession=rtp&destination=local",
     			NULL);
    
   
    usleep (2000000);
    
    // manager->create ("fakesink","firstprobe");
    // manager->invoke_va ("firstprobe", "set_runtime", "runtime", NULL);

    // manager->invoke_va ("firstprobe",
    // 			"connect",
    // 			"/tmp/switcher_rtptest_uri_application_0",
    // 			NULL);
    
    // usleep (2000000);
    
    //  manager->create ("fakesink","secondprobe");
    //  manager->invoke_va ("secondprobe", "set_runtime", "receiver_runtime", NULL);
    //  manager->invoke_va ("secondprobe",
    // 			 "connect",
    // 			 "/tmp/switcher_rtptest_uri_application_1",
    // 			 NULL);
     
    //  manager->make_subscriber ("sub", mon_property_cb, (void *)user_string);
    //  manager->subscribe_property ("sub","firstprobe","last-message");
    //  manager->subscribe_property ("sub","secondprobe","last-message");

    //  manager->unsubscribe_property ("sub","firstprobe","last-message");
    //  manager->unsubscribe_property ("sub","secondprobe","last-message");
   
    //  usleep (2000000);
    //manager->remove ("firstprobe");
    g_print ("**************************************************************\n");
    //manager->remove ("uri");
 
  }

  return 0;

  if (success)
    return 0;
  else
    return 1;
}



