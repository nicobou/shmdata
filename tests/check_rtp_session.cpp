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

static bool audio_success;
static bool video_success;
static bool do_continue;

static char *user_string = "hello world";

void 
mon_property_cb(std::string subscriber_name, 
		std::string quiddity_name, 
		std::string property_name, 
		std::string value, 
		void *user_data)
{

  //g_print ("%s, %s, %s\n", quiddity_name.c_str (), property_name.c_str (), value.c_str ());
  if (!audio_success && g_strcmp0 (quiddity_name.c_str (), "firstprobe") == 0)
    {
      g_message ("audio received !");
      audio_success = true;
      if (video_success)
	  do_continue = false;
    }
  if (!video_success && g_strcmp0 (quiddity_name.c_str (), "secondprobe") == 0)
    {
      g_message ("video received !");
      video_success = true;
      if (audio_success)
	  do_continue = false;
    }
}

void 
stop_test ()
{
  do_continue=false;
}

int
main (int argc,
      char *argv[])
{
  audio_success = false;
  video_success = false;
  do_continue = true;
  {
    switcher::QuiddityManager::ptr manager = 
      switcher::QuiddityManager::make_manager("rtptest");  
    
    manager->create ("runtime", "runtime");
    
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
    
    //wait 6 sec for the session being created
    usleep (6000000); 

    manager->create ("httpsdp", "uri");
    manager->invoke_va ("uri", "set_runtime", "runtime", NULL);
    manager->invoke_va ("uri",
      			"to_shmdata",
       			"http://localhost:8084/sdp?rtpsession=rtp&destination=local",
       			NULL);
    
    //wait 2 sec for uri to get the stream 
    usleep (2000000);

    manager->create ("fakesink","firstprobe");
    manager->invoke_va ("firstprobe", "set_runtime", "runtime", NULL);
    manager->invoke_va ("firstprobe",
			"connect",
			"/tmp/switcher_rtptest_uri_application_0",
			NULL);
    
    
     manager->create ("fakesink","secondprobe");
     manager->invoke_va ("secondprobe", "set_runtime", "runtime", NULL);
     manager->invoke_va ("secondprobe",
     			"connect",
     			"/tmp/switcher_rtptest_uri_application_1",
     			NULL);
    
     

    manager->make_subscriber ("sub", mon_property_cb, (void *)user_string);
    manager->subscribe_property ("sub","firstprobe","last-message");
    manager->subscribe_property ("sub","secondprobe","last-message");
    
    g_timeout_add (2000, (GSourceFunc) stop_test, NULL);

    while (do_continue)
      {
	usleep (100000);
      }
  }
 
  if (audio_success && video_success)
    return 0;
  else
    return 1;
}



