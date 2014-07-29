/*
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



#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "switcher/quiddity-manager.h"
#include <vector>
#include <string>
#include <unistd.h>  //sleep

static bool audio_success;
static bool video_success;
static bool do_continue;

void 
mon_property_cb(std::string /*subscriber_name*/, 
		std::string quiddity_name, 
		std::string property_name, 
		std::string value, 
		void */*user_data*/)
{
  // if (g_strcmp0 (property_name.c_str (), "caps") == 0)
  //   g_print ("-caps- %s\n",value.c_str ());

  // g_print ("%s, %s, %s\n", quiddity_name.c_str (), property_name.c_str (), value.c_str ());

  if (!audio_success && g_strcmp0 (quiddity_name.c_str (), "audioprobe") == 0)
    {
      g_message ("audio received !");
      audio_success = true;
      if (video_success)
	do_continue = false;
    }
  if (!video_success && g_strcmp0 (quiddity_name.c_str (), "videoprobe") == 0)
    {
      g_message ("video received !");
      video_success = true;
      if (audio_success)
	do_continue = false;
    }

}

int
main ()
{
  audio_success = false;
  video_success = false;
  do_continue = true;
  {
    switcher::QuiddityManager::ptr manager = 
      switcher::QuiddityManager::make_manager("rtptest");  

#ifdef HAVE_CONFIG_H
    gchar *usr_plugin_dir = g_strdup_printf ("../plugins/gsoap/%s", LT_OBJDIR);
    manager->scan_directory_for_plugins (usr_plugin_dir);
    g_free (usr_plugin_dir);
#else
    return 1;
#endif
    
    manager->create ("SOAPcontrolServer", "soapserver");

    manager->invoke_va ("soapserver", 
			"set_port", 
			nullptr, 
			"38084", 
			nullptr);
    
    //testing uncompressed data transmission
    manager->create ("audiotestsrc","a");
    manager->set_property ("a", "started", "true");
   
    manager->create ("videotestsrc","v");
    manager->set_property ("v", "started", "true");

     manager->create ("rtpsession","rtp");
    
     manager->invoke_va ("rtp", 
       			"add_data_stream",
     			nullptr,
       			"/tmp/switcher_rtptest_a_audio",
       			nullptr);
    
     manager->invoke_va ("rtp", 
			 "add_data_stream",
			 nullptr,
			 "/tmp/switcher_rtptest_v_video",
			 nullptr);
     
     manager->invoke_va ("rtp", 
			 "add_data_stream",
			 nullptr,
			 "/tmp/switcher_rtptest_v_encoded-video",
			 nullptr);
     
     manager->invoke_va ("rtp", 
			 "add_destination",
			 nullptr,
			 "local",
			 "127.0.0.1",
			 nullptr);
     manager->invoke_va ("rtp", 
      			 "add_udp_stream_to_dest",
      			 nullptr,
      			 "/tmp/switcher_rtptest_a_audio",
      			 "local",
      			 "9066",
      			 nullptr);
     manager->invoke_va ("rtp",
			 "add_udp_stream_to_dest",
			 nullptr,
			 "/tmp/switcher_rtptest_v_video",
			 "local",
			 "9076",
			 nullptr);

     usleep (1000000);

     manager->create ("httpsdpdec", "uri");
     manager->invoke_va ("uri", 
     			 "to_shmdata",
     			 nullptr,
     			 "http://127.0.0.1:38084/sdp?rtpsession=rtp&destination=local",
     			 nullptr);
     manager->make_property_subscriber ("sub", mon_property_cb, nullptr);
     manager->create ("fakesink","audioprobe");
     manager->subscribe_property ("sub","audioprobe","caps");
     manager->subscribe_property ("sub","audioprobe","last-message");
     manager->invoke_va ("audioprobe",
     			 "connect",
     			 nullptr,
     			 "/tmp/switcher_rtptest_uri_audio-0",
     			 nullptr);
     manager->create ("fakesink","videoprobe");
     manager->subscribe_property ("sub","videoprobe","last-message");
     manager->subscribe_property ("sub","videoprobe","caps");
     manager->invoke_va ("videoprobe",
       			"connect",
     			nullptr,
       			"/tmp/switcher_rtptest_uri_video-0",
       			nullptr);

     //wait 3 seconds
     uint count = 3;
     while (do_continue)
       {
	 if (count == 0)
	   do_continue = false;
	 else
	   {
	     --count;
	     usleep (1000000);
	   }
       }
  }
 
  if (audio_success && video_success)
    return 0;
  else
    return 1;
}
