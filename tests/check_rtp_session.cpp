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



#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "switcher/quiddity-manager.h"
#include <vector>
#include <string>
#include <unistd.h>  //sleep

static bool audio_success;
static bool video_success;
static bool jpeg_success;
static bool do_continue;

static const char *user_string = "hello world";

void 
mon_property_cb(std::string /*subscriber_name*/, 
		std::string quiddity_name, 
		std::string /*property_name*/, 
		std::string /*value*/, 
		void */*user_data*/)
{
  // if (g_strcmp0 (property_name.c_str (), "caps") == 0)
  //   g_print ("-caps- %s\n",value.c_str ());

  //g_print ("%s, %s, %s\n", quiddity_name.c_str (), property_name.c_str (), value.c_str ());

  jpeg_success = true;
  // if (!jpeg_success && g_strcmp0 (quiddity_name.c_str (), "jpegprobe") == 0)
  //   {
  //     g_message ("jpeg received !");
  //     jpeg_success = true;
  //     if (audio_success && video_success)
  // 	do_continue = false;
  //   }

  if (!audio_success && g_strcmp0 (quiddity_name.c_str (), "audioprobe") == 0)
    {
      g_message ("audio received !");
      audio_success = true;
      if (video_success && jpeg_success)
	do_continue = false;
    }
  if (!video_success && g_strcmp0 (quiddity_name.c_str (), "videoprobe") == 0)
    {
      g_message ("video received !");
      video_success = true;
      if (audio_success && jpeg_success)
	do_continue = false;
    }

}

int
main (int /*argc*/,
      char */*argv*/[])
{
  audio_success = false;
  video_success = false;
  jpeg_success = false;
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
			NULL, 
			"38084", 
			NULL);
    
    //testing uncompressed data transmission
    manager->create ("runtime", "av_runtime");
    manager->create ("audiotestsrc","a");
    manager->invoke_va ("a", "set_runtime", NULL, "av_runtime", NULL);
    manager->set_property ("a", "started", "true");
   
    manager->create ("videotestsrc","v");
    manager->invoke_va ("v", "set_runtime", NULL, "av_runtime", NULL);

    //manager->set_property ("v", "codec", "jpegenc");
    
    manager->set_property ("v", "started", "true");

    manager->create ("runtime", "rtp_runtime");
    manager->create ("rtpsession","rtp");
    manager->invoke_va ("rtp", "set_runtime", NULL, "rtp_runtime", NULL);
    
    manager->invoke_va ("rtp", 
      			"add_data_stream",
			NULL,
      			"/tmp/switcher_rtptest_a_audio",
      			NULL);
    
    manager->invoke_va ("rtp", 
      			"add_data_stream",
			NULL,
      			"/tmp/switcher_rtptest_v_video",
      			NULL);

    manager->invoke_va ("rtp", 
      			"add_data_stream",
			NULL,
      			"/tmp/switcher_rtptest_v_encoded-video",
      			NULL);

    manager->invoke_va ("rtp", 
      			"add_destination",
			NULL,
      			"local",
      			"localhost",
      			NULL);
    manager->invoke_va ("rtp", 
       			"add_udp_stream_to_dest",
			NULL,
       			"/tmp/switcher_rtptest_a_audio",
       			"local",
       			"9066",
       			NULL);
    manager->invoke_va ("rtp",
       			"add_udp_stream_to_dest",
			NULL,
       			"/tmp/switcher_rtptest_v_video",
       			"local",
       			"9076",
       			NULL);

    manager->invoke_va ("rtp",
       			"add_udp_stream_to_dest",
			NULL,
       			"/tmp/switcher_rtptest_v_encoded-video",
       			"local",
       			"9076",
       			NULL);

    usleep (1000000);

    manager->create ("runtime", "receiver_runtime");
    manager->create ("httpsdpdec", "uri");
    manager->invoke_va ("uri", 
			"set_runtime", 
			NULL, 
			"receiver_runtime", NULL);
    manager->invoke_va ("uri", 
       			"to_shmdata",
			NULL,
			"http://localhost:38084/sdp?rtpsession=rtp&destination=local",
			NULL);
    
    manager->make_property_subscriber ("sub", mon_property_cb, (void *)user_string);


    manager->create ("runtime", "probe_runtime");
    manager->create ("fakesink","audioprobe");
    
    manager->subscribe_property ("sub","audioprobe","caps");
    manager->subscribe_property ("sub","audioprobe","last-message");
    manager->invoke_va ("audioprobe", 
			"set_runtime", 
			NULL,
			"probe_runtime", 
			NULL);
    manager->invoke_va ("audioprobe",
			"connect",
     			NULL,
     			"/tmp/switcher_rtptest_uri_audio-0",
     			NULL);

    manager->create ("fakesink","videoprobe");
    manager->subscribe_property ("sub","videoprobe","last-message");
    manager->subscribe_property ("sub","videoprobe","caps");
    manager->invoke_va ("videoprobe", 
			"set_runtime", 
			NULL, 
			"probe_runtime", 
			NULL);
    manager->invoke_va ("videoprobe",
      			"connect",
			NULL,
      			"/tmp/switcher_rtptest_uri_video-0",
      			NULL);

    // manager->create ("fakesink","jpegprobe");
    // manager->subscribe_property ("sub","jpegprobe","last-message");
    // manager->subscribe_property ("sub","jpegprobe","caps");
    // manager->invoke_va ("jpegprobe", 
    // 			"set_runtime", 
    // 			NULL, 
    // 			"probe_runtime", 
    // 			NULL);
    // manager->invoke_va ("jpegprobe",
    //   			"connect",
    // 			NULL,
    //   			"/tmp/switcher_rtptest_uri_video-1",
    //   			NULL);
    
    while (do_continue)
      usleep (100000);
  }
 
  if (audio_success && video_success)
    return 0;
  else
    return 1;
}



