/*
 * This file is part of switcher-pjsip.
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
#include "switcher/quiddity-basic-test.h"
#include <vector>
#include <string>
#include <iostream>
#include <unistd.h>//usleep

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

int
main ()
{

  bool success = true;
  {
    switcher::QuiddityManager::ptr manager = switcher::QuiddityManager::make_manager("siptest");  
    
#ifdef HAVE_CONFIG_H
    gchar *usr_plugin_dir = g_strdup_printf ("./%s", LT_OBJDIR);
    manager->scan_directory_for_plugins (usr_plugin_dir);
    g_free (usr_plugin_dir);
#else
    return 1;
#endif
    
    //do not test
    return 0;

    // if (!switcher::QuiddityBasicTest::test_full (manager, "sip"))
    //   success = false;
    
    //testing uncompressed data transmission
    manager->create ("audiotestsrc","a");
    manager->set_property ("a", "started", "true");
    
    manager->create ("videotestsrc","v");
    manager->set_property ("v", "started", "true");
    
    manager->create ("rtpsession","rtp");
    
    manager->invoke_va ("rtp", 
       			"add_data_stream",
     			nullptr,
       			"/tmp/switcher_siptest_a_audio",
       			nullptr);
    
    manager->invoke_va ("rtp", 
			"add_data_stream",
			nullptr,
			"/tmp/switcher_siptest_v_video",
			nullptr);
    
    // manager->invoke_va ("rtp", 
    // 			"add_destination",
    // 			nullptr,
    // 			"local",
    // 			"127.0.0.1",
    // 			nullptr);
    
    usleep (2000000);
    
    //SIP
    if (0 != manager->create ("sip", "test").compare ("test"))
      {
	g_print ("cannot create\n");
	return 1;
      }
    
    manager->set_property ("test", "port", "5070"); 

    if (!manager->invoke_va ("test","register", nullptr, 
			     "1004", //user
			     "10.10.30.179", //domain
			     "1234", //password
			     nullptr))
      {
	g_print ("cannot register \n");
	return 1;
      }

    manager->set_property ("test", "rtp-session", "rtp");

    manager->invoke_va ("test",
			"call",
			nullptr,
			"sip:1002@10.10.30.223",
    			nullptr);


    usleep (2000000);

    manager->set_property ("test","status", "Away");

    usleep (2000000);


    manager->set_property ("test","status-note", "coucou");

    usleep (2000000);

    manager->set_property ("test","status", "BRB");

    usleep (2000000);
    manager->invoke_va ("test",
			"hang-up",
			nullptr,
			"sip:1002@10.10.30.223",
			nullptr);

    usleep (20000000);

    manager->remove ("test");
     
  }//end of scope is releasing the manager
   
  if (success)
    return 0;
  else
    return 1;
}



