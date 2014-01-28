/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher-gtk.
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
#include <unistd.h>  //sleep

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

int
main ()
{
  {
    switcher::QuiddityManager::ptr manager = switcher::QuiddityManager::make_manager("gtktest");  
    
#ifdef HAVE_CONFIG_H
    gchar *usr_plugin_dir = g_strdup_printf ("./%s", LT_OBJDIR);
    manager->scan_directory_for_plugins (usr_plugin_dir);
    g_free (usr_plugin_dir);
#else
    return 1;
#endif

     //creating a "myplugin" quiddity
     if (manager->create("gtkvideosink", "win").compare ("win") != 0)
       {
     	//cannot create gtk window, stoping the test
     	return 0;
       }
    
     //creating a video source quiddity
     if (manager->create("videotestsrc", "vid").compare ("vid") != 0)
       return 1;
	 
     if (!manager->set_property ("vid", "started", "true"))
       return 1;
     
     //usleep (1000000);
     
     //connecting 
     if (!manager->invoke_va ("win", "connect", NULL, "/tmp/switcher_gtktest_vid_video", NULL))
       return 1;

     //usleep (10000000);

     //removing quiddities
     if (!manager->remove ("win"))
       return 1;

     if (!manager->remove ("vid"))
       return 1;

     if (!switcher::QuiddityBasicTest::test_full (manager, "gtkvideosink"))
       return 1;

  }//end of scope is releasing the manager

  return 0;//success
}



