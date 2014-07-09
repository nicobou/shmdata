/*
 * This file is part of switcher-top.
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
#include <chrono>
#include <vector>
#include <string>
#include <iostream>
#include <thread>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

void 
quiddity_created_removed_cb (std::string /*subscriber_name*/, 
			     std::string quiddity_name, 
			     std::string signal_name, 
			     std::vector<std::string> params, 
			     void *user_data)
{
  g_print ("%s: %s\n", signal_name.c_str (), params[0].c_str ());
  switcher::QuiddityManager* ctx = static_cast<switcher::QuiddityManager*>(user_data);
  std::cout << ctx->get_info (quiddity_name, params[0]) << std::endl;
}

int
main ()
{
   bool success = true;

   {
      switcher::QuiddityManager::ptr manager = switcher::QuiddityManager::make_manager("test_manager");  

  #ifdef HAVE_CONFIG_H
      gchar *usr_plugin_dir = g_strdup_printf ("./%s", LT_OBJDIR);
      manager->scan_directory_for_plugins (usr_plugin_dir);
      g_free (usr_plugin_dir);
  #else
      return 1;
  #endif

      if (!switcher::QuiddityBasicTest::test_full (manager, "syphon"))
        success = false;

      ////creating a "top" quiddity
      //if (g_strcmp0 (manager->create("systemusage", "test").c_str (), "test") != 0)
      //  success = false;

      //if (!manager->set_property ("test", "period", "0.5"))
      //  success = false;
      //
      //manager->make_signal_subscriber ("signal_subscriber", quiddity_created_removed_cb, manager.get ());
      //manager->subscribe_signal ("signal_subscriber","test","on-tree-grafted");
      //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
   }//end of scope is releasing the manager

   if (success)
     return 0;
   else
     return 1;
}



