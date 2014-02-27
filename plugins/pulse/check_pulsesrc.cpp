/*
 * This file is part of switcher-pulsesrc.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

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

    if (!switcher::QuiddityBasicTest::test_full (manager, "pulsesrc"))
      success = false;
    
    if (!switcher::QuiddityBasicTest::test_full (manager, "pulsesink"))
      success = false;

  }//end of scope is releasing the manager
  
  if (success)
    return 0;
  else
    return 1;
}



