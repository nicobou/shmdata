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
#include <iostream>

switcher::QuiddityManager::ptr manager;

gpointer
set_runtime_invoker (gpointer name)
{
  if (manager->has_method ((char *)name, "set_runtime"))
    manager->invoke_va ((char *)name, "set_runtime", "pipeline0", NULL);
  g_free (name);
  return NULL;
}

void 
quiddity_created_removed_cb (std::string subscriber_name, 
			     std::string quiddity_name, 
			     std::string signal_name, 
			     std::vector<std::string> params, 
			     void *user_data)
{
  g_thread_create (set_runtime_invoker, 
		   g_strdup (params[0].c_str ()),
		   FALSE,
		   NULL);
}


int
main (int argc,
      char *argv[])
{
  bool success = true;
  manager = switcher::QuiddityManager::make_manager("test_manager");  
  
  manager->create ("runtime");
  manager->create ("create_remove_spy", "create_remove_spy");
  manager->make_signal_subscriber ("create_remove_subscriber", quiddity_created_removed_cb, NULL);
  manager->subscribe_signal ("create_remove_subscriber","create_remove_spy","on-quiddity-created");

  std::vector<std::string> classes = manager->get_classes ();
  
  std::vector<std::string>::iterator iter;
  
  for (iter = classes.begin(); iter != classes.end (); ++iter)
    {
      std::string class_name (*iter);
      //std::cout << class_name << std::endl; 
      std::string res = manager->create(class_name, class_name);
      
      if (res.compare (class_name) != 0)
	{
	  g_printerr ("quiddity %s not created properly\n",iter->c_str ());
	}
      else
	if (!manager->remove (class_name))
	  {
	    g_printerr ("error while removing quiddity %s\n",iter->c_str ());
	    success = false;
	  }
    }

  //cleanning manager
  {
    switcher::QuiddityManager::ptr empty;
    manager.swap (empty);
  }
  
  if (success)
    return 0;
  else
    return 1;
}



